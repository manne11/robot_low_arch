
#include "rclcpp/rclcpp.hpp"

#include "./translation_unit.hpp"

#include <termios.h>
#include <string.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


using namespace std::chrono_literals;
using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

namespace RobotLocomotion
{

 TranslationUnit::TranslationUnit()
    : Node("TranslationUnit")
    , read_buffer_size_{12}
    , pub_topic_speed ("/serial/speeds")
    , pub_topic_encoder ("/serial/encoders")  
    , straight_move_(true)
    , curve_move_(false)
    , write_speed_to_motor_(false)
    , speed_msg__read(std::make_shared<rtodometry_msgs::msg::StampedSpeeds>())
    , encoder_msg__read(std::make_shared<rtodometry_msgs::msg::StampedEncoders>())
    , speed_msg__write(std::make_shared<rtodometry_msgs::msg::StampedSpeeds>())
    , min_ang_vel(0.05)
    , min_lin_vel(0.2)
    , max_speed_robot(1.5)
    , motor1_max_rpm(3000)
    , motor2_max_rpm(3000)
    , gear_ratio(12)
    , wheel_base_length(0.2318*2)
    , wheel_diff(1.0068)
    , wheel_diameter(0.12434)
    , calibration_left(2.0 / (wheel_diff + 1.0))
    , calibration_right(2.0 / ( (1.0/wheel_diff) + 1.0))
    , m1_factor(1000.0 / (motor1_max_rpm / gear_ratio))
    , m2_factor(1000.0 / (motor2_max_rpm / gear_ratio))
    , motor_cmd("!M 0 0")
    , realtime_qos_profile{rmw_qos_profile_default}  
    , teleop_setpoint_qos_profile{rmw_qos_profile_default}  
    , memory_strategy__subscriber_vel_(std::make_shared<MessagePoolMemoryStrategy
				                       <geometry_msgs::msg::Twist, 1>>())  
  {
     // timer_readwheelspeed_ = this->create_wall_timer(
     //  				       publish_period_readspeed_,
     //  				       std::bind(&TranslationUnit::timer_callback_readspeed_or_readspeed_writespeed, this)  );

      
      // Joystick
    //   subscriber_teleop_    = this->create_subscription<sensor_msgs::msg::Joy>(
    // "joy",
    // std::bind(&TranslationUnit::teleop_subscription_callback, this, std::placeholders::_1));

#ifdef REAL_TIME_613710284348

      // set the realtime_profile
      realtime_qos_profile.reliability         = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      realtime_qos_profile.history             = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
      realtime_qos_profile.depth               = 1;

      teleop_setpoint_qos_profile =       realtime_qos_profile;
      teleop_setpoint_qos_profile.durability = 	RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

      // use the subscriber with the qos_profile and memory pool strat
      subscriber_vel_    = this->create_subscription<geometry_msgs::msg::Twist>
      	(
      	 "/turtle1/cmd_vel",
      	 std::bind(&TranslationUnit::vel_subscription_callback, this, std::placeholders::_1),
	 teleop_setpoint_qos_profile,
      	 nullptr,
      	 false,
	 memory_strategy__subscriber_vel_	 );

      


#else

      subscriber_vel_    = this->create_subscription<geometry_msgs::msg::Twist>
	(
	 "/turtle1/cmd_vel",
	 std::bind(&TranslationUnit::vel_subscription_callback, this, std::placeholders::_1));

      
#endif
      
      // set the buffer elements in the array to zero
      std::memset(buffer_12_, 0, sizeof buffer_12_);

      // clearing the timespec structures
      std::memset(&time_now_speed_, 0, sizeof(timespec));
      std::memset(&time_prev_speed_, 0, sizeof(timespec));
      std::memset(&time_now_encoder_, 0, sizeof(timespec));
      std::memset(&time_prev_encoder_, 0, sizeof(timespec)); 

  }
  

  void TranslationUnit::set_publisher__translational_data_speeds()
    {
      // puts("set_publisher_translational_data_speeds");
	   
     #ifdef REAL_TIME_613710284348

      Translation_pub_speed_ = this->create_publisher<rtodometry_msgs::msg::StampedSpeeds>
        (pub_topic_speed,
      	 realtime_qos_profile
      );
     #else
      Translation_pub_speed_ = this->create_publisher<rtodometry_msgs::msg::StampedSpeeds>
        (pub_topic_speed
      );
     #endif

    }

  void TranslationUnit::set_publisher__translational_data_encoders()
    {
      // puts("set_publisher_translational_data_encoders");
     #ifdef REAL_TIME_613710284348

            Translation_pub_encoder_ = this->create_publisher<rtodometry_msgs::msg::StampedEncoders>
	      (pub_topic_encoder,
	       realtime_qos_profile
	       );

     #else
	    Translation_pub_encoder_ = this->create_publisher<rtodometry_msgs::msg::StampedEncoders>
	      (pub_topic_encoder
	       );
     #endif


 
    }  
  
  void TranslationUnit::timer_callback_readspeed_or_readspeed_writespeed()
  {
    counter_triggering_write_80ms++;





    if( counter_triggering_write_80ms >= 40)
      {
	set_velocity_to_robot();
    	counter_triggering_write_80ms = 0;
      }else
      {
        read__wheel_speed();
      }
    
    
  }

  bool TranslationUnit::parse__recieved_data()
  {
    // puts("parse__received_data");

    found_match_ = false;
    switch ( buffer_12_[0]) {


    case '#':
      {
	if (buffer_12_[1] == 'S')
	  {
	    parser__sscanf_return_value_speed = sscanf(buffer_12_, "#S=%ld:%ld#", & speed_left_, &speed_right_);

	    if(parser__sscanf_return_value_speed == 2)
	      {
		publishSpeedData( speed_left_, speed_right_);
		found_match_ = true;
		parser__sscanf_return_value_speed = -1;

	      }
	  }
	break;
      }
      
    case 'S':
      {

	parser__sscanf_return_value_speed = sscanf(buffer_12_, "S=%ld:%ld#", & speed_left_, &speed_right_);

	if(parser__sscanf_return_value_speed == 2)
	      {
		publishSpeedData( speed_left_, speed_right_);
		found_match_ = true;
		parser__sscanf_return_value_speed = -1;

	      }
	break;
      }
      


	
    case '+':
      {


      
 	if (buffer_12_[3] == 'S')
	  {
	  
	    parser__sscanf_return_value_speed = sscanf(buffer_12_, "+\r#S=%ld:%ld#", & speed_left_, &speed_right_);

	    if (parser__sscanf_return_value_speed == 2)
	    {
	      publishSpeedData( speed_left_, speed_right_);
	      found_match_ = true;
	      parser__sscanf_return_value_speed = -1;
	    } 	    
	
	    

	  }
	
	  break;	  
      }









    default:
      break;
      
    }

    if(found_match_ == false)
      {
    	temp__messages_not_parsed++;
      }
    else
      {
    	temp__messages_parsed++;
      } 
    
    return found_match_;
    
  }

  void TranslationUnit::publishSpeedData( const int64_t speed_left_, const int64_t speed_right_)
    {
      // puts("PublishSpeedData");

    clock_gettime(CLOCK_MONOTONIC_RAW, &time_now_speed_);

    
    dt__speed_           = ( time_now_speed_.tv_sec
                  -  time_prev_speed_.tv_sec)*1000.0
            
            
                            + (time_now_speed_.tv_nsec -
                    time_prev_speed_.tv_nsec)/ 1000000.0;


    time_prev_speed_                    = time_now_speed_;	
    speed_msg__read->speeds.left_wheel  = speed_left_;
    speed_msg__read->speeds.right_wheel = speed_right_;
    speed_msg__read->speeds.time_delta  = dt__speed_;
    
    Translation_pub_speed_ -> publish(speed_msg__read);

    
    }

  void TranslationUnit::publishEncoderData(const int64_t encoder_left_, const int64_t encoder_right_)
  {                        // clock that prints the time between consecutive samples

    clock_gettime(CLOCK_MONOTONIC_RAW, &time_now_encoder_);

  
    dt__encoder_           = ( time_now_encoder_.tv_sec
  			     -  time_prev_encoder_.tv_sec)*1000.0
          
          
                             + (time_now_encoder_.tv_nsec -
                		 time_prev_encoder_.tv_nsec)/ 1000000.0;


   time_prev_encoder_                      = time_now_encoder_;	
   encoder_msg__read->encoders.left_wheel  = encoder_left_;
   encoder_msg__read->encoders.right_wheel = encoder_right_;
   encoder_msg__read->encoders.time_delta  = dt__encoder_;
   
  Translation_pub_encoder_ -> publish(encoder_msg__read);

  }

  void TranslationUnit::teleop_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    velocity = msg -> axes[1];
    angular_z = msg -> axes[0];
    set_velocity_to_robot();
  }

  void TranslationUnit::vel_subscription_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {

    // Subscriber to velocity commands from the controller / controller simulation
    if(msg-> linear.z == 0) {
    velocity = msg -> linear.x;	
    angular_z = msg -> angular.z;
    } else if (msg-> linear.z == 1) {

      stop_motor = true;
      velocity = 0;
      angular_z = 0;
    }
  }
  
  void TranslationUnit::timer_callback_setbool_writespeed()
  {
    write_speed_to_motor_ = true;
  }


  void TranslationUnit::port_heartbeat()
  {

    std::cout << "port exists" << std::endl;
  }

  void TranslationUnit::port_connect()
  {
    
    open_serial_port("/dev/ttyXR0");
    set_input_mode();
    config__motor_controller();
    
    std::cout << "connected to the port" << std::endl;

  }

  void TranslationUnit::port_disconnect()
  {
    std::cout << "disconnected to port" << std::endl;
    close(fd_serial_);
    tcsetattr (fd_serial_, TCSANOW, &saved_attributes_);
    
    std::cout << "total number of messages: " << temp__messages_parsed + temp__messages_not_parsed
	      << std::endl;
    std::cout << " messages parsed: " << temp__messages_parsed
	      << std::endl;
    std::cout << " messages not parsed: " << temp__messages_not_parsed
	      << std::endl;
    
  }


  void TranslationUnit::min_saturation_velocity()
  {
    zero_cnt = 0;

    if (std::fabs(velocity) < 1.0e-6)
    {
      velocity = 0.0;
      zero_cnt++;
    }

    if (std::fabs(angular_z) < 1.0e-6)
    {
      angular_z = 0.0;
      zero_cnt++;
    }

    if ( zero_cnt == 2 )
    {
      r_speed = 0.0;
      l_speed = 0.0;
    }
    else
    {
      if ( zero_cnt == 1 ) // if just one is zero, check for minimum values
      {
        if (velocity == 0.0 && std::fabs(angular_z) < min_ang_vel)
        {
          if ( angular_z > 0.0 )
            angular_z = min_ang_vel;
          else
            angular_z = -min_ang_vel;
        }
        else if ( angular_z == 0.0 && std::fabs(velocity) < min_lin_vel )
        {
          if ( velocity > 0.0 )
            velocity = min_lin_vel;
          else
            velocity = -min_lin_vel;
          
        }
      
      }

  }
  }

  void TranslationUnit::max_saturation_velocity()
  {

    // Limit maximum speeds of robot
    if (rws > max_speed_robot) { 
        rws = max_speed_robot;
      }
      else if (rws < -max_speed_robot) {
        rws = -max_speed_robot;
      }

      if (lws > max_speed_robot) { 
        lws = max_speed_robot;
      }
      else if (lws < -max_speed_robot) {
        lws = -max_speed_robot;
      }
  }

  void TranslationUnit::set_velocity_to_robot()
  {
    min_saturation_velocity();

    V2 = 2.0*velocity;
    B_Th = wheel_base_length * angular_z;

    // convert code to target_speed and target_direction
    rws = calibration_right * (V2 + B_Th) * 0.5;	// unit: m/s
    lws = calibration_left * (V2 - B_Th) * 0.5;	// unit: m/s

    max_saturation_velocity();

    r_speed = 60.0 * rws /(M_PI * wheel_diameter); // unit: rpm
    l_speed  = 60.0 * lws /(M_PI * wheel_diameter); // unit: rpm

    m1_cmd = l_speed*m1_factor;
	  m2_cmd = r_speed*m2_factor;
    motor_cmd_saturation();
    
    motor_cmd = (SSTR("!M " << (int)m1_cmd << " " << (int)m2_cmd)+std::string(1,'\r')) .c_str();
    
    // TO DO : FILL THIS FUNCTION
    //write__wheel_speed();
    //std::cout << "Motor Command Sent: " << motor_cmd << std::endl;
    if( counter_triggering_write_80ms >= 40)
      {

	write_to_port(motor_cmd);
	counter_triggering_write_80ms = 0;
      }

  }

  void TranslationUnit::read__wheel_speed()
  {
    read_from_port();
    parse__recieved_data();
  }

  

  void TranslationUnit::motor_cmd_saturation()
  {
    if (m1_cmd > 1000.0)
  {
	  //ROS_WARN_STREAM_THROTTLE(1.0, "Trying to set M1 speed too high! it will be throttled!");
	  scale_factor = 1000.0 / m1_cmd;
	  m1_cmd = 1000;
	  m2_cmd *= scale_factor;
  }
  else if (m1_cmd < -1000.0)
  {
	  //ROS_WARN_STREAM_THROTTLE(1.0, "Trying to set M1 speed too high!! it will be throttled!");
	  scale_factor = -1000.0 / m1_cmd;
	  m1_cmd = -1000;
	  m2_cmd *= scale_factor;
  }

  if (m2_cmd > 1000.0)
  {
	  //ROS_WARN_STREAM_THROTTLE(1.0, "Trying to set M2 speed too high! it will be throttled!");
	  scale_factor = 1000.0 / m2_cmd;
	  m2_cmd = 1000;
	  m1_cmd *= scale_factor;
  }
  else if (m2_cmd < -1000.0)
  {
	  //ROS_WARN_STREAM_THROTTLE(1.0, "Trying to set M2 speed too high!! it will be throttled!");
	  scale_factor = -1000.0 / m2_cmd;
	  m2_cmd = -1000;
	  m1_cmd *= scale_factor;
  }
  }

  
  void TranslationUnit::set_input_mode ()
  {
    
    /* Make sure stdin is a terminal. */
    if (!isatty (fd_serial_))
      {
        fprintf (stderr, "Not a terminal.\n");
        // exit (EXIT_FAILURE);
      }
    
    /* Save the terminal attributes so we can restore them later. */
    tcgetattr (fd_serial_, &saved_attributes_);
    
    /* ********set terminal modes********** */
    tcgetattr (fd_serial_, &new_attributes_);
    
    /* set the BSD style raw mode */
    // cfmakeraw (&new_attributes_);
    new_attributes_.c_cflag |= CREAD | CLOCAL;
    
    /* only need 1 stop bit */
    new_attributes_.c_cflag &= ~CSTOPB;
    
    /* no hardware flowcontrol */
    new_attributes_.c_cflag &= ~CRTSCTS;
    
    /* set the baud rate */
    cfsetspeed (&new_attributes_,B115200);
    
    
    /* set the attributes for tuning read() */
    // new_attributes_.c_cc[VMIN] = 9;
    // new_attributes_.c_cc[VTIME] = 0 ;
    new_attributes_.c_iflag = IGNPAR | ICRNL;
    new_attributes_.c_oflag = 0;
    new_attributes_.c_lflag = ICANON;
    tcflush(fd_serial_, TCIFLUSH);

    tcsetattr (fd_serial_, TCSAFLUSH, &new_attributes_);
    tcflush(fd_serial_, TCIOFLUSH);
  }

  /* opens the requested serial port(tty)  */
  void TranslationUnit::open_serial_port(const char * tty)
  {
    // CHANGED
    // fd_serial_ = open (tty, O_RDWR | O_NOCTTY | O_SYNC);
        fd_serial_ = open (tty, O_RDWR | O_NOCTTY | O_SYNC); 

    if (fd_serial_ < 0) {
      perror("port");
      std::printf ("error opening file ");
      //exit (EXIT_FAILURE);
    }



  }

  void TranslationUnit::config__motor_controller()
  {
    // puts("confug_motor_controller");
    clock_gettime(CLOCK_MONOTONIC_RAW, &time_prev_encoder_);
    clock_gettime(CLOCK_MONOTONIC_RAW, &time_prev_speed_);

    write_to_port(disable_echo_);
    std::this_thread::sleep_for(1ms);
    
    write_to_port(set_zero_speed_);
    std::this_thread::sleep_for(1ms);

    write_to_port(push_out_data_);
    std::this_thread::sleep_for(1ms);

    write_to_port(disable_emergency_state_);
    std::this_thread::sleep_for(2.5s);
    tcflush(fd_serial_, TCIFLUSH);
  }

  void TranslationUnit::write_to_port(const char* write_string)
  {
    if (write (fd_serial_, write_string, strlen(write_string)) <= 0){
      puts ("error writing to serial port");
      //exit (EXIT_FAILURE);
    }
    usleep(100);
    tcflush(fd_serial_, TCIFLUSH);



  }

  void TranslationUnit::read_from_port()
  {

    tcflush(fd_serial_, TCIFLUSH);


    characters_read_= read (fd_serial_, &buffer_12_, read_buffer_size_-1);


        // clock that prints the time between consecutive samples
    if (characters_read_ < 0){
      puts ("error reading from serial port");
  // exit (EXIT_FAILURE);
    }
    /* set the last array cell to null so it can be read by printf */
    buffer_12_[characters_read_] = 0;
    	// printf("%s\n", buffer_12_);
  }

  int TranslationUnit::get__file_descriptor()
  {
    return  fd_serial_;
  }

  struct termios  TranslationUnit::get__original_tty_attributes()
  {
    return saved_attributes_;
  }

}  // namespace RobotLocomotion
