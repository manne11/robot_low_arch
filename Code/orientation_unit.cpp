#include <termios.h>
#include <chrono>
#include <thread>
#include <string.h>
#include <cstring>
#include <iostream>
#include <cmath>
#include <math.h>
#include <unistd.h>


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>



#include "rclcpp/rclcpp.hpp"

#include "./orientation_unit.hpp"



namespace RobotLocomotion
{

 OrientationUnit::OrientationUnit()
    : Node("OrientationUnit")
    , pub_topic_ ("/imu_data")
    , imu_msg_(std::make_shared<rtodometry_msgs::msg::InertialUnit>())   
 {

#ifdef REAL_TIME_613710284348
      realtime_qos_profile_O.reliability         = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      realtime_qos_profile_O.history             = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
      realtime_qos_profile_O.depth               = 1;
      

   imu_publisher_ = this->create_publisher<rtodometry_msgs::msg::InertialUnit>
     (pub_topic_,
      realtime_qos_profile_O);

#else
   imu_publisher_ = this->create_publisher<rtodometry_msgs::msg::InertialUnit>
     (pub_topic_
      );
#endif   


   // set the buffer elements in the array to zero
   std::memset(buffer_100_, 0, sizeof buffer_100_);



  }


  




void OrientationUnit::port_heartbeat()
{
  std::cout << "port exists" << std::endl;
}

void OrientationUnit::port_connect()
{
  open_serial_port();
  set_input_mode();
  std::cout << "connected to the port" << std::endl;
  
}

void OrientationUnit::port_disconnect()
{
  close(fd_serial_);
  tcsetattr (fd_serial_, TCSANOW, &saved_attributes_);
  std::cout << "disconnected to port" << std::endl;
  std::cout <<  "total messages: "
	    << messages_not_parsed_ +  messages_parsed_
	    << std::endl;

  std::cout <<  "messages parsed: "
	    << messages_parsed_
	    << std::endl;

  std::cout <<  "messages not parsed: "
	    << messages_not_parsed_
	    << std::endl;

}


  


  void OrientationUnit::timer_callback_readspeed()
  {
    read__IMU();
  }

  void OrientationUnit::read__IMU()
  {
    read_from_port();

    parse__recieved_data();


  }


  void OrientationUnit::read_from_port()
  {
    // clock that prints the time between consecutive samples
    tcflush(fd_serial_, TCIFLUSH);
    characters_read_= read (fd_serial_, &buffer_100_, read_buffer_size_-1);


    if (characters_read_ < 0){
      puts ("error reading from serial port");
  // exit (EXIT_FAILURE);
    }
    /* set the last array cell to null so it can be read by printf */
    buffer_100_[characters_read_] = 0;
    // printf("%s\n",buffer_100_);
  }

  bool OrientationUnit::parse__recieved_data ()
  {
    data = std::move(buffer_100_);
    
    if (data.compare(0, 4, "<IMU") == 0) // We have an IMU message
    {
      // Check for data message
	floats_parsed_ = sscanf(data.c_str(), "\n<IMU %f %f %f - %f %f %f - %f %f %f - %f>", &r_deg, &p_deg, &y_deg, &ax, &ay, &az, &gx_deg, &gy_deg, &gz_deg, &temp);

	

            if (floats_parsed_ == 10)
            {
	      messages_parsed_++;
	      r  = DEG2RAD(r_deg);
	      p  = DEG2RAD(p_deg);
	      y  = DEG2RAD(y_deg); 
	      gx = DEG2RAD(gx_deg);
	      gy = DEG2RAD(gy_deg);
	      gz = DEG2RAD(gz_deg);
	      static double last_yaw = y;
	      dYaw = angles::shortest_angular_distance(last_yaw, y);
	      last_yaw = y;
	      //ROS_INFO_STREAM("dYaw: " << dYaw << " Gain: " << dcs.gain_yaw);
	      static double last_corr_yaw = y;
	      last_corr_yaw = angles::normalize_angle(last_corr_yaw + dYaw *gain_yaw);
	      
	      // convert from ypr to quat

                q.setRPY(r, p, last_corr_yaw);
                
                imu_msg_->w_imu = gz * gain_yaw;
		clock_gettime(CLOCK_MONOTONIC_RAW, &time_elapsed_imu);
		imu_msg_dt =  (time_elapsed_imu.tv_sec - prev_elapsed_imu.tv_sec )* 1000.0
                              + (time_elapsed_imu.tv_nsec - prev_elapsed_imu.tv_nsec) / 1000000.0; 
		imu_msg_ -> dt_imu = imu_msg_dt;
 
		prev_elapsed_imu = time_elapsed_imu;

		imu_publisher_->publish(imu_msg_);                

                // update for next call

                last_y_ = y;
                last_p_ = p;
                last_r_ = r;






            
	    } else{
	      
	      messages_not_parsed_++;
	    }
	    

    }
    else 
    {
      
      messages_not_parsed_++;
      // publish unknown data

    }

    return true;
  } // readHandler


  void OrientationUnit::open_serial_port()
  {
      // fd_serial_ = open ("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
    fd_serial_ = open ("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
      if (fd_serial_ < 0) {
	perror("port");
	std::printf ("error opening file ");
	//exit (EXIT_FAILURE);
      }

  }

  void OrientationUnit::set_input_mode()
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
    // CHANGED
    // cfmakeraw (&new_attributes_);
    new_attributes_.c_cflag |= CREAD | CLOCAL;
    
    /* only need 1 stop bit */
    new_attributes_.c_cflag &= ~CSTOPB;
    
    /* no hardware flowcontrol */
    new_attributes_.c_cflag &= ~CRTSCTS;
    
    /* set the baud rate */
    cfsetspeed (&new_attributes_,B115200);
    
    
    /* set the attributes for tuning read() */
    // CHANGED
    // new_attributes_.c_cc[VMIN] = 70;
    // new_attributes_.c_cc[VTIME] = 0 ;
    new_attributes_.c_iflag = IGNPAR | ICRNL;
    new_attributes_.c_oflag = 0;
    new_attributes_.c_lflag = ICANON;
    tcflush(fd_serial_, TCIFLUSH);

    tcsetattr (fd_serial_, TCSAFLUSH, &new_attributes_);
    tcflush(fd_serial_, TCIOFLUSH);
  }
  
}  // namespace RobotLocomotion

