/* contains the private part of translation_unit.hpp */
/* File: translation_unit_private.h*/

private:
  size_t read_buffer_size_{};
  std::chrono::nanoseconds  publish_period_readspeed_;
  const std::string pub_topic_speed;
  const std::string pub_topic_encoder;
  bool straight_move_ {};
  bool curve_move_ {};
  bool write_speed_to_motor_ {false};

  rtodometry_msgs::msg::StampedSpeeds::SharedPtr speed_msg__read{};
  rtodometry_msgs::msg::StampedEncoders::SharedPtr encoder_msg__read{};
  rtodometry_msgs::msg::StampedSpeeds::SharedPtr speed_msg__write{};

  double scale_factor;




  // parser
  bool found_match_ {};
  int64_t encoder_left_ {}, encoder_right_{};
  int8_t parser__sscanf_return_value_encoder{};
  int8_t parser__sscanf_return_value_speed{};
  int64_t speed_left_ {}, speed_right_{};
  int64_t temp__messages_not_parsed {0};
  int64_t temp__messages_parsed {0};
  struct timespec time_now_speed_, time_prev_speed_;
  struct timespec time_now_encoder_,  time_prev_encoder_;
  float dt__speed_ {}, dt__encoder_{};


  rclcpp::Publisher<rtodometry_msgs::msg::StampedEncoders>::SharedPtr
  Translation_pub_encoder_{};
  rclcpp::Publisher<rtodometry_msgs::msg::StampedSpeeds>::SharedPtr
  Translation_pub_speed_{};
  rclcpp::TimerBase::SharedPtr timer_readwheelspeed_{};
  rclcpp::TimerBase::SharedPtr timer_teleop_{};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr       subscriber_teleop_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       subscriber_vel_;



  // Robot parameters dynamic:

  std::atomic<double> velocity {}, angular_z {};
  double r_speed {}, l_speed {};
  double V2 {}, B_Th {};
  int zero_cnt {};
  
  
  // Robot parameters static:

  double min_ang_vel, min_lin_vel;
  const int max_speed_robot;
  const double motor1_max_rpm, motor2_max_rpm;
  int gear_ratio;
  const double wheel_base_length, wheel_diff, wheel_diameter;
  const double calibration_left, calibration_right;


  double m1_factor, m2_factor;
  double rws,lws;
  double m1_cmd, m2_cmd;
  const char * motor_cmd;

  // realtime add-ons
  rmw_qos_profile_t realtime_qos_profile{};
  rmw_qos_profile_t teleop_setpoint_qos_profile{};
  
  // pool strategy for subscribers
  rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy
  <geometry_msgs::msg::Twist, 1>::SharedPtr memory_strategy__subscriber_vel_;

  

 // serial port related
  int fd_serial_ = 0;
  struct termios new_attributes_;
  struct termios saved_attributes_;
  int characters_read_{};
  char buffer_12_[12];
  int8_t test_port_printer{0};
  bool stop_motor{false};

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
            ( std::ostringstream() << std::dec << x ) ).str()

 // motor controller initialization commands
const char * disable_echo_              = "^ECHOF 1\r";
const char * set_zero_speed_            = "!M 0 0\r";
const char * push_out_data_             = "!r 2\r";
const char * disable_emergency_state_   = "!MG\r";
const char * enable_emergency_state_    = "!EX\r";
 struct timespec test_tim1_, previous_test_tim1_;
 float delta_t_test_1{};







   int counter_triggering_write_80ms{0};


  /*-----private function headers-------*/
  
  // reads the wheel speed from the port
  // void timer_callback_readspeed_or_readspeed_writespeed();
  void timer_callback_setbool_writespeed();

  // read_wheel_speed() has three responsibilities,
  // 1. get the data from the motor
  // 2. filter the data recieved
  // 3. store the data in speed_msg
  //
  // for now focus on getting it store the data in
  // the speed_msg type
  
  // TODO: these three responsibilities can be segregated
  
  void read__wheel_speed();

  // we create a teleop to publish to Translation the speeds
  // to be written to the motor controller, this callback is
  // is invoked within the translation when we recieve the speeds
  // of left and right wheel
  void teleop_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // subscription for "cmd_vel" from the controller / controller simulator.
  void vel_subscription_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // setting the input mode for the port
  void set_input_mode();

  // opening the serial port by passing a file(port) to open
  void open_serial_port(const char * tty);


 // setting the echo off from the serial port
  void set_echo_off();

  // writing to the serial port for constant char
  void write_to_port(const char* write_string);

  // read data from the port
  void read_from_port();

  // instrumented publish to include time stamping
  // and other needed functionalities before
  // publishing encoder data
  void publishEncoderData(const int64_t encoder_left_, const int64_t encoder_right_);


  // instrumented publish to include time stamping
  // and other needed functionalities before
  // publishing speed data
  void publishSpeedData(const int64_t speed_left_, const int64_t speed_right_);



  // parser for the incoming data that provides string from another node
  bool parse__recieved_data();

  // set the initialization phase for the motor controller
  void config__motor_controller();

  // set the velocity of robot with which it should
  // move on the track, angular_velocity can be
  // calculated from the radius
  void set_velocity_to_robot();


  void min_saturation_velocity();
  void max_saturation_velocity();
  void motor_cmd_saturation();


    // TODO: because this is to run in a seperate process,
    // should the condition of port_heartbeat ... and all
    // the other interfaces be in private?
  
    // TODO: put the initialized private variables in the constructor
