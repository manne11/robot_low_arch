/* contains the private part of orientation_unit.hpp */
/* File: orientation_unit_priv.h*/
#ifndef RAD2DEG
  #define RAD2DEG(x) ((x)*57.29578)
#endif

#ifndef DEG2RAD
  #define DEG2RAD(x) ((x)*0.017453293)
#endif



private:
  const std::string pub_topic_;
  rtodometry_msgs::msg::InertialUnit::SharedPtr imu_msg_;
  rmw_qos_profile_t realtime_qos_profile_O {rmw_qos_profile_default};
  rclcpp::Publisher<rtodometry_msgs::msg::InertialUnit>::SharedPtr imu_publisher_;
  
  rclcpp::TimerBase::SharedPtr timer_teleop_;
  rclcpp::TimerBase::SharedPtr timer_readOrientation_;

  


  // serial port related
  int fd_serial_ = 0;
  struct termios new_attributes_;
  struct termios saved_attributes_;
  int characters_read_{};
  char buffer_100_[100];
  size_t read_buffer_size_ = 100;

  // parser
  std::vector<std::string> data_string_vec {};
  float y_deg{},  p_deg{},  r_deg{},
           ax{},     ay{},     az{},
       gx_deg{}, gy_deg{}, gz_deg{},
         temp{};
  double    r{},   p{}, y{};
  double   gx{},  gy{}, gz{};
  int floats_parsed_ {0};
  double dYaw{}, gain_yaw{1.0};
  tf2::Quaternion q;
  double last_y_,
         last_p_,
         last_r_;
  int messages_not_parsed_ {0};
  int messages_parsed_ {0};
  std::string data{};
  struct timespec time_elapsed_imu;
  struct timespec prev_elapsed_imu;
  float imu_msg_dt{};


  // clock for time between consecutive samples
  struct timespec test_tim1_, previous_test_tim1_;
  float delta_t_test_1{};




  // private functions


  // read from the port, involves parsing and reading the data 
  void read_from_port();

  // read the data from Teensy
  void read__IMU();

  // parse the recieved data from the teensy
  bool parse__recieved_data();

  // open serial port fro the IMU data
  void open_serial_port();

  // set the proper mode for the port that we read
  // IMU data from
  void set_input_mode();

  
  

