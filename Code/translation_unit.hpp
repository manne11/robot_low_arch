#ifndef TRANSLATION_UNIT_HPP_8391950897557_
#define TRANSLATION_UNIT_HPP_8391950897557_

#include <chrono>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <vector>

// ros-specific
#include "rclcpp/rclcpp.hpp"

// messages
#include <rtodometry_msgs/msg/translation_speed.hpp>
#include <rtodometry_msgs/msg/stamped_speeds.hpp>
#include <rtodometry_msgs/msg/stamped_encoders.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

// message memory strategy
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>



namespace RobotLocomotion
{


class TranslationUnit : public rclcpp::Node

{
public:
  TranslationUnit();

  
  // checks if the port is available
  void port_heartbeat();

  // connects to the port
  void port_connect();

  // disconnects the port
  void port_disconnect();

  // set the publisher to publish translation data from motor
  void set_publisher__translational_data_encoders();


  // set the publisher to publish the encoder data
  void set_publisher__translational_data_speeds();

  // get the file descriptor for the opened port
  int get__file_descriptor();

  // get the termios structure that existed before the program
  // enforced its custom termios structure
  struct termios get__original_tty_attributes();


  // reads the wheel speed from the port
  void timer_callback_readspeed_or_readspeed_writespeed();
  
  #include "translation_unit_priv.hpp"
    };


} // namespace RobotLocomotion

#endif	//  TRANSLATION_UNIT_HPP_8391950897557_
