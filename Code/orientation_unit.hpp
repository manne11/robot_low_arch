#ifndef ORIENTATION_UNIT_HPP_4747977511_
#define ORIENTATION_UNIT_HPP_4747977511_

#include <chrono>
#include <termios.h>
#include <geometry_msgs/msg/twist.hpp>

// ros-specific
#include "rclcpp/rclcpp.hpp"

// messages, be careful with the naming
#include <sensor_msgs/msg/imu.hpp>
#include <angles/angles.h>
#include "tf2/LinearMath/Quaternion.h"
#include <rtodometry_msgs/msg/inertial_unit.hpp>

namespace RobotLocomotion
{

class OrientationUnit : public rclcpp::Node
{  
public:
  OrientationUnit();
  
  // checks if the port is available
  void port_heartbeat();

  // connects to the port
  void port_connect();

  // disconnects the port
  void port_disconnect();

  // to synchronize we try to both read and write at the same time using bools
  void timer_callback_readspeed();





  #include "orientation_unit_priv.hpp"
  
};// class OrientationUnit
} // namespace robotLocomotion

#endif	//  ORIENTATION_UNIT_HPP_4747977511_
