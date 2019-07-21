#include <rclcpp/rclcpp.hpp>
#include <rttest/rttest.h>



#include "rtodometrycontrol_sync/rtt_executor.hpp"

#include "math.h"


// message types
#include <rtodometry_msgs/msg/differential_time.hpp>
#include <rtodometry_msgs/msg/stamped_encoders.hpp>
#include <rtodometry_msgs/msg/stamped_speeds.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rtodometry_msgs/msg/inertial_unit.hpp>

//  memory strategy

#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

// TLSF allocator
#include <tlsf_cpp/tlsf.hpp>





using namespace std::chrono_literals;
using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;


class OdometryMovement : public rclcpp::Node {

 public:
OdometryMovement(std::chrono::nanoseconds period_odometry)
  :Node("odometry_node")
  ,publish_period_odom_(period_odometry)
  ,pub_topic_odom("/odom")
  ,pub_topic_dt("/dt")
  ,sub_topic_speed("/serial/speeds")
  ,sub_topic_imu("/imu_data")
  ,estimated_pose_msg_(std::make_shared<nav_msgs::msg::Odometry>())
  ,dt_msg_(std::make_shared<rtodometry_msgs::msg::DifferentialTime>())
  ,differential_timestamp(10.0 / 1000.0)
  ,x(0.0000)
  ,y(0.0000)
  ,th(0.0000)
  ,wheel_base_length(0.2318*2)
  ,wheel_diameter(12.434/100.0)
  ,memory_strategy__translation_subscription__(std::make_shared<MessagePoolMemoryStrategy
					      <rtodometry_msgs::msg::StampedSpeeds, 2>>())
  ,memory_strategy__orientation_subscription__(std::make_shared<MessagePoolMemoryStrategy
				                       <rtodometry_msgs::msg::InertialUnit, 2>>())

{

#ifdef REAL_TIME_613710284348
  // realtime qos profile
  realtime_qos_profile__.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  realtime_qos_profile__.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  realtime_qos_profile__.depth       = 1;

  
  // odom publisher
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>
    (
     pub_topic_odom,
     realtime_qos_profile__ );
  
  // dt publisher
  dt_pub_ = this->create_publisher<rtodometry_msgs::msg::DifferentialTime>
    (
     pub_topic_dt,
     realtime_qos_profile__ );

  // Subscribers
   translation_subscription = this->create_subscription<rtodometry_msgs::msg::StampedSpeeds>(
   sub_topic_speed,
   std::bind(&OdometryMovement::Translation_subscribe_callback,this,std::placeholders::_1),
   realtime_qos_profile__,
   nullptr,
   false,
   memory_strategy__translation_subscription__   
);



 orientation_subscription = this-> create_subscription<rtodometry_msgs::msg::InertialUnit>(
  sub_topic_imu,
  std::bind(&OdometryMovement::Orientation_subscribe_callback, this ,std::placeholders::_1),
  realtime_qos_profile__,
  nullptr,
  false,
  memory_strategy__orientation_subscription__);

#else
// odom publisher
odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
  pub_topic_odom);

// dt publisher
dt_pub_ = this->create_publisher<rtodometry_msgs::msg::DifferentialTime>(
  pub_topic_dt);


  // Subscribers
   translation_subscription = this->create_subscription<rtodometry_msgs::msg::StampedSpeeds>(
   sub_topic_speed,
   std::bind(&OdometryMovement::Translation_subscribe_callback,this,std::placeholders::_1)
  );



 orientation_subscription = this-> create_subscription<rtodometry_msgs::msg::InertialUnit>(
  sub_topic_imu,
  std::bind(&OdometryMovement::Orientation_subscribe_callback, this ,std::placeholders::_1)
  );

 
#endif
    
timer_odometry_ = this->create_wall_timer(
      				       publish_period_odom_,
      				       std::bind(&OdometryMovement::odometry_fusion, this)  );









}


void Translation_subscribe_callback(const rtodometry_msgs::msg::StampedSpeeds::UniquePtr speed_msg)
{
  // puts("callback called");
    clock_gettime(CLOCK_MONOTONIC_RAW, &time_elapsed_speed);
    speed_msg_dt =  (time_elapsed_speed.tv_sec - prev_elapsed_speed.tv_sec )* 1000.0
                  + (time_elapsed_speed.tv_nsec - prev_elapsed_speed.tv_nsec) / 1000000.0; 
    dt_msg_ -> speed = speed_msg_dt;

    prev_elapsed_speed = time_elapsed_speed;
    left_speed_rpm.store(speed_msg ->speeds.left_wheel,std::memory_order_relaxed);
    right_speed_rpm.store(speed_msg ->speeds.right_wheel,std::memory_order_relaxed);


}

void Orientation_subscribe_callback(const rtodometry_msgs::msg::InertialUnit::UniquePtr imu_msg){

    clock_gettime(CLOCK_MONOTONIC_RAW, &time_elapsed_imu);
    angular_velocity_imu.store(imu_msg ->w_imu,std::memory_order_relaxed);
    imu_msg_dt = (time_elapsed_imu.tv_sec - prev_elapsed_imu.tv_sec )* 1000.0 + (time_elapsed_imu.tv_nsec - prev_elapsed_imu.tv_nsec) / 1000000.0; 
    dt_msg_ -> imu = imu_msg_dt;
    
    prev_elapsed_imu = time_elapsed_imu;

}


  // to avoid concurrent access issues, we copy the values
  void caculate_forward_and_differential_velocities(double left_speed_rpm_copy, double right_speed_rpm_copy )
  {
  forward_velocity_rpm = (left_speed_rpm_copy + right_speed_rpm_copy) / 2.0;
    forward_velocity_ms = (forward_velocity_rpm * wheel_diameter * M_PI) / 60.0;

    differential_speed_ms = (right_speed_rpm_copy-left_speed_rpm_copy) * wheel_diameter * M_PI / 60;

  }


void odometry_fusion(){

  caculate_forward_and_differential_velocities(left_speed_rpm, right_speed_rpm);

  angular_velocity_encoders = differential_speed_ms/(wheel_base_length) ; //angular velocity in radian per second. * 2 to test
    
  if (left_speed_rpm > 1.1*right_speed_rpm 
      || left_speed_rpm < 0.9* right_speed_rpm)
    {
      pose_estimation(angular_velocity_imu, differential_timestamp);

    }
    else
      {
        // we rely on motor encoders.
        pose_estimation(angular_velocity_encoders, differential_timestamp );
    }
    
}
  


  void model_update(const double& z_ang, const double& dt)
{
  // TODO: in case dt is calculated dynamically and not constant.

  delta_x = (forward_velocity_ms * cos(th)) * dt;
  delta_y = (forward_velocity_ms * sin(th)) * dt;
  
  delta_th = z_ang * dt;
    
  x += delta_x;
  y += delta_y;
  th += delta_th;
  dt_msg_ -> x = x;
  dt_msg_ -> y = y;
  //dt_msg_ -> theta = th;
  dt_msg_ -> linear_v = forward_velocity_ms;
  dt_msg_ -> ang_w = z_ang;
}

void to_pose_msg()
{
  	//odom_quat = tf::createQuaternionMsgFromYaw(th);
    odom_quat.setRPY( 0.0, 0.0, th);

    quat_msg         = tf2::toMsg(odom_quat);
    dt_msg_ -> theta = atan2(2* quat_msg.z * quat_msg.w,
			     1 - (2 * quat_msg.z * quat_msg.z));

    estimated_pose_msg_ -> header.frame_id = pub_topic_odom.c_str(); // "odom";

    estimated_pose_msg_ -> pose.pose.position.x = x;
    estimated_pose_msg_ -> pose.pose.position.y = y;
    estimated_pose_msg_ -> pose.pose.position.z = 0.0;
    estimated_pose_msg_ -> pose.pose.orientation = quat_msg;
    



}

void to_twist_msg( const double& w_z)
{
  //set the velocity
    estimated_pose_msg_ -> child_frame_id = "base_link";
    estimated_pose_msg_ -> twist.twist.linear.x = forward_velocity_ms;
    estimated_pose_msg_ -> twist.twist.linear.y = 0;
    estimated_pose_msg_ -> twist.twist.angular.z = w_z;

}

void pose_estimation(double z_ang, double &dt) 
{
  
    //compute odometry way given the velocities of the robot
  model_update(z_ang, dt);
  // store the values in the odom/pose msg
  to_pose_msg();
  // store the values in the odom/twist msg
  to_twist_msg(z_ang);
  // publish the msg with time stamped
  publish_time_stamped_msgs();   

} 

void publish_time_stamped_msgs()
{
  //publish the message

    clock_gettime(CLOCK_MONOTONIC_RAW, &time_elapsed_odom);
   
    odometry_msg_dt = (time_elapsed_odom.tv_sec - prev_elapsed_odom.tv_sec )* 1000.0 + (time_elapsed_odom.tv_nsec - prev_elapsed_odom.tv_nsec) / 1000000.0; 
    dt_msg_ -> odometry = odometry_msg_dt;
 
    prev_elapsed_odom = time_elapsed_odom;
    dt_pub_ -> publish(dt_msg_);

    odom_pub_ -> publish(estimated_pose_msg_);
}





 private:
     std::chrono::nanoseconds  publish_period_odom_;
    
    // ROS2
    
    // Topics names
    const std::string pub_topic_odom{"/odom"};
    const std::string pub_topic_dt{"/dt"};
    const std::string sub_topic_speed{"/serial/speeds"};
    const std::string sub_topic_imu{"/imu_data"};
    
    // Messages
    nav_msgs::msg::Odometry::SharedPtr estimated_pose_msg_;
    rtodometry_msgs::msg::DifferentialTime::SharedPtr dt_msg_;
    
    // Publishers
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    rclcpp::Publisher<rtodometry_msgs::msg::DifferentialTime>::SharedPtr dt_pub_{};
    
    // Subscribers
    
    rclcpp::Subscription<rtodometry_msgs::msg::StampedSpeeds>::SharedPtr       translation_subscription;
    rclcpp::Subscription<rtodometry_msgs::msg::InertialUnit>::SharedPtr        orientation_subscription;
    
    // Timers
    
    rclcpp::TimerBase::SharedPtr timer_odometry_{};
    
    // Model:
    double differential_timestamp;
    double delta_x, delta_y, delta_th;
    double x, y, th;
    double angular_velocity_encoders, forward_velocity_rpm, forward_velocity_ms;
    std::atomic<double>angular_velocity_imu{} ;
    std::atomic<double> left_speed_rpm{}, right_speed_rpm{};
    double differential_speed_ms;
    float theta_from_quat;

  
    // robot parameters:
    double wheel_base_length, wheel_diameter;
    
    // timestamped signals:
    
    double imu_msg_dt, speed_msg_dt, odometry_msg_dt;
    
    // process odometry msg
    tf2::Quaternion odom_quat;
    geometry_msgs::msg::Quaternion quat_msg;
    
    // timers 
    struct timespec time_elapsed_speed;
    struct timespec prev_elapsed_speed;
    
    struct timespec time_elapsed_imu;
    struct timespec prev_elapsed_imu;
    
    struct timespec time_elapsed_odom;
    struct timespec prev_elapsed_odom;

    // realtime qos profile
    rmw_qos_profile_t realtime_qos_profile__{rmw_qos_profile_default};


    // // pool strategy for subscribers
    MessagePoolMemoryStrategy
    <rtodometry_msgs::msg::StampedSpeeds, 2>::SharedPtr  memory_strategy__translation_subscription__;

    MessagePoolMemoryStrategy
    <rtodometry_msgs::msg::InertialUnit, 2>::SharedPtr memory_strategy__orientation_subscription__;



};   // class OdometryMovement

























int main(int argc, char** argv){

#ifdef REAL_TIME_613710284348
  // set policy and priority
  struct sched_param scheduler_parameter;
  scheduler_parameter.sched_priority = 97;
    if(sched_setscheduler(0, SCHED_FIFO , &scheduler_parameter))
      {
  	perror("policy Fail:");
  	exit(EXIT_FAILURE);
      }
#endif

  int cpu_number = 2;
  cpu_set_t mask;

#ifdef REALTIME_CPU_AFFINITY_4667881194365
  CPU_ZERO(&mask);
  CPU_SET(cpu_number, &mask);
  if (sched_setaffinity(0, sizeof(mask), &mask)) {
    puts("cpu not shielded");
    exit(EXIT_FAILURE);
  }
#endif  

  

  rttest_read_args(argc, argv);
  rclcpp::init(argc, argv);

  
  auto Odometry_unit = std::make_shared<OdometryMovement>(std::chrono::nanoseconds(10000000));

    
  // set realtime qos profile
    rmw_qos_profile_t qos_profile__;
  qos_profile__             = rmw_qos_profile_default;
  qos_profile__.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos_profile__.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_profile__.depth       = 1;
  
  /* add the tf broadcaster in ROS2
    * it is not the same,as used in ROS1
    */



  rclcpp::executor::ExecutorArgs args;

  auto executor = std::make_shared<RobotLocomotion::RttExecutor>(args);


  executor->add_node(Odometry_unit);




    
#ifdef REAL_TIME_613710284348    
  // lock memory, prefault if necessary and lock dynamically
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
      perror("mlockall failed: ");
      exit(EXIT_FAILURE);
    }
  
  // set the CPU affinity
#endif

  

  executor->spin();

  rclcpp::shutdown();

 
}




