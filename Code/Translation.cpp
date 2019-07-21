// This process takes care of reading the translation
// data from the motor controller and writes to the motor controller for moving the robot


#include <memory>
#include <malloc.h>
#include <signal.h>


#include <rclcpp/rclcpp.hpp>
#include <rttest/rttest.h>


#include "./translation_unit.hpp"
#include "./translation_unit.cpp"
#include "rtodometrycontrol_sync/rtt_executor.hpp"


#include <rtodometry_msgs/msg/translation_speed.hpp>
#include <rtodometry_msgs/msg/rttest_results.hpp>




// <<<<<<<functions>>>>>>>
void signal_callback_handler(int signum);
void reset_input_mode ();

// <<<<<<<globals>>>>>>>
  int TR__file_descriptor;
  // for restoring the tty settings when the program
  // shuts down
  struct termios TR__saved_attributes;



int main(int argc, char** argv)
{

  // core isolation  
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




  rclcpp::init(argc, argv);
  auto Translation_unit = std::make_shared<RobotLocomotion::TranslationUnit>();


  Translation_unit->port_connect();


  // set the publisher to publish wheel speed to
  // the topic "/serial/encoders"
  Translation_unit-> set_publisher__translational_data_speeds();

  // set the publisher to publish encoder data to
  // the topic "/serial/speeds"
  Translation_unit-> set_publisher__translational_data_encoders();




#ifdef REAL_TIME_613710284348
  // set policy and priority
  struct sched_param scheduler_parameter;
  scheduler_parameter.sched_priority = 96;
  if(sched_setscheduler(0, SCHED_FIFO , &scheduler_parameter))
    {
      perror("policy Fail:");
      Translation_unit->port_disconnect();
      exit(EXIT_FAILURE);
      
    }
#endif

  
#ifdef REAL_TIME_613710284348  
  // lock memory, prefault if necessary and lock dynamically
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
      perror("mlockall failed: ");
      Translation_unit->port_disconnect();
      exit(EXIT_FAILURE);
    }

  // set the CPU affinity
#endif

  struct timespec time_n, time_p;
  float dt__enc{};
  
  while(rclcpp::ok())
    {
      Translation_unit ->timer_callback_readspeed_or_readspeed_writespeed();
      rclcpp::spin_some(Translation_unit);

      clock_gettime(CLOCK_MONOTONIC_RAW, &time_n);
        dt__enc          = ( time_n.tv_sec
			    -  time_p.tv_sec)*1000.0
          
	
                             + (time_n.tv_nsec -
				time_p.tv_nsec)/ 1000000.0;
	time_p = time_n;
	std::cout << dt__enc << std::endl;

    }




  // shutdown rclcpp here
  rclcpp::shutdown();

  // disconnect to the RS232 port
  Translation_unit->port_disconnect();
  
  return EXIT_SUCCESS;
}









 void signal_callback_handler(int signum)
{
  //reset_input_mode(), close() and _exit() are async safe
  reset_input_mode();
  close(TR__file_descriptor);
  _exit(signum);
}

void reset_input_mode (void)
{
  // tcsetattr and close are async-safe
  tcsetattr (TR__file_descriptor, TCSANOW, &TR__saved_attributes);
  close(TR__file_descriptor);
}


