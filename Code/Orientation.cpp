// This process takes care of reading the orientation
// data from the Teensy for
// moving the robot

#include "./orientation_unit.hpp"
#include "./orientation_unit.cpp"

#include <rttest/rttest.h>
#include <rclcpp/rclcpp.hpp>
#include "rtodometrycontrol_sync/rtt_executor.hpp"


int main(int argc, char** argv)
{


#ifdef REAL_TIME_613710284348
    // set policy and priority
    struct sched_param scheduler_parameter;
    scheduler_parameter.sched_priority = 95;
      if(sched_setscheduler(0, SCHED_FIFO , &scheduler_parameter))
	{
	  perror("policy Fail:");
	  exit(EXIT_FAILURE);
	}
#endif
  int cpu_number = 2;
  cpu_set_t mask;


#ifdef REALTIME_CPU_AFFINITY_4667881194365
  // coer isolation
  CPU_ZERO(&mask);
  CPU_SET(cpu_number, &mask);
  if (sched_setaffinity(0, sizeof(mask), &mask)) {
    puts("cpu not shielded");
    exit(EXIT_FAILURE);
  }
#endif  
  // initialize rt-test and rclcpp
  rttest_read_args(argc, argv);
  rclcpp::init(argc, argv);

  const std::chrono::nanoseconds period_readspeed(20*1000*1000);

  auto Orientation_unit = std::make_shared<RobotLocomotion::OrientationUnit>();


  // connect to the USB port
  Orientation_unit->port_connect();



      
#ifdef REAL_TIME_613710284348
      
    // lock memory, prefault if necessary and lock dynamically
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
      {
	perror("mlockall failed: ");
	exit(EXIT_FAILURE);
      }
    
    // TODO: set the CPU affinity
#endif  


  struct timespec time_n, time_p;
  float dt__enc{};    
    
    while(rclcpp::ok())
      {
	Orientation_unit ->timer_callback_readspeed();
	
	rclcpp::spin_some(Orientation_unit);
	
      clock_gettime(CLOCK_MONOTONIC_RAW, &time_n);
        dt__enc          = ( time_n.tv_sec
			    -  time_p.tv_sec)*1000.0
          
	
                             + (time_n.tv_nsec -
				time_p.tv_nsec)/ 1000000.0;
	time_p = time_n;
	std::cout << dt__enc << std::endl;
    }
  

    rclcpp::shutdown();

    // disonnect USB port
    Orientation_unit->port_disconnect();

    return EXIT_SUCCESS;
}
