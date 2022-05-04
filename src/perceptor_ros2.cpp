/**
 * @brief Perceptor for ROS 2 package executable code.
 *
 * @author Fabrizio Romanelli <fabrizio.romanelli@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 *
 * @date Apr 23, 2022
 */

#include <iostream>
#include <cstdio>
#include <thread>

#include "realsense.hpp"
#include "perceptor_ros2.hpp"

#ifdef SMT
#pragma message "Generating multithreaded VIO process"
#else
#pragma message "Generating single-threaded VIO process"
#endif

#ifdef PX4
#pragma message "Activated publishers and subscribers for PX4 topics"
#endif

/* The works. */
int main(int argc, char **argv)
{
  // Disable buffering on stdout and stderr, for performance and correctness.
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  // Create ORB_SLAM2 instance.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, false, false);

  // Initialize RealSense cameras
  RealSense::sModality mode = RealSense::MULTI;
  RealSense *realsense = new RealSense(mode);

  // Initialize ROS 2 connection and MT executor.
  rclcpp::init(argc, argv);
#ifdef SMT
  rclcpp::executors::MultiThreadedExecutor perceptor_mt_executor;
#else
  rclcpp::executors::SingleThreadedExecutor perceptor_st_executor;
#endif
  std::cout << "ROS 2 executor initialized" << std::endl;

  // Create PerceptorNode.
  auto perceptor_node_ptr = std::make_shared<PerceptorNode>(&SLAM, realsense);

#ifdef SMT
  perceptor_mt_executor.add_node(perceptor_node_ptr);
  perceptor_mt_executor.spin();
#else
  perceptor_st_executor.add_node(perceptor_node_ptr);
  perceptor_st_executor.spin();
#endif

  // Done!
  rclcpp::shutdown();
  SLAM.Shutdown();
  delete(realsense);
  exit(EXIT_SUCCESS);
}
