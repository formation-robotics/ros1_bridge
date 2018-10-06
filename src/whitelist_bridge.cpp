#include <iostream>
#include <string>
#include <sstream>

// Include ROS1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// Include ROS2 and bridge
#include "rclcpp/rclcpp.hpp"
#include "ros1_bridge/bridge.hpp"


int main(int argc, char * argv[]) {
  std::stringstream timestamp_stream;
  timestamp_stream << std::chrono::system_clock::now().time_since_epoch().count();
  std::string timestamp = timestamp_stream.str();
  std::string name = "whitelist_bridge";
  std::string node_name = name.append(timestamp);
  // ROS 1 node
  ros::init(argc, argv, node_name);
  ros::NodeHandle ros1_node;
  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared(node_name);

  // Parse arguments
    int i_1to2 = 0;
    int i_2to1 = 0;
    std::string arg_1to2 = "1to2";
    std::string arg_2to1 = "2to1";
    for (int i = 0; i < argc; i += 1) {
      if (argv[i] == arg_1to2) {
        if (i_1to2 != 0) {
          std::cout << "Error: Only one 1to2 node argument is allowed.\n";
          return -1;
        }
        i_1to2 = i;
      } else if (argv[i] == arg_2to1) {
        if (i_2to1 != 0) {
          std::cout << "Error: Only one 2to1 node argument is allowed.\n";
          return -1;
        }
        i_2to1 = i;
      }
    }

    bool exists_1to2_bridges = (i_1to2 != 0);
    bool exists_2to1_bridges = (i_2to1 != 0);

    // Validate arguments
    if ((i_2to1 < i_1to2) && (exists_2to1_bridges)) {
      std::cout << "Error: 1to2 node argument must come before 2to1 node argument.\n";
      return -1;
    }

    // Validate 1to2 bridges argument length
    int argc_1to2;
    int num_1to2_bridges = 0;
    if (exists_1to2_bridges) {
      if (exists_2to1_bridges) {
        argc_1to2 = (i_2to1 - i_1to2);
      } else {
        argc_1to2 = (argc - i_1to2);
      }
      if (argc_1to2 % 4 != 1) {
        std::cout << argc_1to2;
        std::cout << "\nError: Incorrect number of node arguments following 1to2.\n"
          << "Must be given as a sequence of sets of four:\n"
          << "... <ros1_type> <ros1_topic> <ros2_type> <ros2_topic> ...\n\n";
        return -1;
      }
      num_1to2_bridges = (argc_1to2 - 1) / 4;
    }

    // Validate 2to1 bridges argument length
    int argc_2to1;
    int num_2to1_bridges = 0;
    if (exists_2to1_bridges) {
      argc_2to1 = (argc - i_2to1);
      if (argc_2to1 % 4 != 1) {
        std::cout << "\nError: Incorrect number of node arguments following 2to1.\n"
          << "Must be given as a sequence of sets of four:\n"
          << "... <ros2_type> <ros2_topic> <ros1_type> <ros1_topic> ...\n\n";
        return -1;
      }
      num_2to1_bridges = (argc_2to1 - 1) / 4;
    }

    std::string ros1_type_name;
    std::string ros1_topic_name;
    std::string ros2_type_name;
    std::string ros2_topic_name;
    size_t queue_size = 10;

    // Create ROS1 --> ROS2 bridges
    ros1_bridge::Bridge1to2Handles handles_1to2[num_1to2_bridges];
    if (exists_1to2_bridges) {
     int bridge_i = 0;
     for (int i = i_1to2 + 1; i < (i_1to2 + argc_1to2); i += 4) {
         ros1_type_name = argv[i];
         ros1_topic_name = argv[i+1];
         ros2_type_name = argv[i+2];
         ros2_topic_name = argv[i+3];
         handles_1to2[bridge_i] = ros1_bridge::create_bridge_from_1_to_2(
           ros1_node, ros2_node,
           ros1_type_name, ros1_topic_name, queue_size,
           ros2_type_name, ros2_topic_name, queue_size
         );
         bridge_i += 1;
     }
   }

   // Create ROS2 --> ROS1 bridges
   ros1_bridge::Bridge2to1Handles handles_2to1[num_2to1_bridges];
   if (exists_2to1_bridges) {
     int bridge_i = 0;
     for (int i = i_2to1 + 1; i < (i_2to1 + argc_2to1); i += 4) {
         ros2_type_name = argv[i];
         ros2_topic_name = argv[i+1];
         ros1_type_name = argv[i+2];
         ros1_topic_name = argv[i+3];
         handles_2to1[bridge_i] = ros1_bridge::create_bridge_from_2_to_1(
           ros2_node, ros1_node,
           ros2_type_name, ros2_topic_name, queue_size,
           ros1_type_name, ros1_topic_name, queue_size
         );
         bridge_i += 1;
     }
   }

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
