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
  std::string name = "whitelist_bidirectional_bridge";
  std::string node_name = name.append(timestamp);

  // ROS 1 node
  ros::init(argc, argv, node_name);
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared(node_name);

  // Parse arguments
  if (argc % 2 != 1) {
      std::cout << "Error: bidirectional bridge has odd number of arguments -- each topic must be matched with a type.\n";
  }

  std::string type_name;
  std::string topic_name;
  size_t queue_size = 10;

  // Create bidirectional bridges
  ros1_bridge::BridgeHandles bidirectional_handles_array[argc / 2];
  int bridge_i = 0;
  for (int i = 1; i < argc; i += 2) {
      type_name = argv[i];
      topic_name = argv[i+1];
      bidirectional_handles_array[bridge_i] = ros1_bridge::create_bidirectional_bridge(
          ros1_node,
          ros2_node,
          type_name,
          type_name,
          topic_name,
          queue_size
      );
      bridge_i += 1;
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
