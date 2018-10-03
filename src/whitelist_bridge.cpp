/* Maybe takes a .yml file as an argument, e.g.

1to2:
  .
  .
  .
  .
2to1:
  .
  .
  .
  .

*/

#include "ros/node_handle.h"
#include "rclcpp/rclcpp.hpp"

#include "ros1_bridge/bridge.hpp"


int main(int argc, char * argv[]) {
  // ROS 1 node
  ros::init(argc, argv, "whitelist_ros2_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("whitelist_ros1_bridge");
  // TODO: add timestamp to name or, better yet, use optional config file name + timestamp

  // Get the data out of a config file passed as an argument to the node.
  // Somehow parse the config file into something we can use.

  std::string ros1_topic_name;
  std::string ros1_type_name;
  std::string ros2_topic_name;
  std::string ros2_type_name;
  size_t queue_size;

  /////////////////////////
  // make 1to2 loop here //
  /////////////////////////
  ros1_topic_name = "chatter";
  ros1_type_name = "std_msgs/String";

  ros2_topic_name = "chatter";
  ros2_type_name = "std_msgs/String";

  queue_size = 10;

  auto 1to2handles = ros1_bridge::create_bridge_from_1_to_2(
    ros1_node, ros2_node,
    ros1_type_name, ros1_topic_name, queue_size,
    ros2_type_name, ros2_topic_name, queue_size
  );

  /////////////////////////
  // make 2to1 loop here //
  /////////////////////////
  ros2_topic_name = "chatter2";
  ros2_type_name = "std_msgs/String";

  ros1_topic_name = "chatter2";
  ros1_type_name = "std_msgs/String";

  queue_size = 10;

  auto 2to1handles = ros1_bridge::create_bridge_from_2_to_1(
    ros2_node, ros1_node,
    ros2_type_name, ros2_topic_name, queue_size,
    ros1_type_name, ros1_topic_name, queue_size
  );

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
