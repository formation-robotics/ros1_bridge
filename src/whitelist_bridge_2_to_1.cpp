#include <string>

// include ROS1 TODO: why all this clang stuff?
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// Include ROS2
#include "rclcpp/rclcpp.hpp"

#include "ros1_bridge/bridge.hpp"


int main(int argc, char * argv[]) {
  // ROS 1 node
  ros::init(argc, argv, "whitelist_bridge_2_to_1");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("whitelist_bridge_2_to_1");
  // TODO: add timestamp to name or, better yet, use optional config file name + timestamp

  // Validate number of arguments
  if (argc % 4 != 1) {
       std::cout << "\nError: Incorrect number of node arguments. \n"
           << "Must be given as a sequence of sets of four: \n"
           << "... <ros2_type> <ros2_topic> <ros1_type> <ros1_topic> ...\n\n";
       return -1;
   }

   int num_bridges = (argc - 1) / 4;
   ros1_bridge::Bridge2to1Handles handles[num_bridges];
   std::string ros2_type_name;
   std::string ros2_topic_name;
   std::string ros1_type_name;
   std::string ros1_topic_name;
   size_t queue_size = 10;

   int bridge_i = 0;
   for (int i = 1; i < argc; i += 4) {
       ros2_type_name = argv[i];
       ros2_topic_name = argv[i+1];
       ros1_type_name = argv[i+2];
       ros1_topic_name = argv[i+3];

       handles[bridge_i] = ros1_bridge::create_bridge_from_2_to_1(
         ros2_node, ros1_node,
         ros2_type_name, ros2_topic_name, queue_size,
         ros1_type_name, ros1_topic_name, queue_size
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
