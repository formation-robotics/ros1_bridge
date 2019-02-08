#include <fstream>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <sstream>
#include <map>

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


struct Bridge {
  std::string topic;
  std::string type;
  BridgeHandles bridge_handles;
}

int main(int argc, char * argv[]) {
  std::string formation_node_uid;
  std::ifstream formation_node_uid_file;
  formation_node_uid_file.open("/opt/formation/.local_state/formation_node_uid");
  std::getline(formation_node_uid_file, formation_node_uid);
  formation_node_uid_file.close();

  std::map<std::string, Bridge> bridges;

  std::string name = "formation_bridge_";
  std::string node_name = name.append(formation_node_uid);

  // ROS 1 node
  ros::init(argc, argv, node_name);
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared(node_name);

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Parsing topics and types
    std::ifstream bridge_types_topics_file;
    bridge_types_topics_file.open("/opt/formation/.local_state/bridge_types_topics");
    std::string line;
    std::vector<std::string> topics;
    std::vector<std::string> types;
    bool reading_topic = false;
    while (std::getline(bridge_types_topics_file, line)) {
      if (reading_topic) {
        topics.push_back(line);
      } else {
        types.push_back(line);
      }
      reading_topic = !reading_topic;
    }
    bridge_types_topics_file.close();

    size_t queue_size = 10;
    // Adding and replacing bridges
    for (uint i = 0; i < topics.size(); i++) {
      if (bridges.count(topics[i]) == 1) {
        if (bridges[topics[i]].type != types[i]) {
          std::cout << "replacing - " << topics[i] << " is now " << types[i] << std::endl;
          bridges.erase(topics[i]);
          bridges[topics[i]].bridge_handles = ros1_bridge::create_bidirectional_bridge(
            ros1_node,
            ros2_node,
            types[i],
            types[i],
            topics[i],
            queue_size
          );
        }
      } else {
        std::cout << "adding - " << types[i] << " " << topics[i] << std::endl;
        bridges[topics[i]].bridge_handles = ros1_bridge::create_bidirectional_bridge(
          ros1_node,
          ros2_node,
          types[i],
          types[i],
          topics[i],
          queue_size
        );
      }
    }

    // Erasing bridges
    for (auto it = bridges.cbegin(); it != bridges.cend();) {
      bool topic_name_exists = std::count(topics.begin(), topics.end(), it->second.topic);
      if (topic_name_exists) {
        ++it; // keep it - the adder or replacer handled it.
      } else {
        std::cout << "erasing - " << it->second.type << " " << it->second.topic << std::endl;
        bridges.erase(it++); // erase it - the adder and replacer skipped its processing.
      }
    }    

    // ROS 1 asynchronous spinner
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    // ROS 2 spinning loop
    rclcpp::executors::SingleThreadedExecutor executor;
    while (ros1_node.ok() && rclcpp::ok()) {
      executor.spin_node_once(ros2_node, std::chrono::milliseconds(500));
    }
  }
  return 0;
}
