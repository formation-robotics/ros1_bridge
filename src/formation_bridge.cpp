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
  ros1_bridge::BridgeHandles bridge_handles;
};


int main(int argc, char * argv[]) {
  std::string formation_node_uid_filename;
  std::string bridge_types_topics_filename;
  std::string termination_flag_filename;

  if (argc > 2) {
    formation_node_uid_filename = argv[1];
    bridge_types_topics_filename = argv[2];
    termination_flag_filename = argv[3];
  } else {
    std::cout << "Error: must provide three arguments to formation_bridge node\n";
    std::cout << "1) filename which contains formation node uid.\n";
    std::cout << "2) filename where bridge types and topics will be written.\n";
    std::cout << "3) filename where if file text is 'true', this process terminates\n";
    return -1;
  }

  std::string formation_node_uid;
  std::ifstream formation_node_uid_file(formation_node_uid_filename.c_str());
  if (!formation_node_uid_file.good()) {
    std::cout << "Problem reading specified formation_node_uid file.\n";
    return -1;
  }
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

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  while (ros1_node.ok() && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Parsing topics and types
    std::ifstream bridge_types_topics_file;
    bridge_types_topics_file.open(bridge_types_topics_filename.c_str());
    if (!bridge_types_topics_file.good()) {
      std::cout << "Problem reading specified bridge_types_topics file.\n";
      return -1;
    }
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

    // Adding and replacing bridges
    size_t queue_size = 10;
    for (uint i = 0; i < topics.size(); i++) {
      if (bridges.count(topics[i]) == 1) {
        if (bridges[topics[i]].type != types[i]) {
          std::cout << "replacing - " << topics[i] << " is now " << types[i] << std::endl;
          bridges.erase(topics[i]);
          bridges[topics[i]] = Bridge();
          bridges[topics[i]].topic = topics[i];
          bridges[topics[i]].type = types[i];
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
        bridges[topics[i]] = Bridge();
        bridges[topics[i]].topic = topics[i];
        bridges[topics[i]].type = types[i];
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

    // Check if this process should terminate
    std::ifstream termination_flag_file(termination_flag_filename.c_str());
    if (!termination_flag_file.good()) {
        std::cout << "Problem reading specified termination flag file.\n";
        return -1;
    }
    std::stringstream buffer;
    buffer << termination_flag_file.rdbuf();
    if (std::strcmp("true", buffer.str().c_str()) == 0) {
        return 0; // termination flag is set to "true", so we terminate with non-error value
    }

  }
  return 0;
}
