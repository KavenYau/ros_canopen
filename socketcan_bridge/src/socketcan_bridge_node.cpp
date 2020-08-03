// Copyright (c) 2016-2019, Ivor Wanders, Mathias Lüdtke, AutonomouStuff
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <socketcan_bridge/socketcan_to_topic.hpp>
#include <socketcan_bridge/topic_to_socketcan.hpp>
#include <socketcan_interface/threading.hpp>
#include <string>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // std::vector<can::ThreadedSocketCANInterfaceSharedPtr> driver_vector;
  std::map<std::string, can::ThreadedSocketCANInterfaceSharedPtr> driver_map;
  std::map<std::string, std::shared_ptr<socketcan_bridge::TopicToSocketCAN>> topic_to_socketCAN_map;
  std::map<std::string, std::shared_ptr<socketcan_bridge::SocketCANToTopic>> socketCAN_to_topic_map;

  auto node = rclcpp::Node::make_shared("socketcan_bridge_node");
  node->declare_parameter("can_devices", std::vector<std::string>());
  std::vector<std::string> can_devices = node->get_parameter("can_devices").as_string_array();
  for (std::string& can_device : can_devices) {
    node->declare_parameter(can_device + ".send_topic", "/" + can_device + "/send");
    node->declare_parameter(can_device + ".receive_topic", "/" + can_device + "/receive");
    std::string send_topic = node->get_parameter(can_device + ".send_topic").as_string();
    std::string receive_topic = node->get_parameter(can_device + ".receive_topic").as_string();
    RCLCPP_INFO(node->get_logger(), "can_device: '%s', send_topic: %s, receive_topic: %s", can_device.c_str(),
                send_topic.c_str(), receive_topic.c_str());

    can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface>();

    if (!driver->init(can_device, 0)) {  // initialize device at can_device, 0 for no loopback.
      RCLCPP_FATAL(node->get_logger(), "Failed to initialize can_device at %s", can_device.c_str());
      // return 1;
      continue;
    } else {
      // driver_vector.push_back(driver);
      driver_map.insert(std::make_pair(can_device, driver));
      RCLCPP_INFO(node->get_logger(), "Successfully connected to %s.", can_device.c_str());
    }

    // initialize the bridge both ways.
    auto to_socketcan_bridge = std::make_shared<socketcan_bridge::TopicToSocketCAN>(node, driver, send_topic);
    to_socketcan_bridge->setup();
    topic_to_socketCAN_map.insert(std::make_pair(can_device, to_socketcan_bridge));

    auto to_topic_bridge = std::make_shared<socketcan_bridge::SocketCANToTopic>(node, driver, receive_topic);
    to_topic_bridge->setup(node);
    socketCAN_to_topic_map.insert(std::make_pair(can_device, to_topic_bridge));
  }

  if (driver_map.size() > 0) {
    rclcpp::spin(node);
    for (auto& driver_it : driver_map) {
      driver_it.second->shutdown();
      driver_it.second.reset();
      RCLCPP_INFO(node->get_logger(), "can_device %s shutdown", driver_it.first.c_str());
    }

  } else {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize any can device");
  }
}
