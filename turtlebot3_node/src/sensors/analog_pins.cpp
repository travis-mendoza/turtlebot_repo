// Copyright 2025 Travis Mendoza
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "turtlebot3_node/sensors/analog_pins.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::sensors::AnalogPins;

AnalogPins::AnalogPins(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name)
: Sensors(nh, "")  // Call parent constructor with node handle and empty frame_id
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  analog_publisher_ = nh->create_publisher<std_msgs::msg::UInt16MultiArray>(topic_name, qos);

  RCLCPP_INFO(nh->get_logger(), "Succeeded to create analog pins publisher");
}

void AnalogPins::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  (void)now;  // Mark as unused intentionally to suppress warning
  (void)dxl_sdk_wrapper;  // Also mark this as unused for now
  
  try {
    auto analog_msg = std::make_unique<std_msgs::msg::UInt16MultiArray>();
    
    // Set up dimensions for the message
    analog_msg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    analog_msg->layout.dim[0].label = "analog_pins";
    analog_msg->layout.dim[0].size = 6;
    analog_msg->layout.dim[0].stride = 6;
    analog_msg->layout.data_offset = 0;
    
    // Just publish zeroes for now - we'll add real data reading later
    analog_msg->data.resize(6, 0);
    
    analog_publisher_->publish(std::move(analog_msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("analog_pins"),
      "Exception in analog_pins publish: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("analog_pins"),
      "Unknown exception in analog_pins publish");
  }
}

