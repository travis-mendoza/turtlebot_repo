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
: Sensors(nh)
{
  analog_publisher_ = nh->create_publisher<std_msgs::msg::UInt16MultiArray>(topic_name, rclcpp::QoS(rclcpp::KeepLast(10)));
}

void AnalogPins::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  try {
    // Add debug logging
    RCLCPP_INFO(rclcpp::get_logger("AnalogPins"), "Publishing analog pins data");
    
    auto analog_msg = std::make_unique<std_msgs::msg::UInt16MultiArray>();
    
    analog_msg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    analog_msg->layout.dim[0].label = "analog_pins";
    analog_msg->layout.dim[0].size = 6;
    analog_msg->layout.dim[0].stride = 6;
    analog_msg->layout.data_offset = 0;
    
    analog_msg->data.resize(6);
    
    // Use default values in case of error
    analog_msg->data[0] = 0;
    analog_msg->data[1] = 0;
    analog_msg->data[2] = 0;
    analog_msg->data[3] = 0;
    analog_msg->data[4] = 0;
    analog_msg->data[5] = 0;
    
    // Check if we can safely read the data
    if (dxl_sdk_wrapper) {
      RCLCPP_INFO(rclcpp::get_logger("AnalogPins"), "Reading A0");
      try {
        analog_msg->data[0] = dxl_sdk_wrapper->get_data_from_device<uint16_t>(
          extern_control_table.analog_a0.addr, extern_control_table.analog_a0.length);
          
        RCLCPP_INFO(rclcpp::get_logger("AnalogPins"), "Reading A1");
        analog_msg->data[1] = dxl_sdk_wrapper->get_data_from_device<uint16_t>(
          extern_control_table.analog_a1.addr, extern_control_table.analog_a1.length);
          
        // Continue for other pins...
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AnalogPins"), "Error reading analog data: %s", e.what());
      }
    }
    
    analog_publisher_->publish(std::move(analog_msg));
    RCLCPP_INFO(rclcpp::get_logger("AnalogPins"), "Published analog pins data");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("AnalogPins"), "Exception in publish: %s", e.what());
  }
}
