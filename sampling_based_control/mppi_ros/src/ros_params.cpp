//
// Created by giuseppe on 25.04.21.
//

#include "mppi_ros/ros_params.h"

namespace mppi_ros {
bool getString(std::shared_ptr<rclcpp::Node> node, const std::string& param_name, std::string& obj) {
  if (!node->get_parameter(param_name, obj)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse param %s", param_name.c_str());
    return false;
  }
  if (obj.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse param %s. String is empty.", param_name.c_str());
    return false;
  }
  return true;
}

bool getBool(std::shared_ptr<rclcpp::Node> node, const std::string& param_name, bool& obj) {
  if (!node->get_parameter(param_name, obj)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse param %s", param_name.c_str());
    return false;
  }
  return true;
}
}  // namespace mppi_ros
