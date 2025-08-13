//
// Created by giuseppe on 01.03.21.
//

#pragma once
#include <rclcpp/rclcpp.hpp>

namespace mppi_ros {

template <typename T>
bool getNonNegative(std::shared_ptr<rclcpp::Node> node, const std::string& param_name, T& obj) {
  if (!node->get_parameter(param_name, obj)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse param %s", param_name.c_str());
    return false;
  }
  if (obj < 0) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse param %s. Invalid value: %f", param_name.c_str(), static_cast<double>(obj));
    return false;
  }
  return true;
}

template <typename T>
bool getNVector(std::shared_ptr<rclcpp::Node> node, const std::string& param_name, std::vector<T>& obj, size_t dim) {
  if (!node->get_parameter(param_name, obj)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse param %s", param_name.c_str());
    return false;
  }
  if (obj.size() != dim) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse param %s. Invalid vector size: %zu", param_name.c_str(), obj.size());
    return false;
  }
  return true;
}

bool getString(std::shared_ptr<rclcpp::Node> node, const std::string& param_name, std::string& obj);
bool getBool(std::shared_ptr<rclcpp::Node> node, const std::string& param_name, bool& obj);

}  // namespace mppi_ros