#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <mppi/core/solver.h>
#include <mppi/policies/gaussian_policy.h>
#include "mppi_ros/controller_interface.h"

namespace husky_fr3_mppi_ros {

class HuskyFr3ControllerNode : public mppi_ros::ControllerRos {
public:
  explicit HuskyFr3ControllerNode(const std::shared_ptr<rclcpp::Node>& node);
  bool init_ros() override;
  bool set_controller(std::shared_ptr<mppi::Solver>& controller) override;
  bool update_reference() override;
  void publish_ros() override;

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void observationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

private:
  double dt_ {0.01};
  std::string config_path_;
  std::string urdf_path_;
  std::string goal_topic_ {"husky_fr3_controller/target_pose"};
  std::string observation_topic_ {"husky_fr3_controller/mppi_observation"};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obs_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr optimal_base_trajectory_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr optimal_trajectory_publisher_;

  std::mutex ref_mutex_;
  bool have_goal_ {false};
  mppi::reference_trajectory_t ref_;

  mppi::dynamics_ptr dyn_local_;
  mppi::cost_ptr cost_local_;
};

} // namespace husky_fr3_mppi_ros
