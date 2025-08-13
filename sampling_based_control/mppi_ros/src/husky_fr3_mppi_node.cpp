#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <mutex>

#include <mppi/core/solver.h>
#include <mppi/policies/gaussian_policy.h>

#include "mppi_ros/controller_interface.h"

#include "mppi_pinocchio/model.h"
#include <Eigen/Dense>
#include <fstream>
#include <sstream>

using mppi::Solver;
using mppi::dynamics_ptr;
using mppi::cost_ptr;
using mppi::reference_trajectory_t;

#include "mppi_ros/husky_fr3/dynamics.hpp"
#include "mppi_ros/husky_fr3/cost.hpp"
#include "mppi_ros/husky_fr3/node.hpp"

namespace husky_fr3_mppi_ros {
// All implementations moved to split compilation units
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("husky_fr3_mppi", opts);
  auto controller = std::make_shared<husky_fr3_mppi_ros::HuskyFr3ControllerNode>(node);
  if (!controller->init()) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize HuskyFr3 MPPI ROS controller");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "HuskyFr3 MPPI node initialized. Waiting for observations and goals...");
  controller->start();
  rclcpp::spin(node);
  controller->stop();
  rclcpp::shutdown();
  return 0;
}
