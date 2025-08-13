#include "mppi_ros/husky_fr3/node.hpp"
#include "mppi_ros/husky_fr3/dynamics.hpp"
#include "mppi_ros/husky_fr3/cost.hpp"
#include <mppi_pinocchio/model.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>

namespace husky_fr3_mppi_ros {

HuskyFr3ControllerNode::HuskyFr3ControllerNode(const std::shared_ptr<rclcpp::Node>& node)
: mppi_ros::ControllerRos(node) {
  // Defaults for ControllerRos expected params (declare only if absent)
  if (!node->has_parameter("policy_update_rate")) node->declare_parameter<double>("policy_update_rate", 20.0);
  if (!node->has_parameter("reference_update_rate")) node->declare_parameter<double>("reference_update_rate", 10.0);
  if (!node->has_parameter("ros_publish_rate")) node->declare_parameter<double>("ros_publish_rate", 20.0);
  if (!node->has_parameter("publish_ros")) node->declare_parameter<bool>("publish_ros", true);

  if (!node->has_parameter("dt")) node->declare_parameter<double>("dt", 0.01);
  if (!node->has_parameter("mppi_config_path")) node->declare_parameter<std::string>("mppi_config_path", "");
  if (!node->has_parameter("cost_config_path")) node->declare_parameter<std::string>("cost_config_path", "");
  if (!node->has_parameter("urdf_path")) node->declare_parameter<std::string>("urdf_path", "");
  if (!node->has_parameter("srdf_path")) node->declare_parameter<std::string>("srdf_path", "");
  if (!node->has_parameter("goal_topic")) node->declare_parameter<std::string>("goal_topic", "husky_fr3_controller/target_pose");
  if (!node->has_parameter("observation_topic")) node->declare_parameter<std::string>("observation_topic", "husky_fr3_controller/mppi_observation");

  node->get_parameter("dt", dt_);
  node->get_parameter("mppi_config_path", config_path_);
  std::string cost_config_path; node->get_parameter("cost_config_path", cost_config_path);
  node->get_parameter("urdf_path", urdf_path_);
  node->get_parameter("srdf_path", srdf_path_);
  node->get_parameter("goal_topic", goal_topic_);
  node->get_parameter("observation_topic", observation_topic_);

  auto make_abs = [](const std::string& s){ return (!s.empty() && s[0] != '/') ? ("/" + s) : s; };
  goal_topic_ = make_abs(goal_topic_);
  observation_topic_ = make_abs(observation_topic_);

  // Store chosen cost config path in config_path_ if provided separately
  if (!cost_config_path.empty()) {
    // Use a separate cost YAML file
    config_path_ += ""; // keep solver path unchanged
  }
}

bool HuskyFr3ControllerNode::init_ros() {
  auto node = get_node();
  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, 10,
    std::bind(&HuskyFr3ControllerNode::goalCallback, this, std::placeholders::_1));
  obs_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    observation_topic_, 20,
    std::bind(&HuskyFr3ControllerNode::observationCallback, this, std::placeholders::_1));

  // Visualization publishers (PoseArrays)
  optimal_base_trajectory_publisher_ = node->create_publisher<geometry_msgs::msg::PoseArray>(
    "husky_fr3_controller/optimal_base_trajectory", 10);
  optimal_trajectory_publisher_ = node->create_publisher<geometry_msgs::msg::PoseArray>(
    "husky_fr3_controller/optimal_trajectory", 10);
  RCLCPP_INFO(node->get_logger(), "MPPI ROS interfaces ready. Subscribed to %s and %s", goal_topic_.c_str(), observation_topic_.c_str());
  return true;
}

bool HuskyFr3ControllerNode::set_controller(std::shared_ptr<mppi::Solver>& controller) {
  if (config_path_.empty() || urdf_path_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "mppi_config_path or urdf_path is empty");
    return false;
  }
  mppi::config_t cfg;
  if (!cfg.init_from_file(config_path_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load MPPI config: %s", config_path_.c_str());
    return false;
  }
  // Use stable velocity-driven dynamics (v,w,qdot_des)
  dyn_local_ = std::make_shared<HuskyFr3MppiDynamics>(dt_);

  // Require cost_config_path and construct cost with it
  std::string cost_config_path;
  get_node()->get_parameter("cost_config_path", cost_config_path);
  if (cost_config_path.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "cost_config_path is empty but required for HuskyFr3MppiCost");
    return false;
  }
  if (srdf_path_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "srdf_path is empty but required for HuskyFr3MppiCost");
    return false;
  }
  cost_local_ = std::make_shared<HuskyFr3MppiCost>(urdf_path_, cost_config_path, srdf_path_);

  auto pol = std::make_shared<mppi::GaussianPolicy>(dyn_local_->get_input_dimension(), cfg);
  auto solver = std::make_shared<mppi::Solver>(dyn_local_, cost_local_, pol, cfg);
  // Initialize reference (valid unit quaternion)
  ref_.rr.resize(1, Eigen::VectorXd::Zero(7)); ref_.tt.resize(1, 0.0);
  ref_.rr[0](6) = 1.0; // qw = 1
  solver->set_reference_trajectory(ref_); cost_local_->set_reference_trajectory(ref_);
  controller = solver; // hand over to base
  RCLCPP_INFO(get_node()->get_logger(), "MPPI controller set. nx=%zu, nu=%zu, dt=%.3f", dyn_local_->get_state_dimension(), dyn_local_->get_input_dimension(), dt_);
  return true;
}

bool HuskyFr3ControllerNode::update_reference() {
  std::lock_guard<std::mutex> lk(ref_mutex_);
  if (!have_goal_) return true;
  auto& ctrl = get_controller();
  if (ctrl) {
    ctrl->set_reference_trajectory(ref_);
  }
  if (cost_local_) cost_local_->set_reference_trajectory(ref_);
  return true;
}

void HuskyFr3ControllerNode::publish_ros() {
  // Publish PoseArrays for optimal rollout
  mppi::Rollout r;
  if (!get_optimal_rollout(r)) return;
  const int steps = static_cast<int>(r.tt.size());
  if (steps <= 0 || static_cast<size_t>(steps) != r.xx.size()) return;

  geometry_msgs::msg::PoseArray base_pa; base_pa.header.frame_id = "world"; base_pa.header.stamp = get_node()->now();
  geometry_msgs::msg::PoseArray ee_pa; ee_pa.header = base_pa.header;
  base_pa.poses.resize(steps); ee_pa.poses.resize(steps);

  // RobotModel for EE pose
  static bool inited = false; 
  static mppi_pinocchio::RobotModel shared_robot; 
  static std::string urdf_xml_cached;
  if (!inited) {
    std::ifstream file(urdf_path_); std::stringstream buf; buf << file.rdbuf(); urdf_xml_cached = buf.str();
    shared_robot.init_from_xml(urdf_xml_cached); inited = true;
  }
  // Deep-copy the shared model to avoid dangling pointers and double-free
  mppi_pinocchio::RobotModel robot(shared_robot);

  for (int i=0; i<steps; ++i) {
    const auto& x = r.xx[i];
    geometry_msgs::msg::Pose pb; pb.position.x = x(0); pb.position.y = x(1); pb.position.z = 0.0;
    double yaw = x(2);
    Eigen::Quaterniond qb(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    pb.orientation.x = qb.x(); pb.orientation.y = qb.y(); pb.orientation.z = qb.z(); pb.orientation.w = qb.w();
    base_pa.poses[i] = pb;

    // URDF joint configuration only (9-dim)
    Eigen::VectorXd q_pin(9); q_pin.setZero();
    for (int j=0;j<7;++j) q_pin(j) = x(3+j);
    q_pin(7) = 0.0; q_pin(8) = 0.0;
    robot.update_state(q_pin);
    auto pose_pin = robot.get_pose("fr3_link8");

    // Compose with base pose to world
    Eigen::Vector3d t_base(x(0), x(1), 0.0);
    Eigen::Quaterniond q_base(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d p_world = q_base * pose_pin.translation + t_base;
    Eigen::Quaterniond q_world = (q_base * pose_pin.rotation).normalized();

    geometry_msgs::msg::Pose pe; pe.position.x = p_world.x(); pe.position.y = p_world.y(); pe.position.z = p_world.z();
    pe.orientation.x = q_world.x(); pe.orientation.y = q_world.y(); pe.orientation.z = q_world.z(); pe.orientation.w = q_world.w();
    ee_pa.poses[i] = pe;
  }
  optimal_base_trajectory_publisher_->publish(base_pa);
  optimal_trajectory_publisher_->publish(ee_pa);
}

void HuskyFr3ControllerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Reference vector: [px, py, pz, qx, qy, qz, qw]
  Eigen::VectorXd ref(7);
  ref(0) = msg->pose.position.x;
  ref(1) = msg->pose.position.y;
  ref(2) = msg->pose.position.z;
  // Normalize quaternion
  Eigen::Quaterniond q_raw(msg->pose.orientation.w,
                           msg->pose.orientation.x,
                           msg->pose.orientation.y,
                           msg->pose.orientation.z);
  if (q_raw.norm() < 1e-9) q_raw = Eigen::Quaterniond::Identity();
  q_raw.normalize();
  ref(3) = q_raw.x();
  ref(4) = q_raw.y();
  ref(5) = q_raw.z();
  ref(6) = q_raw.w();
  {
    std::lock_guard<std::mutex> lk(ref_mutex_);
    if (ref_.rr.empty()) {
      ref_.rr.resize(1, Eigen::VectorXd::Zero(7));
      ref_.tt.resize(1, 0.0);
    }
    ref_.rr[0] = ref;
    ref_.tt[0] = 0.0;
    have_goal_ = true;
  }
}

void HuskyFr3ControllerNode::observationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  static bool first_obs_logged = false;
  // Expect: [x, y, yaw, q(7), qdot(7), t] -> 18 elements
  if (msg->data.size() < 18) {
    RCLCPP_WARN(get_node()->get_logger(), "Observation array too small: %zu", msg->data.size());
    return;
  }
  mppi::observation_t x(17);
  for (size_t i = 0; i < 17; ++i) x(i) = msg->data[i];
  double t = msg->data[17];

  if (!first_obs_logged) {
    RCLCPP_INFO(get_node()->get_logger(), "First observation received t=%.3f", t);
    first_obs_logged = true;
  }

  // If no goal yet, initialize reference to current EE pose to hold position
  if (!have_goal_) {
    static std::string urdf_xml_cached;
    if (urdf_xml_cached.empty()) {
      std::ifstream file(urdf_path_); std::stringstream buf; buf << file.rdbuf(); urdf_xml_cached = buf.str();
    }
    mppi_pinocchio::RobotModel robot_local;
    robot_local.init_from_xml(urdf_xml_cached);
    Eigen::VectorXd q_pin(9); q_pin.setZero();
    for (int j=0;j<7;++j) q_pin(j) = x(3+j);
    q_pin(7) = 0.0; q_pin(8) = 0.0;
    robot_local.update_state(q_pin);
    auto pose_pin = robot_local.get_pose("fr3_link8");

    // Compose with base pose to world for initial reference
    const double yaw = x(2);
    Eigen::Quaterniond q_base(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d t_base(x(0), x(1), 0.0);
    Eigen::Vector3d p_world = q_base * pose_pin.translation + t_base;
    Eigen::Quaterniond q_world = (q_base * pose_pin.rotation).normalized();

    Eigen::VectorXd ref(7);
    ref.head<3>() = p_world;
    ref(3) = q_world.x(); ref(4) = q_world.y(); ref(5) = q_world.z(); ref(6) = q_world.w();
    {
      std::lock_guard<std::mutex> lk(ref_mutex_);
      if (ref_.rr.empty()) { ref_.rr.resize(1, Eigen::VectorXd::Zero(7)); ref_.tt.resize(1, 0.0); }
      ref_.rr[0] = ref; ref_.tt[0] = 0.0;
      have_goal_ = true;
    }
    auto& ctrl_init = get_controller(); if (ctrl_init) ctrl_init->set_reference_trajectory(ref_);
    if (cost_local_) cost_local_->set_reference_trajectory(ref_);
    RCLCPP_INFO(get_node()->get_logger(), "Initialized MPPI reference to current EE pose.");
  }

  // Set the latest observation
  set_observation(x, t);

  auto& ctrl = get_controller();
  if (!ctrl) return;
  try {
    // Do not call update_policy() here; the worker thread handles policy updates.
    // Just request the latest input; ControllerRos will cache it for the publisher thread.
    mppi::input_t u(dyn_local_ ? dyn_local_->get_input_dimension() : static_cast<size_t>(9));
    this->get_input(x, u, t);
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to compute MPPI input: %s", e.what());
  }
}

} // namespace husky_fr3_mppi_ros
