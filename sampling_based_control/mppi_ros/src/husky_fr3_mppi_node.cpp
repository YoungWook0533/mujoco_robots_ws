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

namespace husky_fr3_mppi_ros {

// State x = [x, y, yaw, q(7), qdot(7)]  -> dim 17
// Input u = [v, w, qdot_des(7)]         -> dim 9
class HuskyFr3MppiDynamics : public mppi::Dynamics {
public:
  explicit HuskyFr3MppiDynamics(double dt) : nx_(17), nu_(9) { this->dt = dt; x_ = Eigen::VectorXd::Zero(nx_); }
  size_t get_input_dimension() override { return nu_; }
  size_t get_state_dimension() override { return nx_; }
  mppi::dynamics_ptr create() override { return std::make_shared<HuskyFr3MppiDynamics>(dt); }
  mppi::dynamics_ptr clone() const override { return std::make_shared<HuskyFr3MppiDynamics>(dt); }
  void reset(const mppi::observation_t& x, const double t) override { x_ = x; t_ = t; }
  mppi::observation_t step(const mppi::input_t& u, const double) override {
    const double v = u(0); const double w = u(1);
    double x = x_(0), y = x_(1), yaw = x_(2);
    Eigen::VectorXd q = x_.segment(3,7);
    Eigen::VectorXd qdot = x_.segment(10,7);
    Eigen::VectorXd qdot_des = u.segment(2,7);
    const double alpha = 1.0; // no lag
    qdot = (1.0 - alpha) * qdot + alpha * qdot_des;
    q += qdot * dt;
    x += v * std::cos(yaw) * dt; y += v * std::sin(yaw) * dt; yaw += w * dt;
    mppi::observation_t xn = x_;
    xn(0)=x; xn(1)=y; xn(2)=yaw; xn.segment(3,7)=q; xn.segment(10,7)=qdot;
    x_ = xn; t_ += dt; return x_;
  }
  mppi::input_t get_zero_input(const mppi::observation_t&) override { return mppi::input_t::Zero(nu_); }
  const mppi::observation_t get_state() const override { return x_; }
private:
  size_t nx_, nu_; mppi::observation_t x_;
};

// Cost: EE pose tracking, input regularization, joint limits (using mppi_pinocchio::RobotModel)
class HuskyFr3MppiCost : public mppi::Cost {
public:
  explicit HuskyFr3MppiCost(const std::string& urdf_path) : urdf_path_(urdf_path) {
    std::ifstream file(urdf_path_);
    std::stringstream buffer; buffer << file.rdbuf();
    urdf_xml_ = buffer.str();
    // Initialize one model for single-threaded paths; threaded compute uses thread_local copies
    robot_.init_from_xml(urdf_xml_);
    w_pos_=500.0; w_ori_=300.0; w_u_=1e-2; w_du_=1e-3; w_jlim_=500.0;
    upper_limits_ = (Eigen::VectorXd(7) << 2.8973, 1.7628, 2.8973, 3.0718, 2.8973, 3.7525, 2.8973).finished();
    lower_limits_ = -upper_limits_;
    u_prev_.setZero(9);
  }
  HuskyFr3MppiCost(const HuskyFr3MppiCost& o)
    : w_pos_(o.w_pos_), w_ori_(o.w_ori_), w_u_(o.w_u_), w_du_(o.w_du_), w_jlim_(o.w_jlim_),
      lower_limits_(o.lower_limits_), upper_limits_(o.upper_limits_), u_prev_(o.u_prev_), urdf_path_(o.urdf_path_), urdf_xml_(o.urdf_xml_) {
    robot_.init_from_xml(urdf_xml_);
  }
  mppi::cost_ptr create() override { return std::make_shared<HuskyFr3MppiCost>(*this); }
  mppi::cost_ptr clone() const override { return std::make_shared<HuskyFr3MppiCost>(*this); }
  mppi::cost_t compute_cost(const mppi::observation_t& x, const mppi::input_t& u, const mppi::reference_t& ref, const double) override {
    // Thread-local RobotModel on heap to ensure proper Eigen alignment
    thread_local std::unique_ptr<mppi_pinocchio::RobotModel> tl_robot;
    if (!tl_robot) { tl_robot = std::make_unique<mppi_pinocchio::RobotModel>(); tl_robot->init_from_xml(urdf_xml_); }

    // Thread-local u_prev to avoid races between threads
    thread_local Eigen::VectorXd tl_u_prev = Eigen::VectorXd::Zero(9);

    Eigen::Vector3d p_ref = ref.head<3>();
    Eigen::Quaterniond q_ref(ref(6), ref(3), ref(4), ref(5));

    // Use URDF joint configuration only: [fr3_joint1..7, wheelL, wheelR] -> dim 9
    Eigen::VectorXd q_pin(9); q_pin.setZero();
    for (int j=0;j<7;++j) q_pin(j) = x(3+j);
    q_pin(7) = 0.0; q_pin(8) = 0.0;
    tl_robot->update_state(q_pin);
    auto pose_pin = tl_robot->get_pose("fr3_link8");

    // Compose with mobile base SE(3) from state (planar x,y,yaw)
    const double yaw = x(2);
    Eigen::Quaterniond q_base(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d t_base(x(0), x(1), 0.0);
    Eigen::Vector3d p_world = q_base * pose_pin.translation + t_base;
    Eigen::Quaterniond q_world = (q_base * pose_pin.rotation).normalized();

    // Position error
    Eigen::Vector3d ep = p_ref - p_world;

    // Orientation geodesic error (angle squared)
    double dot = std::fabs(q_ref.dot(q_world));
    dot = std::min(1.0, std::max(0.0, dot));
    double angle = 2.0 * std::acos(dot); // in [0, pi]
    double J_ori = w_ori_ * angle * angle;

    double J_pos = w_pos_ * ep.squaredNorm();

    // Input regularization
    double J_u = w_u_ * u.squaredNorm();
    double J_du = w_du_ * (u - tl_u_prev).squaredNorm();

    // Joint limit soft barrier
    Eigen::VectorXd qmid = 0.5*(upper_limits_ + lower_limits_);
    Eigen::VectorXd qrange = (upper_limits_ - lower_limits_);
    Eigen::VectorXd s = (x.segment(3,7) - qmid).cwiseQuotient(qrange);
    double J_jl = w_jlim_ * ((s.array().abs() - 0.5).max(0.0).matrix().squaredNorm());

    tl_u_prev = u; 
    return J_pos + J_ori + J_u + J_du + J_jl;
  }
private:
  // For single-threaded paths only; compute_cost uses thread-local model
  mppi_pinocchio::RobotModel robot_;
  double w_pos_, w_ori_, w_u_, w_du_, w_jlim_;
  Eigen::VectorXd lower_limits_, upper_limits_;
  Eigen::VectorXd u_prev_;
  std::string urdf_path_;
  std::string urdf_xml_;
};

class HuskyFr3ControllerNode : public mppi_ros::ControllerRos {
public:
  explicit HuskyFr3ControllerNode(const std::shared_ptr<rclcpp::Node>& node)
  : mppi_ros::ControllerRos(node) {
    // Defaults for ControllerRos expected params (declare only if absent)
    if (!node->has_parameter("policy_update_rate")) node->declare_parameter<double>("policy_update_rate", 30.0);
    if (!node->has_parameter("reference_update_rate")) node->declare_parameter<double>("reference_update_rate", 30.0);
    if (!node->has_parameter("ros_publish_rate")) node->declare_parameter<double>("ros_publish_rate", 10.0);
    if (!node->has_parameter("publish_ros")) node->declare_parameter<bool>("publish_ros", true);

    if (!node->has_parameter("dt")) node->declare_parameter<double>("dt", 0.01);
    if (!node->has_parameter("mppi_config_path")) node->declare_parameter<std::string>("mppi_config_path", "");
    if (!node->has_parameter("urdf_path")) node->declare_parameter<std::string>("urdf_path", "");
    if (!node->has_parameter("goal_topic")) node->declare_parameter<std::string>("goal_topic", "husky_fr3_controller/target_pose");
    if (!node->has_parameter("observation_topic")) node->declare_parameter<std::string>("observation_topic", "husky_fr3_controller/mppi_observation");

    node->get_parameter("dt", dt_);
    node->get_parameter("mppi_config_path", config_path_);
    node->get_parameter("urdf_path", urdf_path_);
    node->get_parameter("goal_topic", goal_topic_);
    node->get_parameter("observation_topic", observation_topic_);

    auto make_abs = [](const std::string& s){ return (!s.empty() && s[0] != '/') ? ("/" + s) : s; };
    goal_topic_ = make_abs(goal_topic_);
    observation_topic_ = make_abs(observation_topic_);
  }
  bool init_ros() override {
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
  bool set_controller(std::shared_ptr<Solver>& controller) override {
    if (config_path_.empty() || urdf_path_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "mppi_config_path or urdf_path is empty");
      return false;
    }
    mppi::config_t cfg;
    if (!cfg.init_from_file(config_path_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load MPPI config: %s", config_path_.c_str());
      return false;
    }
    dyn_local_ = std::make_shared<HuskyFr3MppiDynamics>(dt_);
    cost_local_ = std::make_shared<HuskyFr3MppiCost>(urdf_path_);
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
  bool update_reference() override {
    std::lock_guard<std::mutex> lk(ref_mutex_);
    if (!have_goal_) return true;
    auto& ctrl = get_controller();
    if (ctrl) {
      ctrl->set_reference_trajectory(ref_);
    }
    if (cost_local_) cost_local_->set_reference_trajectory(ref_);
    return true;
  }
  void publish_ros() override {
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
private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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

  void observationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
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
private:
  // Params
  double dt_ {0.01};
  std::string config_path_;
  std::string urdf_path_;
  std::string goal_topic_ {"husky_fr3_controller/target_pose"};
  std::string observation_topic_ {"husky_fr3_controller/mppi_observation"};

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obs_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr optimal_base_trajectory_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr optimal_trajectory_publisher_;

  // Reference handling
  std::mutex ref_mutex_;
  bool have_goal_ {false};
  mppi::reference_trajectory_t ref_;

  // Local controller components we need access to
  mppi::dynamics_ptr dyn_local_;
  mppi::cost_ptr cost_local_;
};

} // namespace husky_fr3_mppi_ros

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
