#include "mppi_ros/husky_fr3/cost.hpp"
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace husky_fr3_mppi_ros {

HuskyFr3MppiCost::HuskyFr3MppiCost(const std::string& urdf_path) : urdf_path_(urdf_path) {
  std::ifstream file(urdf_path_);
  std::stringstream buffer; buffer << file.rdbuf();
  urdf_xml_ = buffer.str();
  robot_model_.init_from_xml(urdf_xml_);
  // Defaults similar to before
  params_.upper_limits = (Eigen::VectorXd(7) << 2.8973, 1.7628, 2.8973, 3.0718, 2.8973, 3.7525, 2.8973).finished();
  params_.lower_limits = -params_.upper_limits;
  params_.ee_frame = "fr3_link8";
  params_.arm_dof = 7;
  params_.wheel_dof = 2;
  u_prev_.setZero(params_.arm_dof + params_.wheel_dof);
  // Prepare buffer sized to model nq if possible
  q_pin_buf_.resize(params_.arm_dof + params_.wheel_dof);
  q_pin_buf_.setZero();
}

HuskyFr3MppiCost::HuskyFr3MppiCost(const std::string& urdf_path, const std::string& cost_config_path)
  : HuskyFr3MppiCost(urdf_path) {
  if (!cost_config_path.empty()) {
    load_config(cost_config_path);
    // resize u_prev_ and buffer according to possibly updated dofs
    u_prev_.setZero(params_.arm_dof + params_.wheel_dof);
    q_pin_buf_.resize(params_.arm_dof + params_.wheel_dof);
    q_pin_buf_.setZero();
  }
}

HuskyFr3MppiCost::HuskyFr3MppiCost(const HuskyFr3MppiCost& o)
  : params_(o.params_), u_prev_(o.u_prev_), q_pin_buf_(o.q_pin_buf_), urdf_path_(o.urdf_path_), urdf_xml_(o.urdf_xml_) {
  robot_model_.init_from_xml(urdf_xml_);
}

mppi::cost_ptr HuskyFr3MppiCost::create() { return std::make_shared<HuskyFr3MppiCost>(*this); }
mppi::cost_ptr HuskyFr3MppiCost::clone() const { return std::make_shared<HuskyFr3MppiCost>(*this); }

bool HuskyFr3MppiCost::load_config(const std::string& yaml_path) {
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node cost = root["cost"].IsDefined() ? root["cost"] : root;

    auto getd = [&](const char* k, double& dst){ if(cost[k]) dst = cost[k].as<double>(); };
    getd("linear_weight", params_.linear_weight);
    getd("angular_weight", params_.angular_weight);
    getd("regularization", params_.regularization);
    getd("vel_regularization", params_.vel_regularization);
    getd("joint_limit_cost", params_.joint_limit_cost);
    getd("reach_weight", params_.reach_weight);
    getd("max_reach", params_.max_reach);
    getd("min_dist", params_.min_dist);

    if (cost["upper_limits"]) {
      auto v = cost["upper_limits"].as<std::vector<double>>();
      if (!v.empty()) { params_.upper_limits = Eigen::Map<Eigen::VectorXd>(v.data(), (int)v.size()); }
    }
    if (cost["lower_limits"]) {
      auto v = cost["lower_limits"].as<std::vector<double>>();
      if (!v.empty()) { params_.lower_limits = Eigen::Map<Eigen::VectorXd>(v.data(), (int)v.size()); }
    } else {
      if (params_.lower_limits.size() != params_.upper_limits.size()) {
        params_.lower_limits = -params_.upper_limits;
      }
    }

    if (cost["ee_frame"]) params_.ee_frame = cost["ee_frame"].as<std::string>();
    if (cost["arm_base_frame"]) params_.arm_base_frame = cost["arm_base_frame"].as<std::string>();
    if (cost["arm_dof"]) params_.arm_dof = cost["arm_dof"].as<int>();
    if (cost["wheel_dof"]) params_.wheel_dof = cost["wheel_dof"].as<int>();

    // adjust buffers on config load
    u_prev_.setZero(params_.arm_dof + params_.wheel_dof);
    q_pin_buf_.resize(params_.arm_dof + params_.wheel_dof);
    q_pin_buf_.setZero();

    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to load cost config from " << yaml_path << ": " << e.what() << std::endl;
    return false;
  }
}

void HuskyFr3MppiCost::ensure_u_prev_size(const mppi::input_t& u) {
  if (u_prev_.size() != u.size()) u_prev_.setZero(u.size());
}

void HuskyFr3MppiCost::build_q_pin_from_observation(const mppi::observation_t& x) {
  const int tail = std::max(0, params_.wheel_dof);
  const int dof = std::max(0, params_.arm_dof);
  const int nq = dof + tail;
  if (q_pin_buf_.size() != nq) { q_pin_buf_.resize(nq); }
  q_pin_buf_.setZero();
  for (int j = 0; j < dof && (3 + j) < (int)x.size(); ++j) q_pin_buf_(j) = x(3 + j);
  robot_model_.update_state(q_pin_buf_);
}

void HuskyFr3MppiCost::compute_ee_world_pose(const mppi::observation_t& x, Eigen::Vector3d& p_world, Eigen::Quaterniond& q_world) {
  auto pose_pin = robot_model_.get_pose(params_.ee_frame);
  const double yaw = x(2);
  Eigen::Quaterniond q_base(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d t_base(x(0), x(1), 0.0);
  p_world = q_base * pose_pin.translation + t_base;
  q_world = (q_base * pose_pin.rotation).normalized();
}

void HuskyFr3MppiCost::parse_reference_pose(const mppi::reference_t& ref, Eigen::Vector3d& ref_t, Eigen::Quaterniond& ref_q) const {
  ref_t = ref.head<3>();
  ref_q = Eigen::Quaterniond(ref(6), ref(3), ref(4), ref(5));
}

mppi::cost_t HuskyFr3MppiCost::compute_cost(const mppi::observation_t& x, const mppi::input_t& u, const mppi::reference_t& ref, const double) {
  double cost = 0.0;

  ensure_u_prev_size(u);
  build_q_pin_from_observation(x);

  Eigen::Vector3d p_world; Eigen::Quaterniond q_world;
  compute_ee_world_pose(x, p_world, q_world);

  Eigen::Vector3d ref_t; Eigen::Quaterniond ref_q;
  parse_reference_pose(ref, ref_t, ref_q);
 
  // Position cost
  Eigen::Vector3d pos_err = ref_t - p_world;
  cost += params_.linear_weight * pos_err.squaredNorm();

  // Orientation cost
  double dq = std::fabs(ref_q.dot(q_world));
  dq = std::min(1.0, std::max(0.0, dq));
  double ang = 2.0 * std::acos(dq);
  cost += params_.angular_weight * ang * ang;

  // Arm reach cost (use frame offset in the robot model)
  if (params_.reach_weight > 0.0) {
    robot_model_.get_offset(params_.arm_base_frame, params_.ee_frame, distance_vector_);
    double reach = distance_vector_.head<2>().norm();
    if (reach > params_.max_reach) {
      cost += params_.reach_weight + params_.reach_weight * std::pow(reach - params_.max_reach, 2);
    }
    if (params_.min_dist > 0.0 && distance_vector_.norm() < params_.min_dist) {
      cost += params_.reach_weight + params_.reach_weight * std::pow(params_.min_dist - distance_vector_.norm(), 2);
    }
  }

  // Input regularization
  cost += params_.regularization * u.squaredNorm();
  cost += params_.vel_regularization * (u - u_prev_).squaredNorm();

  // Joint limits cost
  const int jdof = std::min<int>(params_.arm_dof, std::min<int>((int)params_.upper_limits.size(), (int)params_.lower_limits.size()));
  for (int i = 0; i < jdof && (3 + i) < (int)x.size(); ++i) {
    const double qi = x(3 + i);
    const double lo = params_.lower_limits(i);
    const double hi = params_.upper_limits(i);
    if (qi < lo) {
      const double d = lo - qi;
      cost += params_.joint_limit_cost * (1.0 + d * d);
    } else if (qi > hi) {
      const double d = qi - hi;
      cost += params_.joint_limit_cost * (1.0 + d * d);
    }
  }

  u_prev_ = u;
  return cost;
}

} // namespace husky_fr3_mppi_ros
