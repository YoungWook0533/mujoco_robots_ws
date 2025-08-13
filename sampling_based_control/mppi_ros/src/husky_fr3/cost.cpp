#include "mppi_ros/husky_fr3/cost.hpp"
#include <fstream>
#include <sstream>

namespace husky_fr3_mppi_ros {

HuskyFr3MppiCost::HuskyFr3MppiCost(const std::string& urdf_path) : urdf_path_(urdf_path) {
  std::ifstream file(urdf_path_);
  std::stringstream buffer; buffer << file.rdbuf();
  urdf_xml_ = buffer.str();
  robot_.init_from_xml(urdf_xml_);
  w_pos_=500.0; w_ori_=300.0; w_u_=1e-2; w_du_=1e-3; w_jlim_=500.0;
  upper_limits_ = (Eigen::VectorXd(7) << 2.8973, 1.7628, 2.8973, 3.0718, 2.8973, 3.7525, 2.8973).finished();
  lower_limits_ = -upper_limits_;
  u_prev_.setZero(9);
}

HuskyFr3MppiCost::HuskyFr3MppiCost(const HuskyFr3MppiCost& o)
  : w_pos_(o.w_pos_), w_ori_(o.w_ori_), w_u_(o.w_u_), w_du_(o.w_du_), w_jlim_(o.w_jlim_),
    lower_limits_(o.lower_limits_), upper_limits_(o.upper_limits_), u_prev_(o.u_prev_), urdf_path_(o.urdf_path_), urdf_xml_(o.urdf_xml_) {
  robot_.init_from_xml(urdf_xml_);
}

mppi::cost_ptr HuskyFr3MppiCost::create() { return std::make_shared<HuskyFr3MppiCost>(*this); }
mppi::cost_ptr HuskyFr3MppiCost::clone() const { return std::make_shared<HuskyFr3MppiCost>(*this); }

mppi::cost_t HuskyFr3MppiCost::compute_cost(const mppi::observation_t& x, const mppi::input_t& u, const mppi::reference_t& ref, const double) {
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

} // namespace husky_fr3_mppi_ros
