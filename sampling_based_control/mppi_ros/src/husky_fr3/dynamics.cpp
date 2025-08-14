#include "mppi_ros/husky_fr3/dynamics.hpp"
#include <cmath>

namespace husky_fr3_mppi_ros {

HuskyFr3MppiDynamics::HuskyFr3MppiDynamics(double dt) : nx_(17), nu_(9) {
  this->dt = dt;
  x_ = Eigen::VectorXd::Zero(nx_);
}

size_t HuskyFr3MppiDynamics::get_input_dimension() { return nu_; }
size_t HuskyFr3MppiDynamics::get_state_dimension() { return nx_; }

mppi::dynamics_ptr HuskyFr3MppiDynamics::create() { return std::make_shared<HuskyFr3MppiDynamics>(dt); }
mppi::dynamics_ptr HuskyFr3MppiDynamics::clone() const { return std::make_shared<HuskyFr3MppiDynamics>(dt); }

void HuskyFr3MppiDynamics::reset(const mppi::observation_t& x, const double t) { x_ = x; t_ = t; }

// Step the kinematics forward with input u
mppi::observation_t HuskyFr3MppiDynamics::step(const mppi::input_t& u, const double) {
  const double v = u(0); const double w = u(1);
  double x = x_(0), y = x_(1), yaw = x_(2);
  Eigen::VectorXd q = x_.segment(3,7);
  Eigen::VectorXd qdot = x_.segment(10,7);
  Eigen::VectorXd qdot_des = u.segment(2,7);
  qdot = qdot_des;
  q += qdot * dt;
  x += v * std::cos(yaw) * dt; y += v * std::sin(yaw) * dt; yaw += w * dt;
  mppi::observation_t xn = x_;
  xn(0)=x; xn(1)=y; xn(2)=yaw; xn.segment(3,7)=q; xn.segment(10,7)=qdot;
  x_ = xn; t_ += dt; return x_;
}

mppi::input_t HuskyFr3MppiDynamics::get_zero_input(const mppi::observation_t&) { return mppi::input_t::Zero(nu_); }

const mppi::observation_t HuskyFr3MppiDynamics::get_state() const { return x_; }

} // namespace husky_fr3_mppi_ros
