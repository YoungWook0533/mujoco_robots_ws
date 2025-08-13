#pragma once
#include <mppi/core/cost.h>
#include <mppi_pinocchio/model.h>
#include <Eigen/Dense>
#include <memory>

namespace husky_fr3_mppi_ros {

class HuskyFr3MppiCost : public mppi::Cost {
public:
  explicit HuskyFr3MppiCost(const std::string& urdf_path);
  HuskyFr3MppiCost(const HuskyFr3MppiCost& o);
  mppi::cost_ptr create() override;
  mppi::cost_ptr clone() const override;
  mppi::cost_t compute_cost(const mppi::observation_t& x, const mppi::input_t& u, const mppi::reference_t& ref, const double) override;
private:
  mppi_pinocchio::RobotModel robot_; // single-thread fallback
  double w_pos_, w_ori_, w_u_, w_du_, w_jlim_;
  Eigen::VectorXd lower_limits_, upper_limits_;
  Eigen::VectorXd u_prev_;
  std::string urdf_path_;
  std::string urdf_xml_;
};

} // namespace husky_fr3_mppi_ros
