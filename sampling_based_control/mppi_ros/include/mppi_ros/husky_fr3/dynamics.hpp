#pragma once
#include <mppi/core/dynamics.h>
#include <Eigen/Dense>

namespace husky_fr3_mppi_ros {

// State x = [x, y, yaw, q(7), qdot(7)]  -> dim 17
// Input u = [v, w, qdot_des(7)]         -> dim 9
class HuskyFr3MppiDynamics : public mppi::Dynamics {
public:
  explicit HuskyFr3MppiDynamics(double dt);
  size_t get_input_dimension() override;
  size_t get_state_dimension() override;
  mppi::dynamics_ptr create() override;
  mppi::dynamics_ptr clone() const override;
  void reset(const mppi::observation_t& x, const double t) override;
  mppi::observation_t step(const mppi::input_t& u, const double) override;
  mppi::input_t get_zero_input(const mppi::observation_t&) override;
  const mppi::observation_t get_state() const override;
private:
  size_t nx_, nu_;
  mppi::observation_t x_;
};

}  // namespace husky_fr3_mppi_ros
