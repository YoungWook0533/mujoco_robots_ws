#pragma once
#include <mppi/core/cost.h>
#include <mppi_pinocchio/model.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace husky_fr3_mppi_ros {

struct CostParams {
  double linear_weight;
  double angular_weight;
  double regularization;
  double vel_regularization;
  double joint_limit_cost;

  double reach_weight;           // constant penalty when violating
  double max_reach;         // meters, configurable via YAML
  double min_dist;          // optional minimum distance (meters)

  std::string srdf_path;              // SRDF path for disabled pairs and groups

  // Distance-based self collision parameters
  double Q_collision{0.0};            // weight
  double collision_threshold{0.0};    // activate penalty if min distance < threshold

  // Joint limits
  Eigen::VectorXd lower_limits; // size 7
  Eigen::VectorXd upper_limits; // size 7
  // Pinocchio/robot config
  std::string ee_frame{"fr3_link8"};
  std::string arm_base_frame{"fr3_link0"};
  int arm_dof{7};
  int wheel_dof{2}; // number of base/wheel DoF appended as zeros to match model nq
};

class HuskyFr3MppiCost : public mppi::Cost {
public:
  // Single constructor requiring SRDF path
  HuskyFr3MppiCost(const std::string& urdf_path, const std::string& cost_config_path, const std::string& srdf_path);
  HuskyFr3MppiCost(const HuskyFr3MppiCost& o);
  mppi::cost_ptr create() override;
  mppi::cost_ptr clone() const override;
  mppi::cost_t compute_cost(const mppi::observation_t& x, const mppi::input_t& u, const mppi::reference_t& ref, const double) override;

  // Load weights and joint limits from a YAML file. Returns true on success.
  bool load_config(const std::string& yaml_path);

  // Override SRDF path at runtime (e.g., from launch) and rebuild collision models
  void set_srdf_path(const std::string& srdf_path);

private:
  // Helpers to keep compute_cost clean
  void ensure_u_prev_size(const mppi::input_t& u);
  void build_q_from_obs(const mppi::observation_t& x);
  void compute_ee_world_pose(const mppi::observation_t& x, Eigen::Vector3d& p_world, Eigen::Quaterniond& q_world);
  void parse_reference_pose(const mppi::reference_t& ref, Eigen::Vector3d& ref_t, Eigen::Quaterniond& ref_q) const;

  // Collision helpers (implemented using mppi_pinocchio::RobotModel)
  void init_collision_models();

  mppi_pinocchio::RobotModel robot_model_; // per-instance (FK and collisions)
  CostParams params_;
  Eigen::VectorXd u_prev_;
  // Reusable buffer for Pinocchio configuration vector
  Eigen::VectorXd q_pin_buf_;
  // Buffer for reach computation
  Eigen::Vector3d distance_vector_{Eigen::Vector3d::Zero()};
  std::string urdf_path_;
  std::string urdf_xml_;
};

} // namespace husky_fr3_mppi_ros
