//
// Created by giuseppe on 01.03.21.
//

#pragma once
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <limits>

// Geometry & SRDF support
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/geometry.hpp>

namespace mppi_pinocchio {

struct Pose {
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;

  Pose() = default;
  Pose(Eigen::Vector3d t, Eigen::Quaterniond r) : translation(t), rotation(r){};
};

Pose operator*(const Pose&, const Pose&);
Eigen::Matrix<double, 6, 1> diff(const Pose&, const Pose&);

class RobotModel {
 public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  struct MinDistResult {
    double distance{std::numeric_limits<double>::infinity()};
    Eigen::VectorXd grad;      // dq of distance (size = nq)
    Eigen::VectorXd grad_dot;  // d/dt grad (size = nq)
    int pair_index{-1};
    std::string link1;
    std::string link2;
    void setZero(int nq) { grad.setZero(nq); grad_dot.setZero(nq); }
  };

  RobotModel() = default;
  ~RobotModel();

  RobotModel(const RobotModel& rhs);
  /**
   * Build kinematic model from URDF XML string.
   */
  bool init_from_xml(const std::string& robot_description);

  /**
   * Initialize geometry model for collision checking using stored URDF XML.
   * Optionally apply SRDF to disable collision pairs/reference configs.
   */
  bool init_geometry_from_srdf(const std::string& srdf_path = std::string());
  bool init_geometry_from_srdf(const std::string& srdf_path, const std::string& packages_path);

  void set_urdf_path(const std::string& p) { urdf_path_file_ = p; }

  /** Update kinematics only */
  void update_state(const Eigen::VectorXd& q);
  /** Update kinematics with velocities */
  void update_state(const Eigen::VectorXd& q, Eigen::VectorXd& qd);

  /** Update geometry placements after calling update_state */
  void update_geometry();

  /** Count current self-collision pairs (after update_geometry) */
  std::size_t count_self_collisions() const;

  /** Minimum distance among all enabled collision pairs (requires update_geometry). Returns +inf if no pairs. */
  double min_distance_self() const;

  /** Full minimum distance computation with optional gradients (recomputes distances). */
  MinDistResult compute_min_distance(const Eigen::VectorXd& q, const Eigen::VectorXd* qdot = nullptr,
                                     bool with_grad = false, bool with_graddot = false, bool verbose = false) const;  // now uses caching

  /** Return nq (configuration vector size) */
  int nq() const;

  void get_error(const std::string& from_frame, const std::string& to_frame,
                 Vector6d& error) const;
  void get_error(const std::string& frame, const Eigen::Quaterniond& rot,
                 const Eigen::Vector3d& trans, Vector6d& error) const;

  void get_offset(const std::string& from_frame, const std::string& to_frame,
                  Eigen::Vector3d& offset);

  Pose get_pose(const std::string& frame) const;

  void print_info() const;

 private:
  // Kinematics
  pinocchio::Model* model_ {nullptr};
  pinocchio::Data* data_ {nullptr};

  // Geometry for collision
  pinocchio::GeometryModel* geom_model_ {nullptr};
  pinocchio::GeometryData* geom_data_ {nullptr};

  // Cached URDF XML to rebuild geometry
  std::string urdf_xml_;
  std::string urdf_path_file_;

  // Caching
  mutable Eigen::VectorXd q_cache_;
  mutable bool kinematics_valid_ {false};
  mutable bool geometry_valid_ {false};
  mutable bool joint_jacobians_valid_ {false};

  // New caching-aware API
  void update_kinematics(const Eigen::VectorXd& q);  // forward kinematics + frame placements (no geometry)
  void update_geometry_cached();                     // geometry placements only (requires kinematics valid)
  // Lightweight distance query assuming caches valid; called internally
  double min_distance_self_cached() const;
};
}  // namespace mppi_pinocchio
