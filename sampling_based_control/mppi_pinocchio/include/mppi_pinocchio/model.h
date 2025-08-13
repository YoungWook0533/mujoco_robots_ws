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

  /** Update kinematics only */
  void update_state(const Eigen::VectorXd& q);
  /** Update kinematics with velocities */
  void update_state(const Eigen::VectorXd& q, Eigen::VectorXd& qd);

  /** Update geometry placements after calling update_state */
  void update_geometry();

  /** Count current self-collision pairs (after update_geometry) */
  std::size_t count_self_collisions() const;

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
};
}  // namespace mppi_pinocchio
