//
// Created by giuseppe on 01.03.21.
//

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/collision/distance.hpp>

#include "mppi_pinocchio/model.h"

using namespace pinocchio;

namespace mppi_pinocchio {

Eigen::Matrix<double, 6, 1> diff(const Pose& p1, const Pose& p2) {
  return log6(SE3(p1.rotation, p1.translation)
                  .actInv(SE3(p2.rotation, p2.translation)))
      .toVector();
}

Pose operator*(const Pose& p1, const Pose& p2) {
  Pose res;
  SE3 temp(
      SE3(p1.rotation, p1.translation).act(SE3(p2.rotation, p2.translation)));
  res.translation = temp.translation();
  res.rotation = temp.rotation();
  return res;
}

RobotModel::~RobotModel() {
  delete model_;
  delete data_;
  delete geom_model_;
  delete geom_data_;
};

// deep copy
RobotModel::RobotModel(const RobotModel& rhs) {
  model_ = new pinocchio::Model(*rhs.model_);
  data_ = new pinocchio::Data(*rhs.data_);
  if (rhs.geom_model_) geom_model_ = new pinocchio::GeometryModel(*rhs.geom_model_);
  if (rhs.geom_data_) geom_data_ = new pinocchio::GeometryData(*geom_model_);
  urdf_xml_ = rhs.urdf_xml_;
}

void RobotModel::print_info() const {
  std::stringstream ss;
  ss << "Robot model has " << model_->nq << " dofs." << std::endl;
  std::cout << ss.str();
}

int RobotModel::nq() const { return model_ ? static_cast<int>(model_->nq) : 0; }

bool RobotModel::init_from_xml(const std::string& robot_description) {
  try {
    model_ = new Model();
    pinocchio::urdf::buildModelFromXML(robot_description, *model_);
    data_ = new Data(*model_);
    urdf_xml_ = robot_description;
  } catch (std::runtime_error& exc) {
    std::cout << exc.what();
    return false;
  } catch (...) {
    std::cout << "Unknown exception caught while building model." << std::endl;
    return false;
  }
  return true;
}

bool RobotModel::init_geometry_from_srdf(const std::string& srdf_path) {
  if (!model_) return false;
  try {
    // Build geometry model with collision meshes from URDF XML
    if (geom_model_) delete geom_model_;
    if (geom_data_) delete geom_data_;
    geom_model_ = new pinocchio::GeometryModel();
    pinocchio::urdf::buildGeom(*model_, urdf_xml_, pinocchio::COLLISION, *geom_model_);

    // Add all collision pairs (needed before SRDF filtering or distance queries)
    geom_model_->addAllCollisionPairs();
    std::size_t total_pairs = geom_model_->collisionPairs.size();

    // Apply SRDF disabled collision pairs if provided
    if (!srdf_path.empty()) {
      pinocchio::srdf::removeCollisionPairs(*model_, *geom_model_, srdf_path);
    }
    std::size_t filtered_pairs = geom_model_->collisionPairs.size();

    // Allocate geometry data
    geom_data_ = new pinocchio::GeometryData(*geom_model_);
    std::cout << "[RobotModel] Collision geometry initialized: " << filtered_pairs
              << " pairs (from " << total_pairs << ", removed " << (total_pairs - filtered_pairs)
              << ")" << std::endl;
    if (filtered_pairs == 0) {
      std::cout << "[RobotModel][WARN] No collision pairs available after SRDF filtering." << std::endl;
    }
    return true;
  } catch (std::runtime_error& exc) {
    std::cout << exc.what();
    return false;
  } catch (...) {
    std::cout << "Unknown exception caught while building geometry model." << std::endl;
    return false;
  }
}

bool RobotModel::init_geometry_from_srdf(const std::string& srdf_path, const std::string& packages_path) {
  if (!model_) return false;
  try {
    if (geom_model_) delete geom_model_;
    if (geom_data_) delete geom_data_;
    geom_model_ = new pinocchio::GeometryModel();
    // Use file path variant if provided
    std::string source = urdf_path_file_.empty() ? urdf_xml_ : urdf_path_file_;
    pinocchio::urdf::buildGeom(*model_, source, pinocchio::COLLISION, *geom_model_, packages_path);
    geom_model_->addAllCollisionPairs();
    std::size_t total_pairs = geom_model_->collisionPairs.size();
    if (!srdf_path.empty()) {
      pinocchio::srdf::removeCollisionPairs(*model_, *geom_model_, srdf_path);
    }
    std::size_t filtered_pairs = geom_model_->collisionPairs.size();
    geom_data_ = new pinocchio::GeometryData(*geom_model_);
    std::cout << "[RobotModel] (pkg path) Collision geometry initialized: " << filtered_pairs
              << " pairs (from " << total_pairs << ", removed " << (total_pairs - filtered_pairs) << ")" << std::endl;
    if (filtered_pairs == 0) {
      std::cout << "[RobotModel][WARN] No collision pairs after SRDF filtering (pkg path overload)." << std::endl;
    }
    return true;
  } catch (std::runtime_error& exc) {
    std::cout << exc.what();
    return false;
  } catch (...) {
    std::cout << "Unknown exception in pkg path geometry init." << std::endl;
    return false;
  }
}

void RobotModel::update_state(const Eigen::VectorXd& q) {
  forwardKinematics(*model_, *data_, q);
  updateFramePlacements(*model_, *data_);
  if (geom_model_) update_geometry();
}

void RobotModel::update_state(const Eigen::VectorXd& q, Eigen::VectorXd& qd) {
  forwardKinematics(*model_, *data_, q, qd);
  updateFramePlacements(*model_, *data_);
  if (geom_model_) update_geometry();
}

void RobotModel::update_geometry() {
  if (!geom_model_ || !geom_data_) return;
  pinocchio::updateGeometryPlacements(*model_, *data_, *geom_model_, *geom_data_);
}

void RobotModel::update_kinematics(const Eigen::VectorXd& q) {
  if (!model_) return;
  if (!kinematics_valid_ || q_cache_.size()!=q.size() || (q_cache_ - q).norm() > 0) {
    forwardKinematics(*model_, *data_, q);
    updateFramePlacements(*model_, *data_);
    q_cache_ = q;
    kinematics_valid_ = true;
    geometry_valid_ = false;
    joint_jacobians_valid_ = false;
  }
}

void RobotModel::update_geometry_cached() {
  if (!geom_model_) return;
  if (!geometry_valid_) {
    pinocchio::updateGeometryPlacements(*model_, *data_, *geom_model_, *geom_data_);
    geometry_valid_ = true;
  }
}

std::size_t RobotModel::count_self_collisions() const {
  if (!geom_model_ || !geom_data_) return 0;
  std::size_t count = 0;
  for (const auto& cp : geom_model_->collisionPairs) {
    const auto& go1 = geom_model_->geometryObjects[cp.first];
    const auto& go2 = geom_model_->geometryObjects[cp.second];
    const auto& M1 = geom_data_->oMg[cp.first];
    const auto& M2 = geom_data_->oMg[cp.second];

    hpp::fcl::Transform3s tf1(M1.rotation(), M1.translation());
    hpp::fcl::Transform3s tf2(M2.rotation(), M2.translation());

    hpp::fcl::CollisionRequest req;
    hpp::fcl::CollisionResult res;
    if (hpp::fcl::collide(go1.geometry.get(), tf1, go2.geometry.get(), tf2, req, res) > 0) {
      ++count;
    }
  }
  return count;
}

double RobotModel::min_distance_self() const {
  if (!geom_model_ || !geom_data_) return std::numeric_limits<double>::infinity();
  double min_d = std::numeric_limits<double>::infinity();
  for (const auto& cp : geom_model_->collisionPairs) {
    const auto& go1 = geom_model_->geometryObjects[cp.first];
    const auto& go2 = geom_model_->geometryObjects[cp.second];
    const auto& M1 = geom_data_->oMg[cp.first];
    const auto& M2 = geom_data_->oMg[cp.second];
    hpp::fcl::Transform3s tf1(M1.rotation(), M1.translation());
    hpp::fcl::Transform3s tf2(M2.rotation(), M2.translation());
    hpp::fcl::DistanceRequest dr; dr.enable_nearest_points = false; dr.enable_signed_distance = false; dr.rel_err = 0.0; dr.abs_err = 0.0;
    hpp::fcl::DistanceResult dres;
    hpp::fcl::distance(go1.geometry.get(), tf1, go2.geometry.get(), tf2, dr, dres);
    if (dres.min_distance < min_d) min_d = dres.min_distance;
  }
  return min_d;
}

void RobotModel::get_error(const std::string& from_frame,
                           const std::string& to_frame, Vector6d& error) const {
  error = log6(data_->oMf[model_->getFrameId(to_frame)].actInv(
                   data_->oMf[model_->getFrameId(from_frame)]))
              .toVector();
}

void RobotModel::get_offset(const std::string& from_frame,
                            const std::string& to_frame,
                            Eigen::Vector3d& offset) {
  offset = data_->oMf[model_->getFrameId(to_frame)].translation() -
           data_->oMf[model_->getFrameId(from_frame)].translation();
}

void RobotModel::get_error(const std::string& frame,
                           const Eigen::Quaterniond& rot,
                           const Eigen::Vector3d& trans,
                           Vector6d& error) const {
  error = log6(data_->oMf[model_->getFrameId(frame)].actInv(SE3(rot, trans)))
              .toVector();
}

Pose RobotModel::get_pose(const std::string& frame) const {
  Pose pose;
  pose.translation = data_->oMf[model_->getFrameId(frame)].translation();
  pose.rotation = data_->oMf[model_->getFrameId(frame)].rotation();
  return pose;
}

RobotModel::MinDistResult RobotModel::compute_min_distance(const Eigen::VectorXd& q, const Eigen::VectorXd* qdot,
                                                           bool with_grad, bool with_graddot, bool verbose) const {
  MinDistResult result; result.distance = std::numeric_limits<double>::infinity();
  if (!model_ || !geom_model_ || geom_model_->collisionPairs.empty()) return result;

  // Ensure kinematics & geometry caches (mutable flags allow const method)
  const_cast<RobotModel*>(this)->update_kinematics(q);
  const_cast<RobotModel*>(this)->update_geometry_cached();

  // Compute distances (Pinocchio computeDistances needs fresh geometry placements already updated)
  pinocchio::computeDistances(*model_, *data_, *geom_model_, *geom_data_);

  double minDistance = std::numeric_limits<double>::infinity();
  int minPairIdx = -1;
  for (std::size_t idx = 0; idx < geom_data_->distanceResults.size(); ++idx) {
    const auto &res = geom_data_->distanceResults[idx];
    if (res.min_distance < minDistance) { minDistance = res.min_distance; minPairIdx = static_cast<int>(idx); }
  }
  result.distance = minDistance; result.pair_index = minPairIdx;
  if (minPairIdx >= 0) {
    const auto &pair = geom_model_->collisionPairs[minPairIdx];
    result.link1 = geom_model_->geometryObjects[pair.first].name;
    result.link2 = geom_model_->geometryObjects[pair.second].name;
    if (verbose) {
      std::cout << "[RobotModel] Closest links: " << result.link1 << " <-> " << result.link2 << " | distance = " << minDistance << " m\n";
    }
  }
  if (!(with_grad || with_graddot) || minPairIdx < 0) return result;

  // Joint Jacobians (compute once per config)
  if (!joint_jacobians_valid_) {
    pinocchio::computeJointJacobians(*model_, *data_);
    joint_jacobians_valid_ = true;
  }

  const auto &pair = geom_model_->collisionPairs[minPairIdx];
  const int geomA = pair.first; const int geomB = pair.second;
  const int jointA = geom_model_->geometryObjects[geomA].parentJoint;
  const int jointB = geom_model_->geometryObjects[geomB].parentJoint;

  const auto &res = geom_data_->distanceResults[minPairIdx];
  Eigen::Vector3d pA = res.nearest_points[0];
  Eigen::Vector3d pB = res.nearest_points[1];
  Eigen::Vector3d n = (pB - pA); double n_norm = n.norm(); if (n_norm > 1e-12) n /= n_norm; else n.setZero();

  auto skew_matrix = [](const Eigen::Vector3d &v){ Eigen::Matrix3d m; m << 0,-v.z(),v.y(), v.z(),0,-v.x(), -v.y(),v.x(),0; return m; };

  Eigen::MatrixXd J_jointA = Eigen::MatrixXd::Zero(6, model_->nq);
  Eigen::MatrixXd J_jointB = Eigen::MatrixXd::Zero(6, model_->nq);
  pinocchio::getJointJacobian(*model_, *data_, jointA, pinocchio::LOCAL_WORLD_ALIGNED, J_jointA);
  pinocchio::getJointJacobian(*model_, *data_, jointB, pinocchio::LOCAL_WORLD_ALIGNED, J_jointB);

  Eigen::Vector3d rA = pA - data_->oMi[jointA].translation();
  Eigen::Vector3d rB = pB - data_->oMi[jointB].translation();
  Eigen::MatrixXd JA = J_jointA.topRows<3>() - skew_matrix(rA) * J_jointA.bottomRows<3>();
  Eigen::MatrixXd JB = J_jointB.topRows<3>() - skew_matrix(rB) * J_jointB.bottomRows<3>();

  result.grad = (n.transpose() * (JB - JA)).transpose();
  if (minDistance < 0) result.grad *= -1.0;

  if (with_graddot && qdot) {
    pinocchio::computeJointJacobiansTimeVariation(*model_, *data_, q, *qdot);
    Eigen::MatrixXd J_jointA_dot = Eigen::MatrixXd::Zero(6, model_->nq);
    Eigen::MatrixXd J_jointB_dot = Eigen::MatrixXd::Zero(6, model_->nq);
    pinocchio::getJointJacobianTimeVariation(*model_, *data_, jointA, pinocchio::LOCAL_WORLD_ALIGNED, J_jointA_dot);
    pinocchio::getJointJacobianTimeVariation(*model_, *data_, jointB, pinocchio::LOCAL_WORLD_ALIGNED, J_jointB_dot);

    Eigen::Vector3d pA_dot = JA * (*qdot);
    Eigen::Vector3d pB_dot = JB * (*qdot);
    Eigen::Vector3d rA_dot = pA_dot - J_jointA.topRows<3>() * (*qdot);
    Eigen::Vector3d rB_dot = pB_dot - J_jointB.topRows<3>() * (*qdot);
    Eigen::MatrixXd JA_dot = J_jointA_dot.topRows<3>() - (skew_matrix(rA_dot) * J_jointA.bottomRows<3>() + skew_matrix(rA) * J_jointA_dot.bottomRows<3>());
    Eigen::MatrixXd JB_dot = J_jointB_dot.topRows<3>() - (skew_matrix(rB_dot) * J_jointB.bottomRows<3>() + skew_matrix(rB) * J_jointB_dot.bottomRows<3>());

    result.grad_dot = (n.transpose() * (JB_dot - JA_dot)).transpose();
  }

  return result;
}

}  // namespace mppi_pinocchio