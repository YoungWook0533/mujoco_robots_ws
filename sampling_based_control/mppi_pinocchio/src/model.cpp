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
#include <hpp/fcl/collision.h>

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

    if (!srdf_path.empty()) {
      pinocchio::srdf::removeCollisionPairs(*model_, *geom_model_, srdf_path);
    }
    geom_data_ = new pinocchio::GeometryData(*geom_model_);
    return true;
  } catch (std::runtime_error& exc) {
    std::cout << exc.what();
    return false;
  } catch (...) {
    std::cout << "Unknown exception caught while building geometry model." << std::endl;
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

std::size_t RobotModel::count_self_collisions() const {
  if (!geom_model_ || !geom_data_) return 0;
  std::size_t count = 0;
  for (const auto& cp : geom_model_->collisionPairs) {
    const auto& go1 = geom_model_->geometryObjects[cp.first];
    const auto& go2 = geom_model_->geometryObjects[cp.second];
    const auto& M1 = geom_data_->oMg[cp.first];
    const auto& M2 = geom_data_->oMg[cp.second];

    // Use double-precision transforms to match FCL/COAL scalar type
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

}  // namespace mppi_pinocchio