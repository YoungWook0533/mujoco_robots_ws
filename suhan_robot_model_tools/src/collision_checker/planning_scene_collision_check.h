#pragma once

#include <mutex>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <fstream>

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#define DEBUG_FILE(text) \
if(debug_file_.is_open()) \
{ \
  debug_file_ << text << std::endl;\
}

class PlanningSceneCollisionCheck
{
public:
  // PlanningSceneCollisionCheck(const std::string & param_node_name, 
  //                             const std::string & node_name, 
  //                             const std::string & topic_name, 
  //                             const std::string & robot_description_param = "robot_description");
  PlanningSceneCollisionCheck(const std::string& node_name, 
                              const std::string& topic_name, 
                              const std::string& urdf_string,
                              const std::string& srdf_string);
  void setGroupNamesAndDofs(const std::vector<std::string> &arm_name, const std::vector<int> & dofs);
  bool isValid(const Eigen::Ref<const Eigen::VectorXd> &q) const;
  bool isCurrentValid() const;
  double clearance(const Eigen::Ref<const Eigen::VectorXd> &q) const;

  void updateJoints(const Eigen::Ref<const Eigen::VectorXd> &q);
  geometry_msgs::msg::Pose convertEigenToPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
  void addMeshFromFile(const std::string & file_name, const std::string &id, 
                       const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

  void addMeshFromFile(const std::string & file_name, geometry_msgs::msg::Pose pose, const std::string &id);
  void updateObjectPose(geometry_msgs::msg::Pose pose, const std::string &id);
  Eigen::Isometry3d getObjectPose(const std::string &id) const;
  void updateObjectPose(const std::string &id, const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

  void addBox(const Eigen::Ref<const Eigen::Vector3d> &dim, const std::string &id,
              const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

  void addBox(const Eigen::Ref<const Eigen::Vector3d> &dim, geometry_msgs::msg::Pose pose, const std::string &id);

  void addCylinder(const Eigen::Ref<const Eigen::Vector2d> &dim, const std::string &id,
              const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

  void addCylinder(const Eigen::Ref<const Eigen::Vector2d> &dim, geometry_msgs::msg::Pose pose, const std::string &id);

  void addSphere(const double &dim, const std::string &id,
              const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

  void addSphere(const double &dim, geometry_msgs::msg::Pose pose, const std::string &id);
  
  void attachObject(const std::string &object_id, const std::string &link_name, const std::vector<std::string> &touch_links);
  void detachObject(const std::string &object_id, const std::string &link_name);
  void detachAllObjects(const std::string & link_name);
  void removeObject(const std::string & object_id);
  void removeAllObjects();
  std::vector<std::string> getAllAttachedObjects();
  void changeCollision(const std::string &name1, const std::string &name2, bool allowed);
  void changeCollisions(const std::string &name1, const std::vector< std::string > &other_names, bool allowed);
  void changeCollisionsAll(const std::string &name1, bool allowed);
  Eigen::Isometry3d geometry_pose_to_isometry(geometry_msgs::msg::Pose geometry_pose);

  moveit_msgs::msg::PlanningScene getPlanningSceneMsg();
  void publishPlanningSceneMsg();

  void printCurrentCollisionInfos();
  std::stringstream streamCurrentCollisionInfos();
  double getMinimumDistance(bool is_self, bool is_env);

  void setJointGroupPositions(const std::string& name, const Eigen::Ref<const Eigen::VectorXd> &q);
  void setFrameID(const std::string &frame_id) { obs_frame_id_ = frame_id; }

  void openDebugFile() { debug_file_.open( debug_file_prefix_ + "planning_scene_debug.txt"); }
  void setDebugFilePrefix(const std::string &name) { debug_file_prefix_ = name; }

  planning_scene::PlanningScenePtr& getPlanningScene();
  planning_scene_monitor::PlanningSceneMonitorPtr& getPlanningSceneMonitor();

  bool timeParameterize(const Eigen::Ref<const Eigen::MatrixXd>& path,
                        Eigen::Ref<Eigen::MatrixXd> q_result,
                        Eigen::Ref<Eigen::MatrixXd> qdot_result,
                        Eigen::Ref<Eigen::MatrixXd> qddot_result,
                        Eigen::Ref<Eigen::VectorXd> time_result,
                        const double max_velocity_scaling_factor = 1.0,
                        const double max_acceleration_scaling_factor = 1.0);

                        
  void buildPerLinkACMs();
  Eigen::VectorXd getLinksMinDistances(const std::vector<std::string>& target_links,
                                       bool is_self,
                                       bool is_env) const;

private:
  std::vector<std::pair<std::string,int>> group_infos_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
  std::string debug_file_prefix_;
  std::ofstream debug_file_;
  std::string obs_frame_id_;

  mutable std::mutex planning_scene_mtx_;
  mutable collision_detection::CollisionResult last_collision_result_;

  std::unordered_map<std::string, collision_detection::AllowedCollisionMatrix> link_acm_map_;
};