#include "collision_checker/planning_scene_collision_check.h"

// PlanningSceneCollisionCheck::PlanningSceneCollisionCheck(const std::string & param_node_name, const std::string & node_name, const std::string & topic_name, const std::string & robot_description_param)
// {
//   if (!rclcpp::ok()) rclcpp::init(0, nullptr);
//   node_ = std::make_shared<rclcpp::Node>(node_name);

//   auto client = std::make_shared<rclcpp::SyncParametersClient>(node_, param_node_name);
//   if (!client->wait_for_service(std::chrono::seconds(5))) {
//       RCLCPP_ERROR(node_->get_logger(), "Failed to connect to parameter server in %s.",param_node_name);
//       return;
//   }
//   std::string robot_description;
//   std::string robot_description_semantic;
//   try {
//       robot_description = client->get_parameter<std::string>("robot_description");
//   } catch (const std::exception & e) {
//       RCLCPP_ERROR(node_->get_logger(), "Failed to get robot_description: %s", e.what());
//       return;
//   }
//   try {
//       robot_description_semantic = client->get_parameter<std::string>("robot_description_semantic");
//   } catch (const std::exception & e) {
//       RCLCPP_ERROR(node_->get_logger(), "Failed to get robot_description_semantic: %s", e.what());
//       return;
//   }

//   node_->declare_parameter<std::string>("robot_description", "");
//   node_->set_parameter(rclcpp::Parameter("robot_description", robot_description));
//   node_->declare_parameter<std::string>("robot_description_semantic", "");
//   node_->set_parameter(rclcpp::Parameter("robot_description_semantic", robot_description_semantic));



//   robot_model_loader::RobotModelLoader robot_model_loader(node_, robot_description_param, false);
//   robot_model_ = robot_model_loader.getModel();
//   planning_scene_ = std::make_shared<planning_scene::PlanningScene> (robot_model_);
//   planning_scene_->setName("srmt2 planning scene");
//   scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(topic_name, 1);
//   planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, planning_scene_, robot_description_param);
//   planning_scene_monitor_->providePlanningSceneService();
//   planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, topic_name);
//   planning_scene_monitor_->startSceneMonitor(topic_name);
// };

PlanningSceneCollisionCheck::PlanningSceneCollisionCheck(const std::string& node_name, 
                                                         const std::string& topic_name, 
                                                         const std::string& urdf_string,
                                                         const std::string& srdf_string)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp::Node>(node_name);
  
  node_->declare_parameter<std::string>("robot_description", "");
  node_->set_parameter(rclcpp::Parameter("robot_description", urdf_string));
  node_->declare_parameter<std::string>("robot_description_semantic", "");
  node_->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_string));
  
  robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description", false);
  robot_model_ = robot_model_loader.getModel();
  planning_scene_ = std::make_shared<planning_scene::PlanningScene> (robot_model_);
  planning_scene_->setName("srmt2 planning scene");
  scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(topic_name,rclcpp::QoS(1).reliable().durability_volatile());
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, planning_scene_, "robot_description");
  planning_scene_monitor_->providePlanningSceneService();
  planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, topic_name);
  planning_scene_monitor_->startSceneMonitor(topic_name);

  planning_scene_monitor_->setStateUpdateFrequency(100.0);
  planning_scene_monitor_->setPlanningScenePublishingFrequency(100.0);

  buildPerLinkACMs();
};

void PlanningSceneCollisionCheck::setGroupNamesAndDofs(const std::vector<std::string> &arm_name, const std::vector<int> & dofs)
{
  assert(arm_name.size() == dofs.size());

  int len_groups = arm_name.size();
  group_infos_.resize(len_groups);

  for (int i=0; i<len_groups; ++i)
  {
    group_infos_[i].first = arm_name[i];
    group_infos_[i].second = dofs[i];
  }

}

bool PlanningSceneCollisionCheck::isValid(const Eigen::Ref<const Eigen::VectorXd> &q) const
{
  std::scoped_lock _lock(planning_scene_mtx_);
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
    
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    moveit::core::RobotState current_state = planning_scene_->getCurrentState();
    
    int current_seg_index = 0;
    for (auto & group_info : group_infos_)
    {
      int dof = group_info.second;
      const auto & q_seg = q.segment(current_seg_index, dof);
      const std::vector<double> joint_values(q_seg.data(), q_seg.data() + dof);
      const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(group_info.first);
      current_state.setJointGroupPositions(joint_model_group, joint_values);
      current_seg_index += dof;
    }
    planning_scene_->checkCollision(collision_request, collision_result, current_state);

    last_collision_result_ = collision_result; 

    return !collision_result.collision;
  }
}

bool PlanningSceneCollisionCheck::isCurrentValid() const
{
  std::scoped_lock _lock(planning_scene_mtx_);
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    moveit::core::RobotState current_state = planning_scene_->getCurrentState();
    
    planning_scene_->checkCollision(collision_request, collision_result, current_state);

    last_collision_result_ = collision_result; 
    
    return !collision_result.collision;
  }
}

void PlanningSceneCollisionCheck::setJointGroupPositions(const std::string& name, const Eigen::Ref<const Eigen::VectorXd> &q)
{
  std::scoped_lock _lock(planning_scene_mtx_);
  {
    planning_scene_monitor::LockedPlanningSceneRW lscene(planning_scene_monitor_);

    moveit::core::RobotState & current_state = planning_scene_->getCurrentStateNonConst();
    current_state.setJointGroupPositions(name, q);
  }
}

double PlanningSceneCollisionCheck::clearance(const Eigen::Ref<const Eigen::VectorXd> &q) const
{
  std::scoped_lock _lock(planning_scene_mtx_);
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.distance = true;
    moveit::core::RobotState current_state = planning_scene_->getCurrentState();

    int current_seg_index = 0;
    for (auto & group_info : group_infos_)
    {
      int dof = group_info.second;
      const auto & q_seg = q.segment(current_seg_index, dof);
      const std::vector<double> joint_values(q_seg.data(), q_seg.data() + dof);
      const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(group_info.first);
      current_state.setJointGroupPositions(joint_model_group, joint_values);
      current_seg_index += dof;
    }

    planning_scene_->checkCollision(collision_request, collision_result, current_state);

    last_collision_result_ = collision_result; 
    
    if (collision_result.collision)
    {
      return 0.0;
    }

    return collision_result.distance;
  }
}

void PlanningSceneCollisionCheck::updateJoints(const Eigen::Ref<const Eigen::VectorXd> &q)
{
  planning_scene_monitor::LockedPlanningSceneRW lscene(planning_scene_monitor_);

  moveit::core::RobotState & current_state = planning_scene_->getCurrentStateNonConst();

  int current_seg_index = 0;
  for (auto & group_info : group_infos_)
  {
    int dof = group_info.second;
    const auto & q_seg = q.segment(current_seg_index, dof);
    current_state.setJointGroupPositions(group_info.first, q_seg);
    current_seg_index += dof;
  }
}

geometry_msgs::msg::Pose PlanningSceneCollisionCheck::convertEigenToPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = pos(0);
  pose.position.y = pos(1);
  pose.position.z = pos(2);
  pose.orientation.x = quat(0);
  pose.orientation.y = quat(1);
  pose.orientation.z = quat(2);
  pose.orientation.w = quat(3);
  return pose;
}

void PlanningSceneCollisionCheck::addMeshFromFile(const std::string & file_name, const std::string &id, 
                      const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::msg::Pose pose;
  pose = convertEigenToPose(pos, quat);
  addMeshFromFile(file_name, pose, id);
}

void PlanningSceneCollisionCheck::addMeshFromFile(const std::string & file_name, geometry_msgs::msg::Pose pose, const std::string &id)
{
  shapes::Mesh *mesh = shapes::createMeshFromResource(file_name);
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(mesh, shape_msg);

  shape_msgs::msg::Mesh shape_msgs_mesh = boost::get<shape_msgs::msg::Mesh>(shape_msg);

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = obs_frame_id_;
  co.id = id;
  geometry_msgs::msg::Pose empty_pose;
  empty_pose.orientation.w = 1.0;
  co.meshes.push_back(shape_msgs_mesh);
  co.mesh_poses.push_back(empty_pose);
  co.pose = pose;
  co.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processCollisionObjectMsg(co);
}

void PlanningSceneCollisionCheck::updateObjectPose(geometry_msgs::msg::Pose pose, const std::string &id)
{
  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = obs_frame_id_;
  co.id = id;
// #if ROS_VERSION_MINOR <= 14
  // co.mesh_poses.push_back(pose);
// #else // >= 15
  co.pose = pose;
// #endif
  co.operation = moveit_msgs::msg::CollisionObject::MOVE;

  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processCollisionObjectMsg(co);
}

void PlanningSceneCollisionCheck::updateObjectPose(const std::string &id, const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::msg::Pose pose;
  pose = convertEigenToPose(pos, quat);
  updateObjectPose(pose, id);
}

Eigen::Isometry3d PlanningSceneCollisionCheck::getObjectPose(const std::string &id) const
{
  std::scoped_lock _lock(planning_scene_mtx_);
  const auto T = planning_scene_->getWorld()->getObject(id)->shape_poses_[0];
  return T;
}

void PlanningSceneCollisionCheck::addBox(const Eigen::Ref<const Eigen::Vector3d> &dim, const std::string &id,
            const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::msg::Pose pose;
  pose = convertEigenToPose(pos, quat);
  addBox(dim, pose, id);
}

void PlanningSceneCollisionCheck::addBox(const Eigen::Ref<const Eigen::Vector3d> &dim, geometry_msgs::msg::Pose pose, const std::string &id)
{
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = dim(0);
  primitive.dimensions[1] = dim(1);
  primitive.dimensions[2] = dim(2);

  geometry_msgs::msg::Pose empty_pose;
  empty_pose.orientation.w = 1.0;

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = obs_frame_id_;
  co.id = id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(empty_pose);
  co.pose = pose;
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processCollisionObjectMsg(co);
}

void PlanningSceneCollisionCheck::addCylinder(const Eigen::Ref<const Eigen::Vector2d> &dim, const std::string &id,
            const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::msg::Pose pose;
  pose = convertEigenToPose(pos, quat);
  addCylinder(dim, pose, id);
}

void PlanningSceneCollisionCheck::addCylinder(const Eigen::Ref<const Eigen::Vector2d> &dim, geometry_msgs::msg::Pose pose, const std::string &id)
{
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = dim(0);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = dim(1);

  geometry_msgs::msg::Pose empty_pose;
  empty_pose.orientation.w = 1.0;

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = obs_frame_id_;
  co.id = id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(empty_pose);
  co.pose = pose;
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processCollisionObjectMsg(co);
}

void PlanningSceneCollisionCheck::addSphere(const double &dim, const std::string &id,
            const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::msg::Pose pose;
  pose = convertEigenToPose(pos, quat);
  addSphere(dim, pose, id);
}

void PlanningSceneCollisionCheck::addSphere(const double &dim, geometry_msgs::msg::Pose pose, const std::string &id)
{
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;
  primitive.dimensions.resize(1);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = dim;

  geometry_msgs::msg::Pose empty_pose;
  empty_pose.orientation.w = 1.0;

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = obs_frame_id_;
  co.id = id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(empty_pose);
  co.pose = pose;
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processCollisionObjectMsg(co);
}

void PlanningSceneCollisionCheck::attachObject(const std::string &object_id, const std::string &link_name, const std::vector<std::string> &touch_links)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.object.id = object_id;
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  aco.object.pose.orientation.w = 1.0;
  aco.link_name = link_name;
  aco.touch_links = touch_links;

  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processAttachedCollisionObjectMsg(aco);
}

void PlanningSceneCollisionCheck::detachObject(const std::string &object_id, const std::string &link_name)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.object.id = object_id;
  aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  aco.object.pose.orientation.w = 1.0;
  aco.link_name = link_name;

  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processAttachedCollisionObjectMsg(aco);
}

void PlanningSceneCollisionCheck::detachAllObjects(const std::string & link_name)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  aco.link_name = link_name;

  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processAttachedCollisionObjectMsg(aco);
}

void PlanningSceneCollisionCheck::removeObject(const std::string & object_id)
{
  moveit_msgs::msg::CollisionObject co;
  co.id = object_id;
  co.header.frame_id = obs_frame_id_;
  co.header.stamp = node_->get_clock()->now();
  co.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processCollisionObjectMsg(co);

}

void PlanningSceneCollisionCheck::removeAllObjects()
{
  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->removeAllCollisionObjects();
}

std::vector<std::string> PlanningSceneCollisionCheck::getAllAttachedObjects()
{
  std::vector<std::string> attached_objects;
  std::vector<moveit_msgs::msg::AttachedCollisionObject> acos;
  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->getAttachedCollisionObjectMsgs(acos);
  
  for (auto & aco : acos)
  {
    attached_objects.push_back(aco.object.id);
  }
  return attached_objects;
}

void PlanningSceneCollisionCheck::changeCollision(const std::string &name1, const std::string &name2, bool allowed)
{
  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->getAllowedCollisionMatrixNonConst().setEntry(name1, name2, allowed);
}

void PlanningSceneCollisionCheck::changeCollisions(const std::string &name1, const std::vector< std::string > &other_names, bool allowed)
{
  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->getAllowedCollisionMatrixNonConst().setEntry(name1, other_names, allowed);
}

void PlanningSceneCollisionCheck::changeCollisionsAll(const std::string &name1, bool allowed)
{
  planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->getAllowedCollisionMatrixNonConst().setEntry(name1, allowed);
}

Eigen::Isometry3d PlanningSceneCollisionCheck::geometry_pose_to_isometry(geometry_msgs::msg::Pose geometry_pose)
{
  Eigen::Quaterniond quat_;
  Eigen::Isometry3d transform_;
  quat_.x() = geometry_pose.orientation.x;
  quat_.y() = geometry_pose.orientation.y;
  quat_.z() = geometry_pose.orientation.z;
  quat_.w() = geometry_pose.orientation.w;
  transform_.linear() = quat_.toRotationMatrix();
  transform_.translation() << geometry_pose.position.x, geometry_pose.position.y, geometry_pose.position.z;
  return transform_;
}

moveit_msgs::msg::PlanningScene PlanningSceneCollisionCheck::getPlanningSceneMsg()
{
  moveit_msgs::msg::PlanningScene scene_msgs;
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getPlanningSceneMsg(scene_msgs);
  return scene_msgs;
}

void PlanningSceneCollisionCheck::publishPlanningSceneMsg()
{
  moveit_msgs::msg::PlanningScene scene_msg;
  planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getPlanningSceneDiffMsg(scene_msg);
  scene_pub_->publish(scene_msg);
  planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
}

void PlanningSceneCollisionCheck::printCurrentCollisionInfos()
{
  std::cout << streamCurrentCollisionInfos().str();
}

std::stringstream PlanningSceneCollisionCheck::streamCurrentCollisionInfos()
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  {
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
  
  moveit::core::RobotState current_state = planning_scene_->getCurrentState();
  
  collision_request.contacts = true;
  collision_request.distance = false;
  collision_request.verbose = false;
  planning_scene_->checkCollision(collision_request, collision_result, current_state);
  }
  std::stringstream ss;
  ss << "===================================\n ";
  ss << " > Current collision status: " << collision_result.collision << std::endl;
  for (auto & contact : collision_result.contacts)
  {
    ss << "    > Contact : " << contact.first.first << " - " << contact.first.second << std::endl;
  //             << "      > contact pos :"  << contact.second.
  // Eigen::Vector3d pos;
  }
  ss << "===================================\n ";
  return ss;
}

planning_scene::PlanningScenePtr& PlanningSceneCollisionCheck::getPlanningScene()
{
  return planning_scene_;
}

planning_scene_monitor::PlanningSceneMonitorPtr& PlanningSceneCollisionCheck::getPlanningSceneMonitor()
{
  return planning_scene_monitor_;
}


bool PlanningSceneCollisionCheck::timeParameterize(const Eigen::Ref<const Eigen::MatrixXd>& path,
  Eigen::Ref<Eigen::MatrixXd> q_result,
  Eigen::Ref<Eigen::MatrixXd> qdot_result,
  Eigen::Ref<Eigen::MatrixXd> qddot_result,
  Eigen::Ref<Eigen::VectorXd> time_result,
  const double max_velocity_scaling_factor,
  const double max_acceleration_scaling_factor)
{
  trajectory_processing::IterativeSplineParameterization time_parameterization(true);
  int len_path = path.rows();
  int len_q = path.cols();

  int len_traj = len_path + 2;
  assert (q_result.rows() == len_traj);
  assert (qdot_result.rows() == len_traj);
  assert (qddot_result.rows() == len_traj);
  assert (time_result.rows() == len_traj);
  assert (q_result.cols() == len_q);
  assert (qdot_result.cols() == len_q);
  assert (qddot_result.cols() == len_q);
  assert (time_result.cols() == 1);
  // q_result.setZero(len_traj, len_q);
  // qdot_result.setZero(len_traj, len_q);
  // qddot_result.setZero(len_traj, len_q);
  // time_result.setZero(len_traj);

  int current_seg_index = 0;
  for (auto & group_info : group_infos_)
  {
    int dof = group_info.second;
    const auto & path_seg = path.block(0,current_seg_index, path.rows(), dof);
    robot_trajectory::RobotTrajectory traj(robot_model_, group_info.first);
    moveit::core::RobotState state(traj.getRobotModel());
    const moveit::core::JointModelGroup* group = traj.getGroup();
    const std::vector<int>& idices = group->getVariableIndexList();

    for (size_t i=0; i<len_path; i++)
    { 
      for (size_t j=0; j<len_q; j++)
      {
        state.setVariablePosition(idices[j],path(i, j));
      }
      traj.addSuffixWayPoint(state, time_result(i+1));
    }
    bool result = time_parameterization.computeTimeStamps(traj, max_velocity_scaling_factor, max_acceleration_scaling_factor);

    for (size_t i=0; i<len_traj; i++)
    {
      for (size_t j=0; j<len_q; j++)
      {
        q_result    (i,j) = traj.getWayPoint(i).getVariablePosition(idices[j]);
        qdot_result (i,j) = traj.getWayPoint(i).getVariableVelocity(idices[j]);
        qddot_result(i,j) = traj.getWayPoint(i).getVariableAcceleration(idices[j]);
      }
      time_result(i) = traj.getWayPointDurationFromStart(i);
    }
    current_seg_index += dof;
  }  
  return true;
}

double PlanningSceneCollisionCheck::getMinimumDistance(bool is_self, bool is_env)
{
  {
    std::scoped_lock _lock(planning_scene_mtx_);
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
    //---------------------------------------------------------------
    // Build a generic distance request
    //---------------------------------------------------------------
    collision_detection::DistanceRequest req;
    req.type                   = collision_detection::DistanceRequestTypes::GLOBAL;
    req.enable_nearest_points  = false;
    req.enable_signed_distance = true;
    req.acm                    = &planning_scene_->getAllowedCollisionMatrix();

    double min_dist = std::numeric_limits<double>::infinity();
    const moveit::core::RobotState& state = planning_scene_->getCurrentState();

    //---------------------------------------------------------------
    // 1. Robot self‑distance (link ↔ link)
    //---------------------------------------------------------------
    if (is_self)
    {
      collision_detection::DistanceResult res;
      planning_scene_->getCollisionEnv()->distanceSelf(req, res, state);
      min_dist = std::min(min_dist, res.minimum_distance.distance);
    }

    //---------------------------------------------------------------
    // 2. Robot ↔ environment distance
    //---------------------------------------------------------------
    if (is_env)
    {
      collision_detection::DistanceResult res;
      planning_scene_->getCollisionEnv()->distanceRobot(req, res, state);
      min_dist = std::min(min_dist, res.minimum_distance.distance);
    }
    return min_dist; // +∞ if neither branch ran or no result available
  }
}

void PlanningSceneCollisionCheck::buildPerLinkACMs()
{
  const std::vector<std::string>& link_names = planning_scene_monitor_->getRobotModel()->getLinkModelNames();
  link_acm_map_.clear();
  
  for (const auto& target : link_names)
  {
    std::cout <<"link: "<< target << std::endl;
    auto acm_copy = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix();
    
    for (const auto& link1 : link_names)
    {
      for (const auto& link2 : link_names)
      {
        if (link1 != target && link2 != target)
        {
          acm_copy.setEntry(link1, link2, true);
        }
      }
    }
    link_acm_map_.emplace(target, std::move(acm_copy));
  }
}

Eigen::VectorXd PlanningSceneCollisionCheck::getLinksMinDistances(const std::vector<std::string>& target_links,
                                                                  bool is_self,
                                                                  bool is_env) const
{
  const std::size_t N = target_links.size();
  Eigen::VectorXd min_dists(N);
  min_dists.setConstant(std::numeric_limits<double>::infinity());

  if (!is_self && !is_env)
    return min_dists;

  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
  const auto& state = planning_scene_->getCurrentState();

  collision_detection::DistanceRequest req;
  req.type                   = collision_detection::DistanceRequestTypes::GLOBAL;
  req.enable_nearest_points  = false;
  req.enable_signed_distance = true;

  for (std::size_t i = 0; i < N; ++i)
  {
    const std::string& link_name = target_links[i];
    const auto* lm = planning_scene_->getRobotModel()->getLinkModel(link_name);
    if (!lm) {
      RCLCPP_WARN(node_->get_logger(),
                  "[getLinksMinDistances] unknown link '%s'", link_name.c_str());
      continue;
    }

    double best = std::numeric_limits<double>::infinity();
    collision_detection::DistanceResult res;

    if (is_self)
    {
      req.acm = &link_acm_map_.at(link_name);
      res.clear();
      planning_scene_->getCollisionEnv()->distanceSelf(req, res, state);
      best = std::min(best, res.minimum_distance.distance);
    }

    if (is_env)
    {
      std::set<const moveit::core::LinkModel*> active{ lm };
      req.acm = &planning_scene_->getAllowedCollisionMatrix();
      req.active_components_only = &active;
      res.clear();
      planning_scene_->getCollisionEnv()->distanceRobot(req, res, state);
      best = std::min(best, res.minimum_distance.distance);
    }

    min_dists(i) = best;
  }

  return min_dists;
}
