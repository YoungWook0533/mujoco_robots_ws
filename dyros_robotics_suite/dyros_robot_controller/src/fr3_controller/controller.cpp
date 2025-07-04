#include "fr3_controller/controller.h"
#include <Eigen/Dense>
#include "math_type_define.h"

namespace FR3Controller
{
    Controller::Controller(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd)
    : ControllerInterface(node, dt, std::move(jd))
    {
      std::string urdf_path = ament_index_cpp::get_package_share_directory("dyros_robot_controller")
                              + "/robot/fr3.urdf";
      robot_ = std::make_unique<RobotData>(urdf_path, true);
      rd_joint_names_ = robot_->getJointNames();

      key_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
                 "fr3_controller/mode_input", 10,
                 std::bind(&Controller::keyCallback, this, std::placeholders::_1));
      target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                         "fr3_controller/target_pose", 10,
                         std::bind(&Controller::subtargetPoseCallback, this, std::placeholders::_1));
      cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                 "fr3_controller/cmd_vel", 10,
                 std::bind(&Controller::cmdvelCallback, this, std::placeholders::_1));                   
      ee_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                     "fr3_controller/ee_pose", 10);
    }

    Controller::~Controller()
    {
      if (slow_thread_.joinable()) slow_thread_.join();
    }
    
    void Controller::starting()
    {
      is_mode_changed_ = false;
      mode_ = "home";
      control_start_time_ = current_time_;

      q_init_ = q_;
      qdot_init_ = qdot_;
      x_init_ = x_;
      xdot_init_ = xdot_;

      q_desired_ = q_init_;
      qdot_desired_.setZero();
      tau_desired_.setZero();
      x_desired_ = x_init_;
      xdot_desired_.setZero();

      ee_pose_pub_timer_ = node_->create_wall_timer(
                           std::chrono::milliseconds(100),
                           std::bind(&Controller::pubEEPoseCallback, this)
                           );

      slow_thread_ = std::thread(&Controller::computeSlowLoop, this);



      /* Helper variables for keyboard teleop */
      cmd_vel_.setZero(6);

      q_min_.resize(7);
      q_min_ <<  -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
      q_max_.resize(7);
      q_max_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.897;

      // joint velocity limits
      qdot_limit_.resize(7);
      qdot_limit_ << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;

      // joint torque limits
      tau_limit_.resize(7);
      tau_limit_ << 87, 87, 87, 87, 12, 12, 12;

      joint_position_max.resize(7);
      joint_position_min.resize(7);
      joint_velocity_limits.resize(7);
      joint_torques_limits.resize(7);

      // damping gains
      // kv_safety = {20.0, 20.0, 20.0, 15.0, 10.0, 10.0, 5.0};
      kv_safety.resize(7);
      kv_safety << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 1.0;

      pos_zones.resize(2);
      vel_zones.resize(2);

      // zone definitions
      pos_zones << 6., 9.;  // hard, soft
      // vel_zones = {5., 7.};  // hard, soft
      vel_zones << 8., 10.;  // hard, soft  (8, 6)

      double default_sf = 0.98;  // max violation safety factor 
      for (int i = 0; i < 7; ++i) {
          joint_position_max[i] = default_sf * q_max_[i];
          joint_position_min[i] = default_sf * q_min_[i];
          joint_velocity_limits[i] = default_sf * qdot_limit_[i];
          joint_torques_limits[i] = default_sf * tau_limit_[i];
      }

      soft_min_angles.resize(7);
      soft_max_angles.resize(7);
      hard_min_angles.resize(7);
      hard_max_angles.resize(7);
      soft_min_joint_velocity_limits.resize(7);
      hard_min_joint_velocity_limits.resize(7);
      soft_max_joint_velocity_limits.resize(7);
      hard_max_joint_velocity_limits.resize(7);

      soft_min_angles.setZero();
      soft_max_angles.setZero();
      hard_min_angles.setZero();
      hard_max_angles.setZero();
      soft_min_joint_velocity_limits.setZero();
      hard_min_joint_velocity_limits.setZero();
      soft_max_joint_velocity_limits.setZero();
      hard_max_joint_velocity_limits.setZero();

      for (int i = 0; i < 7; ++i) {
          soft_min_angles[i] = joint_position_min[i] + pos_zones[1] * angle_tol;
          hard_min_angles[i] = joint_position_min[i] + pos_zones[0] * angle_tol;
          soft_max_angles[i] = joint_position_max[i] - pos_zones[1] * angle_tol;
          hard_max_angles[i] = joint_position_max[i] - pos_zones[0] * angle_tol;
          soft_min_joint_velocity_limits[i] = - joint_velocity_limits[i] + vel_zones[1] * vel_tol;
          hard_min_joint_velocity_limits[i] = - joint_velocity_limits[i] + vel_zones[0] * vel_tol;
          soft_max_joint_velocity_limits[i] = joint_velocity_limits[i] - vel_zones[1] * vel_tol;
          hard_max_joint_velocity_limits[i] = joint_velocity_limits[i] - vel_zones[0] * vel_tol;
      }

      pos_limit_flag.resize(7); 
      pos_limit_flag.setZero();
      vel_limit_flag.resize(7); 
      vel_limit_flag.setZero();
      limited_joints.resize(7); 
      limited_joints.setZero();

      x_dot_lfp_ = VectorXd::Zero(6);
      theta_x_ = 0.0;
      theta_y_ = 0.0;
      theta_z_ = 0.0;
    }

    void Controller::updateState(const VecMap& pos_dict, 
                                 const VecMap& vel_dict,
                                 const VecMap& tau_ext_dict, 
                                 const VecMap& sensors_dict, 
                                 double current_time)
    {
      current_time_ = current_time;

      for(size_t i=0; i<rd_joint_names_.size(); i++)
      {
        const auto &name = rd_joint_names_[i];
        q_(i) = pos_dict.at(name)(0);
        qdot_(i) = vel_dict.at(name)(0);
        tau_ext_(i) = tau_ext_dict.at(name)(0);
      }
      if(!robot_->updateState(q_, qdot_))
      {
        RCLCPP_ERROR(node_->get_logger(), "[FR3RobotData] Failed to update robot state.");
      }
      x_.matrix() = robot_->getPose();
      xdot_ = robot_->getVelocity();
    }

    void Controller::compute()
    {
      if(is_mode_changed_)
      {
        is_mode_changed_ = false;
        control_start_time_ = current_time_;
        is_goal_pose_changed_ = false;

        q_init_ = q_;
        qdot_init_ = qdot_;
        x_init_ = x_;
        xdot_init_ = xdot_;
        q_desired_ = q_init_;
        qdot_desired_.setZero();
        x_desired_ = x_init_;
        xdot_desired_.setZero();
        x_goal_ = x_init_;

        joint_safety_mode_flag = false;
      }

      if(mode_ == "init")
      {
        Vector7d target_q;
        target_q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4.;
        q_desired_ = DyrosMath::cubicVector<7>(current_time_, 
                                               control_start_time_,
                                               control_start_time_ + 2.0, 
                                               q_init_, 
                                               target_q, 
                                               Vector7d::Zero(), 
                                               Vector7d::Zero());
        qdot_desired_ = DyrosMath::cubicDotVector<7>(current_time_, 
                                                     control_start_time_,
                                                     control_start_time_ + 2.0, 
                                                     q_init_, 
                                                     target_q, 
                                                     Vector7d::Zero(), 
                                                     Vector7d::Zero());
        tau_desired_ = PDControl(q_desired_, qdot_desired_, Vector7d::Constant(400),Vector7d::Constant(40));
      }
      else if(mode_ == "home")
      {
        Vector7d target_q;
        target_q << 0.0, 0.0, 0.0, -M_PI/2., 0.0, M_PI/2., M_PI / 4.;
        q_desired_ = DyrosMath::cubicVector<7>(current_time_, 
                                               control_start_time_,
                                               control_start_time_ + 2.0, 
                                               q_init_, 
                                               target_q, 
                                               Vector7d::Zero(), 
                                               Vector7d::Zero());
        qdot_desired_ = DyrosMath::cubicDotVector<7>(current_time_, 
                                                     control_start_time_,
                                                     control_start_time_ + 2.0, 
                                                     q_init_, 
                                                     target_q, 
                                                     Vector7d::Zero(), 
                                                     Vector7d::Zero());
        tau_desired_ = PDControl(q_desired_, qdot_desired_, Vector7d::Constant(400),Vector7d::Constant(40));
      }
      else if(mode_ == "clik")
      {
        q_desired_ = CLIK(x_goal_, 2.0);
        tau_desired_ = PDControl(q_desired_, qdot_desired_, Vector7d::Constant(400),Vector7d::Constant(40));
      }
      else if(mode_ == "keyboard_teleop")
      {
        tau_desired_ = EEteleop(cmd_vel_);
      }
      else
      {
        tau_desired_.setZero();
      }
    }

    CtrlInputMap Controller::getCtrlInput() const
    {
      CtrlInputMap ctrl_dict;
      for(size_t i=0; i<rd_joint_names_.size(); i++)
      {
        const auto &name = rd_joint_names_[i];
        ctrl_dict[name] = tau_desired_[i];
      }
      return ctrl_dict;
    }

    void Controller::setMode(const std::string& mode)
    {
      is_mode_changed_ = true;
      mode_ = mode;
      RCLCPP_INFO(node_->get_logger(), "[FR3Controller] Mode changed: %s", mode.c_str());
    }


    void Controller::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(), "[FR3Controller] Key input received: %d", msg->data);
      if(msg->data == 1)      setMode("init");
      else if(msg->data == 2) setMode("home");
      else if(msg->data == 3) setMode("clik");
      else if(msg->data == 4) setMode("keyboard_teleop");
      else                    setMode("none");
  
    }
    void Controller::subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(),
              "[FR3Controller] Target pose received: position=(%.3f, %.3f, %.3f), "
              "orientation=(%.3f, %.3f, %.3f, %.3f)",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
              msg->pose.orientation.x, msg->pose.orientation.y,
              msg->pose.orientation.z, msg->pose.orientation.w);
      is_goal_pose_changed_ = true;

      // Convert to 4x4 homogeneous transform
      Eigen::Quaterniond quat(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
      Eigen::Matrix3d R = quat.toRotationMatrix();

      x_goal_.linear() = R;
      x_goal_.translation() << msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z;
    }
    void Controller::cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      cmd_vel_(0) = msg->linear.x;
      cmd_vel_(1) = msg->linear.y;
      cmd_vel_(2) = msg->linear.z;
      cmd_vel_(3) = msg->angular.x;
      cmd_vel_(4) = msg->angular.y;
      cmd_vel_(5) = msg->angular.z;
      
    }
    void Controller::pubEEPoseCallback()
    {
      Eigen::Matrix4d T = x_.matrix();

      auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
      ee_pose_msg.header.frame_id = "fr3_link0";
      ee_pose_msg.header.stamp = node_->now();

      ee_pose_msg.pose.position.x = T(0, 3);
      ee_pose_msg.pose.position.y = T(1, 3);
      ee_pose_msg.pose.position.z = T(2, 3);

      Eigen::Matrix3d R = T.block<3,3>(0,0);
      Eigen::Quaterniond q(R);
      ee_pose_msg.pose.orientation.x = q.x();
      ee_pose_msg.pose.orientation.y = q.y();
      ee_pose_msg.pose.orientation.z = q.z();
      ee_pose_msg.pose.orientation.w = q.w();
      
      ee_pose_pub_->publish(ee_pose_msg);
    }
    
    void Controller::computeSlowLoop()
    {
      rclcpp::Rate r(1000);
      while (rclcpp::ok()) {
        r.sleep();
      }
    }

    /* Controller functions */
    
    Vector7d Controller::PDControl(const Vector7d& q_desired, 
                                   const Vector7d& qdot_desired, 
                                   const Vector7d& kp,
                                   const Vector7d& kv)
    {
      Vector7d q_error = q_desired - q_;
      Vector7d qdot_error = qdot_desired - qdot_;
      return robot_->getMassMatrix() * (kp.asDiagonal() * q_error + kv.asDiagonal() * qdot_error) + robot_->getCoriolis();
    }

    Vector7d Controller::CLIK(const Affine3d& target_x, const double& duration)
    {
      if(is_goal_pose_changed_)
      {
        is_goal_pose_changed_ = false;
        control_start_time_ = current_time_;
        x_init_ = x_;
        xdot_init_ = xdot_;
      }

      x_desired_.translation() = DyrosMath::cubicVector<3>(current_time_,
                                                           control_start_time_,
                                                           control_start_time_ + duration,
                                                           x_init_.translation(),
                                                           target_x.translation(),
                                                           xdot_init_.head(3),
                                                           Vector3d::Zero());
      x_desired_.linear() = DyrosMath::rotationCubic(current_time_,
                                                     control_start_time_,
                                                     control_start_time_ + duration,
                                                     x_init_.rotation(),
                                                     target_x.rotation());
      xdot_desired_.head(3) = DyrosMath::cubicDotVector<3>(current_time_,
                                                           control_start_time_,
                                                           control_start_time_ + duration,
                                                           x_init_.translation(),
                                                           target_x.translation(),
                                                           xdot_init_.head(3),
                                                           Vector3d::Zero());
      xdot_desired_.tail(3) = DyrosMath::rotationCubicDot(current_time_,
                                                          control_start_time_,
                                                          control_start_time_ + duration,
                                                          Vector3d::Zero(),
                                                          Vector3d::Zero(),
                                                          x_init_.rotation(),
                                                          target_x.rotation());
      Vector6d x_error, xdot_error;
      x_error.head(3) = x_desired_.translation() - x_.translation();
      x_error.tail(3) = DyrosMath::getPhi(x_desired_.rotation(), x_.rotation());
      xdot_error = xdot_desired_ - xdot_;
      
      MatrixXd J = robot_->getJacobian();
      // MatrixXd J_pinv = J.transpose() * (J*J.transpose()).inverse();
      MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();

      Vector6d kp, kv;
      kp << 50, 50, 50, 100, 100, 100;
      kv << 5, 5, 5, 10, 10, 10;

      qdot_desired_ = J_pinv * (kp.asDiagonal() * x_error + kv.asDiagonal() * xdot_error);
      return q_ + qdot_desired_ * dt_;
    }




    /* Functions for keyboard teleop */

    double Controller::getBlendingCoeff(double val, double limit1, double limit2)
    {
        if (limit2 == limit1)
            return 1.0;
        double alpha = (val - limit1) / (limit2 - limit1);
        if (alpha < 0.0) alpha = 0.0;
        if (alpha > 1.0) alpha = 1.0;
        return alpha;
    }

    void Controller::saturatePosition(const VectorXd &q)
    {
        pos_limit_flag.setZero();
        limited_joints.setZero();

        for (int i = 0; i < q.size(); ++i) {
            double curr_q = q[i];
            if (curr_q > soft_min_angles[i] && curr_q < soft_max_angles[i]) {
                pos_limit_flag[i] = SAFE;
            } 
            else if (curr_q < hard_min_angles[i]) {
                pos_limit_flag[i] = MIN_HARD;
                limited_joints[i] = 1;
            } 
            else if (curr_q < soft_min_angles[i]) {
                pos_limit_flag[i] = MIN_SOFT;
                limited_joints[i] = 1;
            } 
            else if (curr_q > hard_max_angles[i]) {
                pos_limit_flag[i] = MAX_HARD;
                limited_joints[i] = 1;
            } 
            else if (curr_q > soft_max_angles[i]) {
                pos_limit_flag[i] = MAX_SOFT;
                limited_joints[i] = 1;
            }
        }
    }

    void Controller::saturateVelocity(const VectorXd &dq)
    {
        for (int i = 0; i < dq.size(); ++i) {
            if (pos_limit_flag[i] == SAFE) {
                if (dq[i] > soft_min_joint_velocity_limits[i] && dq[i] < soft_max_joint_velocity_limits[i]) {
                    vel_limit_flag[i] = V_SAFE;
                } else if (dq[i] > hard_max_joint_velocity_limits[i]) {
                    vel_limit_flag[i] = MAX_HARD_VEL;
                    limited_joints[i] = 1;
                } else if (dq[i] > soft_max_joint_velocity_limits[i]) {
                    vel_limit_flag[i] = MAX_SOFT_VEL;
                    limited_joints[i] = 1;
                } else if (dq[i] < hard_min_joint_velocity_limits[i]) {
                    vel_limit_flag[i] = MIN_HARD_VEL;
                    limited_joints[i] = 1;
                } else if (dq[i] < soft_min_joint_velocity_limits[i]) {
                    vel_limit_flag[i] = MIN_SOFT_VEL;
                    limited_joints[i] = 1;
                }
            } else {
                vel_limit_flag[i] = V_SAFE;
            }
        }
    }

    VectorXd Controller::reviseTorque(const VectorXd &tau,
                                                const VectorXd &q,
                                                const VectorXd &dq)
    {
        saturatePosition(q);
        saturateVelocity(dq);

        VectorXd tau_limited = tau;

        for (int i = 0; i < q.size(); ++i) {
            // Position-related modifications.
            if (pos_limit_flag[i] == MIN_SOFT) {
                double alpha = getBlendingCoeff(q[i], soft_min_angles[i], hard_min_angles[i]);
                tau_limited[i] = tau[i] - alpha * kv_safety[i] * dq[i];
            } else if (pos_limit_flag[i] == MAX_SOFT) {
                double alpha = getBlendingCoeff(q[i], soft_max_angles[i], hard_max_angles[i]);
                tau_limited[i] = tau[i] - alpha * kv_safety[i] * dq[i];
            } else if (pos_limit_flag[i] == MIN_HARD) {
                double alpha = getBlendingCoeff(q[i], hard_min_angles[i], q_min_[i]);
                double tau_hold = tau_limit_[i];
                tau_limited[i] = (1 - std::pow(alpha, 2)) * tau[i] + std::pow(alpha, 2) * tau_hold - kv_safety[i] * dq[i];
            } else if (pos_limit_flag[i] == MAX_HARD) {
                double alpha = getBlendingCoeff(q[i], hard_max_angles[i], q_max_[i]);
                double tau_hold = -tau_limit_[i];
                tau_limited[i] = (1 - std::pow(alpha, 2)) * tau[i] + std::pow(alpha, 2) * tau_hold - kv_safety[i] * dq[i];
            }
            // Velocity-related modifications.
            else if (vel_limit_flag[i] == MIN_SOFT_VEL) {
                double alpha = getBlendingCoeff(dq[i], soft_min_joint_velocity_limits[i], hard_min_joint_velocity_limits[i]);
                tau_limited[i] = (1 - std::pow(alpha, 1)) * tau[i];
            } else if (vel_limit_flag[i] == MAX_SOFT_VEL) {
                double alpha = getBlendingCoeff(dq[i], soft_max_joint_velocity_limits[i], hard_max_joint_velocity_limits[i]);
                tau_limited[i] = (1 - std::pow(alpha, 1)) * tau[i];
            } else if (vel_limit_flag[i] == MIN_HARD_VEL) {
                double alpha = getBlendingCoeff(dq[i], hard_min_joint_velocity_limits[i], -qdot_limit_[i]);
                tau_limited[i] = std::pow(alpha, 4) * tau_limit_[i] * 1e-2;
            } else if (vel_limit_flag[i] == MAX_HARD_VEL) {
                double alpha = getBlendingCoeff(dq[i], hard_max_joint_velocity_limits[i], qdot_limit_[i]);
                tau_limited[i] = -std::pow(alpha, 4) * tau_limit_[i] * 1e-2;
            }
        }

        return tau_limited;
    }

    bool Controller::positionViolation(const Ref<const VectorXd> &q)
    {
        bool is_violated = false;

        for(int i = 0; i < 7 ; i++){
            if(q[i] > joint_position_max[i])
            {
                is_violated = true;
                std::cout << "WARNING : Soft joint upper limit violated on joint " 
                << i << ", engaging safety mode" << std::endl;
            
            }
            if (q[i] < joint_position_min[i])
            {
                is_violated = true;
                std::cout << "WARNING : Soft joint lower limit violated on joint " 
                << i << ", engaging safety mode" << std::endl;
            }
        }   

        return is_violated;
        
    }

    bool Controller::velocityViolation(const Ref<const VectorXd> &dq)
    {
        bool is_violated = false;

        for(int i = 0; i < 7; i ++)
        {
            if (abs(dq[i]) > joint_velocity_limits[i])
            {
                is_violated = true;
                std::cout << "WARNING : Soft velocity limit violated on joint " 
                << i << ", engaging safety mode" << std::endl;
            }
        }

        return is_violated;
    }

    bool Controller::cartesianViolation(const Isometry3d &T)
    {
        bool is_violated = false;

        // cartesian checks
        Vector3d monitoring_point;
        monitoring_point = T.linear()*monitoring_point_ee_frame + T.translation();

        double radius_square = monitoring_point(0)*monitoring_point(0) + monitoring_point(1)*monitoring_point(1);
        double z_ee = monitoring_point(2);

        // lower plane
        if(z_ee < safety_plane_z_coordinate)
        {
            is_violated = true;
            std::cout << "WARNING : End effector too low, engaging safety mode" << std::endl;
            std::cout << "position of monitoring point : " << monitoring_point.transpose() << std::endl;   
            // x_d_.block<3,1>(0,3)(2) = safety_plane_z_coordinate - 0.15;    
        }
        // cylinder
        if (z_ee < safety_cylinder_height && radius_square < pow(safety_cylinder_radius, 2))
        {
            is_violated = true;
            std::cout << "WARNING : End effector too close to center of workspace, engaging safety mode" << std::endl;
            std::cout << "Monitoring point: " << monitoring_point.transpose() << std::endl;
            
            // Vector3d des_pos = x_d_.block<3,1>(0,3);
            // Vector2d xy_des(des_pos(0), des_pos(1));
            // double current_radius = xy_des.norm();
            // if (current_radius < safety_cylinder_radius)
            // {
            //     if (current_radius < safety_cylinder_radius - 0.01)
            //     {
            //         x_d_.block<3,1>(0,3)(2) = safety_cylinder_height - 0.15;
            //     }
            //     else
            //     {
            //         xy_des = xy_des.normalized() * safety_cylinder_radius;
            //         des_pos(0) = xy_des(0);
            //         des_pos(1) = xy_des(1);
            //         x_d_.block<3,1>(0,3) = des_pos;
            //     }   
            // }
        }
        
        return is_violated;
    }

    VectorXd Controller::generateSafetyTorque(bool safety_enabled, const Isometry3d &T, const Ref<const VectorXd> &q, const Ref<const VectorXd> &dq, const Ref<const VectorXd> &tau)
    {
        bool pos_safty, vel_safty;

        VectorXd tau_safe = tau;
        if (!safety_enabled) return tau_safe;

        pos_safty = positionViolation(q);
        vel_safty = velocityViolation(dq);

        if(pos_safty || vel_safty) joint_safety_mode_flag = true;

        if (safety_enabled)
        {
            task_safety_mode_flag = cartesianViolation(T);

            if(task_safety_mode_flag)
            {
                // tuning
                const double k_rep    = 300.0;   // N/m
                const double buf      = 0.05;    // m
                // const double F_MAX    = 40.0;    // N
                // const double TAU_MAX  = 10.0;    // Nm
            
                // monitoring point
                Vector3d mp = T.linear()*monitoring_point_ee_frame + T.translation();
                double dz = mp.z() - safety_plane_z_coordinate;
                double dz_cylinder = mp.z() - safety_cylinder_height;
                double r2 = mp.x()*mp.x() + mp.y()*mp.y();
                double r  = std::sqrt(r2);
            
                // only if inside the buffer do we build a small repulsion
                if (dz < buf || (r < safety_cylinder_radius + buf && dz_cylinder < buf))
                {
                    VectorXd F_rep = VectorXd::Zero(6);
                
                    // plane
                    if (dz < buf) {
                        F_rep.z() = -k_rep*(dz - buf);
                    }
                    // cylinder
                    else if(r < safety_cylinder_radius + buf && dz_cylinder < buf){
                        if (r < safety_cylinder_radius - 0.01) {
                            F_rep.z() = -k_rep*(dz_cylinder - buf);
                        }
                        else {
                            double dr = r - (safety_cylinder_radius + buf);
                            Vector2d dir(mp.x(), mp.y());
                            dir /= r;
                            F_rep.x() = -k_rep * dr * dir.x();
                            F_rep.y() = -k_rep * dr * dir.y();
                        }
                    }
                    
                
                    // clamp wrench
                    // for (int i=0; i<3; ++i) {
                    //     F_rep[i] = std::clamp(F_rep[i], -F_MAX, F_MAX);
                    //     std::cout << F_rep[i] << " ";
                    // }
                    std::cout << std::endl;
                
                    // map into joint‐torque
                    VectorXd tau_rep = robot_->getJacobian().transpose() * F_rep;
                
                    // clamp each joint
                    // for (int i=0; i<tau_rep.size(); ++i)
                    //     tau_rep[i] = std::clamp(tau_rep[i], -TAU_MAX, TAU_MAX);
                
                    tau_safe += tau_rep;
                }
            
                return tau_safe;
            }
        }
    }

    VectorXd Controller::EEteleop(const Vector6d& cmd_vel)
    {
        double alpha = 0.03045; // 10Hz cutoff frequency

        // Low-Pass Filter on Linear Velocity
        for (int i = 0; i < 3; i++){
            x_dot_lfp_(i) = alpha * cmd_vel(i) + (1.0 - alpha) * x_dot_lfp_(i);
        }
        for (int i = 3; i < 6; i++){
            x_dot_lfp_(i) = 0.0;
        }

        // Update Desired Pose
        x_desired_.translation() += x_dot_lfp_.head<3>() * dt_;

        theta_z_ += cmd_vel(5) * dt_;
        theta_y_ += cmd_vel(4) * dt_;
        theta_x_ += cmd_vel(3) * dt_;

        Eigen::Matrix3d R_d = DyrosMath::rotateWithZ(theta_z_) * DyrosMath::rotateWithY(theta_y_) * DyrosMath::rotateWithX(theta_x_);

        x_desired_.linear() = R_d * x_init_.linear();

        // Eigen::VectorXd desired_x(6);
        // desired_x.head<3>() = x_desired_.translation();
        // desired_x.tail<3>() = DyrosMath::rot2Euler(x_desired_.linear());

        // std::cout<<"desired_x : "<< desired_x.transpose() <<std::endl;
        // std::cout<<"xdot : "<< xdot_.transpose() <<std::endl;

        VectorXd tau_desired = VectorXd::Zero(6);
        if(joint_safety_mode_flag) tau_desired = PDControl(q_, qdot_desired_, Vector7d::Constant(400), Vector7d::Constant(40));
        else {
          q_desired_ = CLIK(x_desired_, 2.0);
          tau_desired = PDControl(q_desired_, qdot_desired_, Vector7d::Constant(400),Vector7d::Constant(40));
        }

        VectorXi limited_joints(7);
        limited_joints.setZero();
        int n_limited = 0;
        for (int i = 0; i < 7; ++i) {
            if (q_(i) < q_min_(i) || q_(i) > q_max_(i) - 0.1) {
                limited_joints(i) = 1;
                n_limited++;
            }
        }

        if (n_limited > 0) {
            MatrixXd Js = MatrixXd::Zero(n_limited, 7);
            int row = 0;
            for (int i = 0; i < 7; ++i) {
                if (limited_joints(i) == 1) {
                    Js(row, i) = 1.0;
                    row++;
                }
            }

            MatrixXd M_inv_ = robot_->getMassMatrix().inverse();
            MatrixXd Lambda_s = (Js * M_inv_ * Js.transpose()).inverse();
            MatrixXd Jbar_s = M_inv_ * Js.transpose() * Lambda_s;
            MatrixXd N_s = MatrixXd::Identity(7, 7) - Jbar_s * Js;

            VectorXd tau_revised = reviseTorque(tau_desired, q_, qdot_);
            tau_desired = tau_revised + N_s * tau_desired;
        }
        else {
            tau_desired = tau_desired;
        }

        Isometry3d T;
        T.matrix() = x_desired_.matrix();
        tau_desired = generateSafetyTorque(true, T, q_, qdot_, tau_desired);

        return tau_desired;
    }
    
    /* register with the global registry */
    REGISTER_MJ_CONTROLLER(Controller, "FR3Controller")
}

