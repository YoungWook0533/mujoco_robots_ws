#include "husky_fr3_controller/controller.h"

namespace HuskyFR3Controller
{
    Controller::Controller(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd)
    : ControllerInterface(node, dt, std::move(jd))
    {
        std::string urdf_path = ament_index_cpp::get_package_share_directory("dyros_robot_controller")
                                + "/robot/husky_fr3.urdf";
        robot_ = std::make_unique<RobotData>(urdf_path, true);
        rd_joint_names_ = robot_->getJointNames();

        key_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
                    "husky_fr3_controller/mode_input", 10,
                    std::bind(&Controller::keyCallback, this, std::placeholders::_1));
        target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                            "husky_fr3_controller/target_pose", 10,
                            std::bind(&Controller::subtargetPoseCallback, this, std::placeholders::_1));
        base_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                            "husky_fr3_controller/cmd_vel", 10,
                            std::bind(&Controller::baseVelCallback, this, std::placeholders::_1));
        ee_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                        "husky_fr3_controller/ee_pose", 10);

        q_virtual_.setZero();
        q_virtual_tmp_.setZero();
        q_virtual_init_.setZero();
        q_virtual_desired_.setZero();
        qdot_virtual_.setZero();
        qdot_virtual_init_.setZero();
        qdot_virtual_desired_.setZero();
        
        q_mani_.setZero();
        q_mani_init_.setZero();
        q_mani_desired_.setZero();
        qdot_mani_.setZero();
        qdot_mani_init_.setZero();
        qdot_mani_desired_.setZero();
        
        q_mobile_.setZero();
        q_mobile_init_.setZero();
        q_mobile_desired_.setZero();
        qdot_mobile_.setZero();
        qdot_mobile_init_.setZero();
        
        x_.setIdentity();
        x_init_.setIdentity();
        x_desired_.setIdentity();
        xdot_.setZero();
        xdot_init_.setZero();
        xdot_desired_.setZero();

        mobile_vel_desired_.setZero();
        
        torque_mani_desired_.setZero();
        qdot_mobile_desired_.setZero();
    }

    Controller::~Controller()
    {

    }

    void Controller::starting()
    {
        is_mode_changed_ = false;
        mode_ = "home";
        control_start_time_ = current_time_;

        q_virtual_init_ = q_virtual_;
        qdot_virtual_init_ = qdot_virtual_;
        q_virtual_desired_.setZero();

        q_mani_init_ = q_mani_;
        qdot_mani_init_ = qdot_mani_;
        qdot_mani_desired_.setZero();

        q_mobile_init_ = q_mobile_;
        qdot_mobile_init_ = qdot_mobile_;
        qdot_mobile_desired_.setZero();

        x_init_ = x_;
        x_desired_ = x_init_;
        x_goal_ = x_init_;

        xdot_init_ = xdot_;
        xdot_desired_.setZero();

        ee_pose_pub_timer_ = node_->create_wall_timer(
                           std::chrono::milliseconds(100),
                           std::bind(&Controller::pubEEPoseCallback, this)
                           );
    }

    void Controller::updateState(const VecMap& pos_dict, 
                                 const VecMap& vel_dict,
                                 const VecMap& tau_ext_dict, 
                                 const VecMap& sensors_dict, 
                                 double current_time)
    {
        current_time_ = current_time;

        // Add floating base joint values to q_virtual_
        q_virtual_(0) = pos_dict.at("x_base_joint")(0);
        q_virtual_(1) = pos_dict.at("y_base_joint")(0);
        q_virtual_(2) = pos_dict.at("pivot_joint")(0);

        q_virtual_tmp_ << q_virtual_(0), q_virtual_(1), cos(q_virtual_(2)), sin(q_virtual_(2));

        qdot_virtual_(0) = vel_dict.at("x_base_joint")(0);
        qdot_virtual_(1) = vel_dict.at("y_base_joint")(0);
        qdot_virtual_(2) = vel_dict.at("pivot_joint")(0);

        for(size_t i=0; i<7; i++)
        {
            const std::string& name = "fr3_joint" + std::to_string(i+1);
            q_mani_(i) = pos_dict.at(name)(0);
            qdot_mani_(i) = vel_dict.at(name)(0);
        }

        qdot_mobile_(0) = vel_dict.at("front_left_wheel")(0);
        qdot_mobile_(1) = vel_dict.at("front_right_wheel")(0);

        Vector13d q_tmp;
        Vector12d qdot_tmp;
        q_tmp << q_virtual_tmp_, q_mani_, q_mobile_;
        qdot_tmp << qdot_virtual_, qdot_mani_, qdot_mobile_;
        if(!robot_->updateState(q_tmp, qdot_tmp))
        {
            RCLCPP_ERROR(node_->get_logger(), "[HuskyFR3RobotData] Failed to update robot state.");
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
            
            q_virtual_init_ = q_virtual_;
            qdot_virtual_init_ = qdot_virtual_;
            qdot_virtual_desired_.setZero();
            
            q_mani_init_ = q_mani_;
            qdot_mani_init_ = qdot_mani_;
            qdot_mani_desired_.setZero();
            
            q_mobile_init_ = q_mobile_;
            qdot_mobile_init_ = qdot_mobile_;
            qdot_mobile_desired_.setZero();
            
            x_init_ = x_;
            x_desired_ = x_init_;
            x_goal_ = x_init_;
            
            xdot_init_ = xdot_;
            xdot_desired_.setZero();
        }
        
        if(mode_ == "home")
        {
            Vector7d q_mani_target;
            q_mani_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
            
            
            q_mani_desired_ = DyrosMath::cubicVector<7>(current_time_,
                                                        control_start_time_,
                                                        control_start_time_ + 2.0,
                                                        q_mani_init_,
                                                        q_mani_target,
                                                        qdot_mani_init_,
                                                        VectorXd::Zero(7));
            qdot_mani_desired_ = DyrosMath::cubicDotVector<7>(current_time_,
                                                              control_start_time_,
                                                              control_start_time_ + 2.0,
                                                              q_mani_init_,
                                                              q_mani_target,
                                                              qdot_mani_init_,
                                                              VectorXd::Zero(7));
                                                              
            torque_mani_desired_ = ManiPDControl(q_mani_desired_, qdot_mani_desired_);
            qdot_mobile_desired_.setZero();
        }
        else if(mode_ == "base_teleop")
        {
            Vector7d q_mani_target;
            q_mani_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
            
            
            q_mani_desired_ = DyrosMath::cubicVector<7>(current_time_,
                                                        control_start_time_,
                                                        control_start_time_ + 2.0,
                                                        q_mani_init_,
                                                        q_mani_target,
                                                        qdot_mani_init_,
                                                        VectorXd::Zero(7));
            qdot_mani_desired_ = DyrosMath::cubicDotVector<7>(current_time_,
                                                              control_start_time_,
                                                              control_start_time_ + 2.0,
                                                              q_mani_init_,
                                                              q_mani_target,
                                                              qdot_mani_init_,
                                                              VectorXd::Zero(7));
                                                              
            torque_mani_desired_ = ManiPDControl(q_mani_desired_, qdot_mani_desired_);
            qdot_mobile_desired_ = MobileIK(mobile_vel_desired_);
        }
        else if(mode_ == "wholebody_grav_comp")
        {
            auto tmp_torque_desired = robot_->getGravityActuated();
            torque_mani_desired_ = tmp_torque_desired.head(7); 
            qdot_mobile_desired_ = MobileAdmControl(tmp_torque_desired.tail(2));
        }
        else if(mode_ == "wholebody_impedence")
        {
            Affine3d target_x = x_goal_;
            // Vector6d x_error;
            // x_error.head(3) = target_x.translation() - x_.translation();
            // x_error.tail(3) = DyrosMath::getPhi(target_x.rotation(), x_.rotation());
            
            // Vector6d Kp, Kv;
            // Kp << 1,1,1,1,1,1;
            // Kv << 1,1,1,1,1,1;

            // Vector6d f = Kp.asDiagonal() * x_error - Kv.asDiagonal() * xdot_;
            // MatrixXd J_tilda = robot_->getJacobianActuated();
            // VectorXd etadot = (J_tilda.transpose() * J_tilda).inverse() * J_tilda.transpose() * (f);
            // VectorXd tmp_torque_desired = robot_->getMassMatrixActuated() * etadot + robot_->getGravityActuated();

            // torque_mani_desired_ = tmp_torque_desired.head(7);
            // // qdot_mobile_desired_ += etadot.tail(2) * dt_;
            // qdot_mobile_desired_ = MobileAdmControl(tmp_torque_desired.tail(2));

            if(is_goal_pose_changed_)
            {
                is_goal_pose_changed_ = false;
                control_start_time_ = current_time_;
                x_init_ = x_;
                xdot_init_ = xdot_;
            }
            x_desired_.translation() = DyrosMath::cubicVector<3>(current_time_,
                                                                 control_start_time_,
                                                                 control_start_time_ + 4,
                                                                 x_init_.translation(),
                                                                 target_x.translation(),
                                                                 xdot_init_.head(3),
                                                                 Vector3d::Zero());
            x_desired_.linear() = DyrosMath::rotationCubic(current_time_,
                                                           control_start_time_,
                                                           control_start_time_ + 4,
                                                           x_init_.rotation(),
                                                           target_x.rotation());
            xdot_desired_.head(3) = DyrosMath::cubicDotVector<3>(current_time_,
                                                                 control_start_time_,
                                                                 control_start_time_ + 4,
                                                                 x_init_.translation(),
                                                                 target_x.translation(),
                                                                 xdot_init_.head(3),
                                                                 Vector3d::Zero());
            xdot_desired_.tail(3) = DyrosMath::rotationCubicDot(current_time_,
                                                                 control_start_time_,
                                                                 control_start_time_ + 4,
                                                                 Vector3d::Zero(),
                                                                 Vector3d::Zero(),
                                                                 x_init_.rotation(),
                                                                 target_x.rotation());
            Vector6d x_error, xdot_error;
            x_error.head(3) = x_desired_.translation() - x_.translation();
            x_error.tail(3) = DyrosMath::getPhi(x_desired_.rotation(), x_.rotation());
            xdot_error = xdot_desired_ - xdot_;

            Vector6d Kp, Kv;
            Kp << 10,10,10,1,1,1;
            Kv << 1,1,1,1,1,1;

            Vector6d f = Kp.asDiagonal() * x_error - Kv.asDiagonal() * xdot_;
            MatrixXd J_tilda = robot_->getJacobianActuated();
            VectorXd etadot = (J_tilda.transpose() * J_tilda).inverse() * J_tilda.transpose() * (f);
            VectorXd tmp_torque_desired = robot_->getMassMatrixActuated() * etadot + robot_->getGravityActuated();

            torque_mani_desired_ = tmp_torque_desired.head(7);
            // qdot_mobile_desired_ += etadot.tail(2) * dt_;
            qdot_mobile_desired_ = MobileAdmControl(tmp_torque_desired.tail(2));
        }
        else
        {
            torque_mani_desired_ = (robot_->getGravityActuated()).head(7);
            qdot_mobile_desired_.setZero();
        }
    }

    CtrlInputMap Controller::getCtrlInput() const
    {
        CtrlInputMap ctrl_dict;
        ctrl_dict["left_wheel"] = qdot_mobile_desired_(0);
        ctrl_dict["right_wheel"] = qdot_mobile_desired_(1);
        for(size_t i=0; i<7; i++)
        {
            const std::string name = "fr3_joint" + std::to_string(i+1);
            ctrl_dict[name] = torque_mani_desired_(i);
        }

        return ctrl_dict;
    }

    void Controller::setMode(const std::string& mode)
    {
      is_mode_changed_ = true;
      mode_ = mode;
      RCLCPP_INFO(node_->get_logger(), "[HuskyFR3Controller] Mode changed: %s", mode.c_str());
    }

    void Controller::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(), "[HuskyFR3Controller] Key input received: %d", msg->data);
      if(msg->data == 1)      setMode("home");
      else if(msg->data == 2) setMode("base_teleop");
      else if(msg->data == 3) setMode("wholebody_grav_comp");
      else if(msg->data == 4) setMode("wholebody_impedence");
      else                    setMode("none");
  
    }

    void Controller::subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(),
              "[HuskyFR3Controller] Target pose received: position=(%.3f, %.3f, %.3f), "
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

    void Controller::baseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        mobile_vel_desired_(0) = msg->linear.x;
        mobile_vel_desired_(1) = msg->angular.z;
    }

    void Controller::pubEEPoseCallback()
    {
      Eigen::Matrix4d T = x_.matrix();

      auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
      ee_pose_msg.header.frame_id = "world";
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

    VectorXd Controller::ManiPDControl(const VectorXd& q_mani_desired, const VectorXd& qdot_mani_desired)
    {
        Vector7d Kp, Kv;
        Kp.setConstant(400);
        Kv.setConstant(40);
        Vector7d f = Kp.asDiagonal() * (q_mani_desired - q_mani_) + Kv.asDiagonal() * (qdot_mani_desired - qdot_mani_);
        return (robot_->getMassMatrixActuated()).block(0,0,7,7) * f + (robot_->getGravityActuated()).segment(0,7);
    }

    Vector2d Controller::MobileIK(const Vector2d& mobile_vel_desired)
    {
        double L = 0.2854*2*1.875;
        double R = 0.1651;
        Vector2d qdot_mobile_desired;
        Vector2d base_vel_lim;
        base_vel_lim << 1.0, 3.0;

        Vector2d target_vel_clamped;
        target_vel_clamped(0) = std::clamp(mobile_vel_desired(0), -base_vel_lim(0), base_vel_lim(0));
        target_vel_clamped(1) = std::clamp(mobile_vel_desired(1), -base_vel_lim(1), base_vel_lim(1));

        qdot_mobile_desired(0) = (target_vel_clamped(0) - L/2 * target_vel_clamped(1)) / R;
        qdot_mobile_desired(1) = (target_vel_clamped(0) + L/2 * target_vel_clamped(1)) / R;
        
        return qdot_mobile_desired;
    }

    VectorXd Controller::MobileAdmControl(const VectorXd& torque_mobile_desired)
    {
        double J = 0.02;   // wheel inertia [kg·m²]
        double B = 0.1;    // wheel damping [N·m·s/rad]

        VectorXd alpha = (torque_mobile_desired - B * qdot_mobile_) / J;
        VectorXd omega = qdot_mobile_ + alpha * dt_;

        return omega;
    }

    /* register with the global registry */
    REGISTER_MJ_CONTROLLER(Controller, "HuskyFR3Controller")
}