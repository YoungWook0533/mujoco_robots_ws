#include "husky_dual_fr3_controller/controller.h"

namespace HuskyDualFR3Controller
{
    Controller::Controller(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd)
    : ControllerInterface(node, dt, std::move(jd))
    {
        std::string urdf_path = ament_index_cpp::get_package_share_directory("dyros_robot_controller")
                                + "/robot/husky_dual_fr3.urdf";
        robot_ = std::make_unique<HuskyDualFR3Controller::RobotData>(urdf_path, true);
        rd_joint_names_ = robot_->getJointNames();

        key_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
                    "husky_dual_fr3_controller/mode_input", 10,
                    std::bind(&Controller::keyCallback, this, std::placeholders::_1));
        // target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        //                     "husky_dual_fr3_controller/target_pose", 10,
        //                     std::bind(&Controller::subtargetPoseCallback, this, std::placeholders::_1));
        base_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                        "husky_dual_fr3_controller/cmd_vel", 10,
                        std::bind(&Controller::baseVelCallback, this, std::placeholders::_1));
        r_ee_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                        "husky_dual_fr3_controller/r_ee_pose", 10);
        l_ee_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                        "husky_dual_fr3_controller/l_ee_pose", 10);

        q_virtual_.setZero();
        q_virtual_tmp_.setZero();
        q_virtual_init_.setZero();
        q_virtual_desired_.setZero();
        qdot_virtual_.setZero();
        qdot_virtual_init_.setZero();
        qdot_virtual_desired_.setZero();
        
        r_q_mani_.setZero();
        r_q_mani_init_.setZero();
        r_q_mani_desired_.setZero();
        r_qdot_mani_.setZero();
        r_qdot_mani_init_.setZero();
        r_qdot_mani_desired_.setZero();

        l_q_mani_.setZero();
        l_q_mani_init_.setZero();
        l_q_mani_desired_.setZero();
        l_qdot_mani_.setZero();
        l_qdot_mani_init_.setZero();
        l_qdot_mani_desired_.setZero();
        
        q_mobile_.setZero();
        q_mobile_init_.setZero();
        q_mobile_desired_.setZero();
        qdot_mobile_.setZero();
        qdot_mobile_init_.setZero();
        qdot_mobile_desired_.setZero();
        
        r_x_.setIdentity();
        r_x_init_.setIdentity();
        r_x_desired_.setIdentity();
        r_xdot_.setZero();
        r_xdot_init_.setZero();
        r_xdot_desired_.setZero();

        l_x_.setIdentity();
        l_x_init_.setIdentity();
        l_x_desired_.setIdentity();
        l_xdot_.setZero();
        l_xdot_init_.setZero();
        l_xdot_desired_.setZero();

        mobile_vel_desired_.setZero();
        
        r_torque_mani_desired_.setZero();
        l_torque_mani_desired_.setZero();
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

        r_q_mani_init_ = r_q_mani_;
        l_q_mani_init_ = l_q_mani_;
        r_qdot_mani_init_ = r_qdot_mani_;
        l_qdot_mani_init_ = l_qdot_mani_;
        r_qdot_mani_desired_.setZero();
        l_qdot_mani_desired_.setZero();

        q_mobile_init_ = q_mobile_;
        qdot_mobile_init_ = qdot_mobile_;
        qdot_mobile_desired_.setZero();

        r_x_init_ = r_x_;
        r_x_desired_ = r_x_init_;
        r_x_goal_ = r_x_init_;

        l_x_init_ = l_x_;
        l_x_desired_ = l_x_init_;
        l_x_goal_ = l_x_init_;

        r_xdot_init_ = r_xdot_;
        l_xdot_init_ = l_xdot_;
        r_xdot_desired_.setZero();
        l_xdot_desired_.setZero();

        r_ee_pose_pub_timer_ = node_->create_wall_timer(
                           std::chrono::milliseconds(100),
                           std::bind(&Controller::pubREEPoseCallback, this)
                           );
        l_ee_pose_pub_timer_ = node_->create_wall_timer(
                           std::chrono::milliseconds(100),
                           std::bind(&Controller::pubLEEPoseCallback, this)
                           );                   
    }

    void Controller::updateState(const VecMap& pos_dict, 
                                 const VecMap& vel_dict,
                                 const VecMap& tau_ext_dict, 
                                 const VecMap& sensors_dict, 
                                 double current_time)
    {
        current_time_ = current_time;

        q_virtual_.head(2) = sensors_dict.at("position_sensor").head(2);
        Quaterniond quat(sensors_dict.at("orientation_sensor")(0),
                      sensors_dict.at("orientation_sensor")(1),
                      sensors_dict.at("orientation_sensor")(2),
                      sensors_dict.at("orientation_sensor")(3));
        Vector3d euler_rpy = DyrosMath::rot2Euler(quat.toRotationMatrix());
        q_virtual_(2) = euler_rpy(0);
        q_virtual_tmp_ << q_virtual_(0), q_virtual_(1), cos(q_virtual_(2)), sin(q_virtual_(2));

        qdot_virtual_.head(2) = sensors_dict.at("linear_velocity_sensor").head(2);
        qdot_virtual_(2) = sensors_dict.at("angular_velocity_sensor")(2);

        for(size_t i=0; i<7; i++)
        {
            const std::string& name = "r_fr3_joint" + std::to_string(i+1);
            r_q_mani_(i) = pos_dict.at(name)(0);
            r_qdot_mani_(i) = vel_dict.at(name)(0);
        }
        for(size_t i=0; i<7; i++)
        {
            const std::string& name = "l_fr3_joint" + std::to_string(i+1);
            l_q_mani_(i) = pos_dict.at(name)(0);
            l_qdot_mani_(i) = vel_dict.at(name)(0);
        }

        qdot_mobile_(0) = vel_dict.at("front_left_wheel")(0);
        qdot_mobile_(1) = vel_dict.at("front_right_wheel")(0);

        VectorXd q_tmp(20);
        VectorXd qdot_tmp(19);
        q_tmp << q_virtual_tmp_, l_q_mani_, r_q_mani_, q_mobile_;
        qdot_tmp << qdot_virtual_, r_qdot_mani_, l_qdot_mani_, qdot_mobile_;
        if(!robot_->updateState(q_tmp, qdot_tmp))
        {
            RCLCPP_ERROR(node_->get_logger(), "[HuskyFR3RobotData] Failed to update robot state.");
        }
        r_x_.matrix() = robot_->getPose("r_fr3_link8");
        r_xdot_ = robot_->getVelocity("r_fr3_link8");
        l_x_.matrix() = robot_->getPose("l_fr3_link8");
        l_xdot_ = robot_->getVelocity("l_fr3_link8");
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
            
            r_q_mani_init_ = r_q_mani_;
            r_qdot_mani_init_ = r_qdot_mani_;
            r_qdot_mani_desired_.setZero();

            l_q_mani_init_ = l_q_mani_;
            l_qdot_mani_init_ = l_qdot_mani_;
            l_qdot_mani_desired_.setZero();
            
            q_mobile_init_ = q_mobile_;
            qdot_mobile_init_ = qdot_mobile_;
            qdot_mobile_desired_.setZero();
            
            r_x_init_ = r_x_;
            r_x_desired_ = r_x_init_;
            r_x_goal_ = r_x_init_;

            l_x_init_ = l_x_;
            l_x_desired_ = l_x_init_;
            l_x_goal_ = l_x_init_;
            
            r_xdot_init_ = r_xdot_;
            r_xdot_desired_.setZero();

            l_xdot_init_ = l_xdot_;
            l_xdot_desired_.setZero();
        }
        
        if(mode_ == "home")
        {
            Vector7d r_q_mani_target;
            Vector7d l_q_mani_target;
            r_q_mani_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
            l_q_mani_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
            
            
            r_q_mani_desired_ = DyrosMath::cubicVector<7>(current_time_,
                                                        control_start_time_,
                                                        control_start_time_ + 2.0,
                                                        r_q_mani_init_,
                                                        r_q_mani_target,
                                                        r_qdot_mani_init_,
                                                        VectorXd::Zero(7));
            r_qdot_mani_desired_ = DyrosMath::cubicDotVector<7>(current_time_,
                                                              control_start_time_,
                                                              control_start_time_ + 2.0,
                                                              r_q_mani_init_,
                                                              r_q_mani_target,
                                                              r_qdot_mani_init_,
                                                              VectorXd::Zero(7));

            l_q_mani_desired_ = DyrosMath::cubicVector<7>(current_time_,
                                                        control_start_time_,
                                                        control_start_time_ + 2.0,
                                                        l_q_mani_init_,
                                                        l_q_mani_target,
                                                        l_qdot_mani_init_,
                                                        VectorXd::Zero(7));
            l_qdot_mani_desired_ = DyrosMath::cubicDotVector<7>(current_time_,
                                                              control_start_time_,
                                                              control_start_time_ + 2.0,
                                                              l_q_mani_init_,
                                                              l_q_mani_target,
                                                              l_qdot_mani_init_,
                                                              VectorXd::Zero(7));
                                                              
            r_torque_mani_desired_ = ManiPDControl(r_q_mani_desired_, r_qdot_mani_desired_, r_q_mani_, r_qdot_mani_);
            l_torque_mani_desired_ = ManiPDControl(l_q_mani_desired_, l_qdot_mani_desired_, l_q_mani_, l_qdot_mani_);
            qdot_mobile_desired_.setZero();
        }
        else if(mode_ == "base teleop")
        {
            Vector7d r_q_mani_target;
            Vector7d l_q_mani_target;
            r_q_mani_target << 0, -1.7, 2.3, -3.0, 0, M_PI/2, M_PI/4;
            l_q_mani_target << 0, -1.7, -2.3, -3.0, 0, M_PI/2, M_PI/4;
            
            
            r_q_mani_desired_ = DyrosMath::cubicVector<7>(current_time_,
                                                        control_start_time_,
                                                        control_start_time_ + 2.0,
                                                        r_q_mani_init_,
                                                        r_q_mani_target,
                                                        r_qdot_mani_init_,
                                                        VectorXd::Zero(7));
            r_qdot_mani_desired_ = DyrosMath::cubicDotVector<7>(current_time_,
                                                              control_start_time_,
                                                              control_start_time_ + 2.0,
                                                              r_q_mani_init_,
                                                              r_q_mani_target,
                                                              r_qdot_mani_init_,
                                                              VectorXd::Zero(7));

            l_q_mani_desired_ = DyrosMath::cubicVector<7>(current_time_,
                                                        control_start_time_,
                                                        control_start_time_ + 2.0,
                                                        l_q_mani_init_,
                                                        l_q_mani_target,
                                                        l_qdot_mani_init_,
                                                        VectorXd::Zero(7));
            l_qdot_mani_desired_ = DyrosMath::cubicDotVector<7>(current_time_,
                                                              control_start_time_,
                                                              control_start_time_ + 2.0,
                                                              l_q_mani_init_,
                                                              l_q_mani_target,
                                                              l_qdot_mani_init_,
                                                              VectorXd::Zero(7));
                                                              
            r_torque_mani_desired_ = ManiPDControl(r_q_mani_desired_, r_qdot_mani_desired_, r_q_mani_, r_qdot_mani_);
            l_torque_mani_desired_ = ManiPDControl(l_q_mani_desired_, l_qdot_mani_desired_, l_q_mani_, l_qdot_mani_);
            qdot_mobile_desired_ = MobileIK(mobile_vel_desired_);
        }
        else
        {
            r_torque_mani_desired_ = (robot_->getGravityActuated()).head(7);
            l_torque_mani_desired_ = (robot_->getGravityActuated()).head(7);
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
            const std::string name = "r_fr3_joint" + std::to_string(i+1);
            ctrl_dict[name] = r_torque_mani_desired_(i);
        }
        for(size_t i=0; i<7; i++)
        {
            const std::string name = "l_fr3_joint" + std::to_string(i+1);
            ctrl_dict[name] = l_torque_mani_desired_(i);
        }

        return ctrl_dict;
    }

    void Controller::setMode(const std::string& mode)
    {
      is_mode_changed_ = true;
      mode_ = mode;
      RCLCPP_INFO(node_->get_logger(), "[HuskyDualFR3Controller] Mode changed: %s", mode.c_str());
    }

    void Controller::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(), "[HuskyDualFR3Controller] Key input received: %d", msg->data);
      if(msg->data == 1)      setMode("home");
      else if(msg->data == 2)      setMode("base teleop");
      else                    setMode("none");
  
    }

    // void Controller::subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    // {
    //   RCLCPP_INFO(node_->get_logger(),
    //           "[HuskyDualFR3Controller] Target pose received: position=(%.3f, %.3f, %.3f), "
    //           "orientation=(%.3f, %.3f, %.3f, %.3f)",
    //           msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
    //           msg->pose.orientation.x, msg->pose.orientation.y,
    //           msg->pose.orientation.z, msg->pose.orientation.w);
    //   is_goal_pose_changed_ = true;

    //   // Convert to 4x4 homogeneous transform
    //   Eigen::Quaterniond quat(
    //     msg->pose.orientation.w,
    //     msg->pose.orientation.x,
    //     msg->pose.orientation.y,
    //     msg->pose.orientation.z);
    //   Eigen::Matrix3d R = quat.toRotationMatrix();

    //   x_goal_.linear() = R;
    //   x_goal_.translation() << msg->pose.position.x,
    //                            msg->pose.position.y,
    //                            msg->pose.position.z;
    // }

    void Controller::baseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        mobile_vel_desired_(0) = msg->linear.x;
        mobile_vel_desired_(1) = msg->angular.z;
    }

    void Controller::pubREEPoseCallback()
    {
      Eigen::Matrix4d T = r_x_.matrix();

      auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
      ee_pose_msg.header.frame_id = "base_footprint";    // This should be changed properly
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
      
      r_ee_pose_pub_->publish(ee_pose_msg);
    }

    void Controller::pubLEEPoseCallback()
    {
      Eigen::Matrix4d T = l_x_.matrix();

      auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
      ee_pose_msg.header.frame_id = "base_footprint";    // This should be changed properly
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
      
      l_ee_pose_pub_->publish(ee_pose_msg);
    }

    VectorXd Controller::ManiPDControl(const VectorXd& q_mani_desired, const VectorXd& qdot_mani_desired, const Vector7d& q_mani_curr, const Vector7d& qdot_mani_curr)
    {
        Vector7d Kp, Kv;
        Kp.setConstant(400);
        Kv.setConstant(40);
        Vector7d f = Kp.asDiagonal() * (q_mani_desired - q_mani_curr) + Kv.asDiagonal() * (qdot_mani_desired - qdot_mani_curr);
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
    REGISTER_MJ_CONTROLLER(Controller, "HuskyDualFR3Controller")
}