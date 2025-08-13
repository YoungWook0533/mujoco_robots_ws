#pragma once
#include "mujoco_ros_sim/ControllerInterface.hpp"
#include "mujoco_ros_sim/ControllerRegistry.hpp"

#include "husky_fr3_controller/robot_data.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <thread>
#include <atomic>
#include <mutex>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "math_type_define.h"
#include "suhan_benchmark.h"

using namespace Eigen;

/*
Husky FR3 Joint/Sensor Information
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  1 | front_left_wheel     | _Hinge |  1 |  1 |     7 |    6
  2 | front_right_wheel    | _Hinge |  1 |  1 |     8 |    7
  3 | rear_left_wheel      | _Hinge |  1 |  1 |     9 |    8
  4 | rear_right_wheel     | _Hinge |  1 |  1 |    10 |    9
  5 | fr3_joint1           | _Hinge |  1 |  1 |    11 |   10
  6 | fr3_joint2           | _Hinge |  1 |  1 |    12 |   11
  7 | fr3_joint3           | _Hinge |  1 |  1 |    13 |   12
  8 | fr3_joint4           | _Hinge |  1 |  1 |    14 |   13
  9 | fr3_joint5           | _Hinge |  1 |  1 |    15 |   14
 10 | fr3_joint6           | _Hinge |  1 |  1 |    16 |   15
 11 | fr3_joint7           | _Hinge |  1 |  1 |    17 |   16

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | left_wheel           | _Joint  | front_left_wheel
  1 | right_wheel          | _Joint  | front_right_wheel
  2 | fr3_joint1           | _Joint  | fr3_joint1
  3 | fr3_joint2           | _Joint  | fr3_joint2
  4 | fr3_joint3           | _Joint  | fr3_joint3
  5 | fr3_joint4           | _Joint  | fr3_joint4
  6 | fr3_joint5           | _Joint  | fr3_joint5
  7 | fr3_joint6           | _Joint  | fr3_joint6
  8 | fr3_joint7           | _Joint  | fr3_joint7

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:husky_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:husky_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:husky_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:husky_site

*/

namespace HuskyFR3Controller
{
    typedef Eigen::Matrix<double,7,7> Matrix7d;
    typedef Eigen::Matrix<double,7,1> Vector7d;
    typedef Eigen::Matrix<double,9,9> Matrix9d;
    typedef Eigen::Matrix<double,9,1> Vector9d;
    typedef Eigen::Matrix<double,12,12> Matrix12d;
    typedef Eigen::Matrix<double,12,1> Vector12d;
    typedef Eigen::Matrix<double,13,1> Vector13d;

    class Controller : public ControllerInterface
    {
    public:
        // ====================================================================================
        // ================================== Core Functions ================================== 
        // ====================================================================================
        Controller(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd);
        ~Controller() override;
        void starting() override;
        void updateState(const VecMap&,const VecMap&, const VecMap&, const VecMap&, double) override;
        void compute() override;
        CtrlInputMap getCtrlInput() const override;

        // ====================================================================================
        // ===================== Helper / CB / Background Thread Functions ==================== 
        // ====================================================================================
        void setMode(const std::string& mode);
        void keyCallback(const std_msgs::msg::Int32::SharedPtr);
        void subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
        void baseVelCallback(const geometry_msgs::msg::Twist::SharedPtr);
        void pubEEPoseCallback();
        void computeSlowLoop();

        // ====================================================================================
        // ================================= Control Functions ================================
        // ====================================================================================
        VectorXd ManiPDControl(const VectorXd& q_mani_desired, const VectorXd& qdot_mani_desired);
        Vector2d MobileIK(const Vector2d& mobile_vel_desired);
        VectorXd MobileAdmControl(const VectorXd& torque_mobile_desired);


    private :
        std::unique_ptr<RobotData> robot_;
        std::unique_ptr<RobotData> viz_robot_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       base_vel_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    ee_pose_pub_;
        
        rclcpp::TimerBase::SharedPtr ee_pose_pub_timer_;
        
        std::vector<std::string> rd_joint_names_;

        SuhanBenchmark mobile_timer_;

        bool is_mode_changed_{false};
        bool is_goal_pose_changed_{false};
        std::string mode_{"home"};
        double control_start_time_;
        double current_time_;
        
        //// joint space state
        Vector3d q_virtual_;
        Vector4d q_virtual_tmp_;
        Vector3d q_virtual_init_;
        Vector3d q_virtual_desired_;
        Vector3d qdot_virtual_;
        Vector3d qdot_virtual_init_;
        Vector3d qdot_virtual_desired_;

        Vector7d q_mani_;
        Vector7d q_mani_init_;
        Vector7d q_mani_desired_;
        Vector7d qdot_mani_;
        Vector7d qdot_mani_init_;
        Vector7d qdot_mani_desired_;

        Vector2d q_mobile_;
        Vector2d q_mobile_init_;
        Vector2d q_mobile_desired_;
        Vector2d qdot_mobile_;
        Vector2d qdot_mobile_init_;

        //// operation space state
        Affine3d x_;
        Affine3d x_goal_;
        Affine3d x_init_;
        Affine3d x_desired_;
        Vector6d xdot_;
        Vector6d xdot_init_;
        Vector6d xdot_desired_;

        Vector2d mobile_vel_desired_;

        //// control input
        Vector7d torque_mani_desired_;
        Vector2d qdot_mobile_desired_;

        // Input smoothing to reduce trembling
        Eigen::VectorXd mppi_applied_u_; // smoothed version of last external u
        double mppi_u_smoothing_alpha_{0.2};

        // External MPPI bridge
        bool use_external_mppi_{false};
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mppi_observation_pub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mppi_input_sub_;
        std::string external_input_topic_{"/input"};
        std::mutex mppi_ext_u_mutex_;
        Eigen::VectorXd mppi_ext_last_u_; // size 9
        bool mppi_ext_last_u_valid_{false};
        double mppi_ext_last_u_time_{-1.0};
    };
} // namespace HuskyFR3Controller