#pragma once
#include <string>
#include <mutex>
#include <shared_mutex>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <Eigen/Dense>
using namespace Eigen;

namespace HuskyFR3Controller
{
    typedef Eigen::Matrix<double,6,6> Matrix6d;
    typedef Eigen::Matrix<double,6,1> Vector6d;

    /*
    Joint Information
    Total nq = 13
    Total nv = 12
    
     id | name              | nq | nv | idx_q | idx_v
    ----+-------------------+----+----+-------+------
      1 |        root_joint |  4 |  3 |     0 |    0
      2 |        fr3_joint1 |  1 |  1 |     4 |    3
      3 |        fr3_joint2 |  1 |  1 |     5 |    4
      4 |        fr3_joint3 |  1 |  1 |     6 |    5
      5 |        fr3_joint4 |  1 |  1 |     7 |    6
      6 |        fr3_joint5 |  1 |  1 |     8 |    7
      7 |        fr3_joint6 |  1 |  1 |     9 |    8
      8 |        fr3_joint7 |  1 |  1 |    10 |    9
      9 |        left_wheel |  1 |  1 |    11 |   10
     10 |       right_wheel |  1 |  1 |    12 |   11
    */

    class RobotData
    {
        public:
            RobotData(const std::string& urdf_path, const bool verbose=false);
            ~RobotData();
            bool updateState(const VectorXd& q, const VectorXd& qdot);
            std::vector<std::string> getJointNames(){return joint_names_;}

            Matrix4d computePose(const VectorXd& q, const std::string& link_name=ee_name_);
            MatrixXd computeJacobian(const VectorXd& q, const std::string& link_name=ee_name_);
            MatrixXd computeJacobianActuated(const VectorXd& q, const std::string& link_name=ee_name_);
            MatrixXd computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
            Vector6d computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
            MatrixXd computeMassMatrix(const VectorXd& q);
            MatrixXd computeMassMatrixActuated(const VectorXd& q);
            VectorXd computeCoriolis(const VectorXd& q, const VectorXd& qdot);
            VectorXd computeCoriolisActuated(const VectorXd& q, const VectorXd& qdot);
            VectorXd computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot);
            VectorXd computeNonlinearEffectsActuated(const VectorXd& q, const VectorXd& qdot);
            VectorXd computeGravity(const VectorXd& q);
            VectorXd computeGravityActuated(const VectorXd& q);

            VectorXd getJointPosition(){return q_;}
            VectorXd getJointVelocity(){return qdot_;}
            VectorXd getJointPositionActuated(){return q_actuated_;}
            VectorXd getJointVelocityActuated(){return qdot_actuated_;}
            Matrix4d getPose(const std::string& link_name=ee_name_);
            MatrixXd getJacobian(const std::string& link_name=ee_name_);
            MatrixXd getJacobianActuated(const std::string& link_name=ee_name_);
            MatrixXd getJacobianTimeVariation(const std::string& link_name=ee_name_);
            Vector6d getVelocity(const std::string& link_name=ee_name_);
            MatrixXd getMassMatrix(){return M_;}
            MatrixXd getMassMatrixActuated(){return M_actuated_;}
            VectorXd getCoriolis(){return c_;}
            VectorXd getCoriolisActuated(){return c_actuated_;}
            VectorXd getGravity(){return g_;}
            VectorXd getGravityActuated(){return g_actuated_;}
            VectorXd getNonlinearEffects(){return NLE_;}
            VectorXd getNonlinearEffectsActuated(){return NLE_actuated_;}

        private:
            bool updateKinematics(const VectorXd& q, const VectorXd& qdot);
            bool updateDynamics(const VectorXd& q, const VectorXd& qdot);

            // pinocchio data
            pinocchio::Model model_;
            pinocchio::Data data_;

            // Selection Matrix for virtual joints
            MatrixXd S_;
            const double wheel_radius_ = 0.1651;
            const double mobile_width_ = 0.2775*2.;
            const double mobile2mani_x = 0.2;
            const double mobile2mani_y = 0.;
            MatrixXd J_tmp_; // (left_wheel_vel, right_wheel_vel) -> (b_xdot_mani, b_ydot_mani, b_yawdot_mani) 

            std::vector<std::string> joint_names_;
            std::vector<std::string> joint_names_actuated_;

            // Joint space state
            VectorXd q_;        // joint angle
            VectorXd qdot_;     // joint velocity

            VectorXd q_actuated_;        // joint angle
            VectorXd qdot_actuated_;     // joint velocity

            // Task space state
            static constexpr const char* ee_name_ = "fr3_link8"; // end-effector link name
            Matrix4d x_;         // pose of EE
            Vector6d xdot_;      // velocity of EE
            MatrixXd J_;         // jacobian of EE
            MatrixXd Jdot_;      // time derivative of jacobian of EE
            
            MatrixXd J_actuated_;     // jacobian of EE

            // Joint space Dynamics
            MatrixXd M_;     // inertia matrix
            MatrixXd M_inv_; // inverse of inertia matrix
            VectorXd g_;     // gravity forces
            VectorXd c_;     // centrifugal and coriolis forces
            VectorXd NLE_;   // nonlinear effects ( g_ + c_ )

            MatrixXd M_actuated_;     // inertia matrix
            MatrixXd M_inv_actuated_; // inverse of inertia matrix
            VectorXd g_actuated_;     // gravity forces
            VectorXd c_actuated_;     // centrifugal and coriolis forces
            VectorXd NLE_actuated_;   // nonlinear effects ( g_ + c_ )
    };
}