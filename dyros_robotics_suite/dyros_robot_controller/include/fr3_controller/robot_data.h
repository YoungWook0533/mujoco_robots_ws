#ifndef FR3_ROBOT_DATA_HPP
#define FR3_ROBOT_DATA_HPP

#include <string>

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

namespace FR3Controller
{
    typedef Eigen::Matrix<double,6,6> Matrix6d;
    typedef Eigen::Matrix<double,6,1> Vector6d;

    class RobotData
    {
        public:

            RobotData(const std::string& urdf_path, const bool verbose=false);
            ~RobotData();

            bool updateState(const VectorXd& q, const VectorXd& qdot);
            std::vector<std::string> getJointNames(){return joint_names_;}

            Matrix4d computePose(const VectorXd& q, const std::string& link_name=ee_name_);
            MatrixXd computeJacobian(const VectorXd& q, const std::string& link_name=ee_name_);
            MatrixXd computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
            Vector6d computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
            MatrixXd computeMassMatrix(const VectorXd& q);
            VectorXd computeCoriolis(const VectorXd& q, const VectorXd& qdot);
            VectorXd computeGravity(const VectorXd& q);
            VectorXd computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot);

            VectorXd getJointPosition(){return q_;}
            VectorXd getJointVelocity(){return qdot_;}
            Matrix4d getPose(const std::string& link_name=ee_name_);
            MatrixXd getJacobian(const std::string& link_name=ee_name_);
            MatrixXd getJacobianTimeVariation(const std::string& link_name=ee_name_);
            Vector6d getVelocity(const std::string& link_name=ee_name_);
            MatrixXd getMassMatrix(){return M_;}
            VectorXd getCoriolis(){return c_;}
            VectorXd getGravity(){return g_;}
            VectorXd getNonlinearEffects(){return NLE_;}

            

        private:
            bool updateKinematics(const VectorXd& q, const VectorXd& qdot);
            bool updateDynamics(const VectorXd& q, const VectorXd& qdot);

            pinocchio::Model model_;
            pinocchio::Data data_;

            std::vector<std::string> joint_names_;

            // Joint space state
            VectorXd q_;       // joint angle
            VectorXd qdot_;    // joint velocity

            // Task space state
            static constexpr const char* ee_name_ = "fr3_link8"; // end-effector link name
            Matrix4d x_;     // pose of EE
            Vector6d xdot_;  // velocity of EE
            MatrixXd J_;     // jacobian of EE
            MatrixXd Jdot_;  // time derivative of jacobian of EE

            // Joint space Dynamics
            MatrixXd M_;     // inertia matrix
            MatrixXd M_inv_; // inverse of inertia matrix
            VectorXd g_;     // gravity forces
            VectorXd c_;     // centrifugal and coriolis forces
            VectorXd NLE_;   // nonlinear effects ( g_ + c_ )

            // Task space Dynamics
            Matrix6d M_ee_;     // inertia matrix
            Matrix6d M_ee_inv_; // inverse of inertia matrix
            Vector6d g_ee_;     // gravity forces
            Vector6d c_ee_;     // centrifugal and coriolis forces
            Vector6d NLE_ee_;   // nonlinear effects ( g_ + c_ )
    };
}

#endif // FR3_ROBOT_DATA_HPP