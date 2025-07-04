#include "husky_fr3_controller/robot_data.h"


namespace HuskyFR3Controller
{
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
    RobotData::RobotData(const std::string& urdf_path, const bool verbose)
    {
        std::ifstream file(urdf_path);
        if (!file.good()) std::cout << "\033[1;31m" << "URDF file does not exist! : " << "\033[0m" << urdf_path << "\033[0m" << std::endl;
        
        pinocchio::JointModelPlanar planar;
        pinocchio::urdf::buildModel(urdf_path, planar, model_, false);
        data_ = pinocchio::Data(model_);
                
        joint_names_ = model_.names;
        joint_names_.erase(joint_names_.begin());  // Remove the first element "universe_joint"

        if(verbose)
        {
            std::cout << "Total nq = " << model_.nq << '\n' << "Total nv = " << model_.nv << "\n\n";
            std::cout << " id | name              | nq | nv | idx_q | idx_v\n";
            std::cout << "----+-------------------+----+----+-------+------\n";
            for(pinocchio::JointIndex id = 1; id < model_.joints.size(); ++id)
            {
                std::cout << std::setw(3)  << id << " | "
                          << std::setw(17) << model_.names[id] << " | "
                          << std::setw(2)  << model_.nqs[id]   << " | "
                          << std::setw(2)  << model_.nvs[id]   << " | "
                          << std::setw(5)  << model_.idx_qs[id]<< " | "
                          << std::setw(4)  << model_.idx_vs[id]<< '\n';
            }
        }


        // Initialize joint space state
        q_.setZero(model_.nq);
        qdot_.setZero(model_.nv);

        q_actuated_.setZero(model_.nq-4);
        qdot_actuated_.setZero(model_.nv-3);

        // Initialize task space state
        x_.setIdentity();
        xdot_.setZero();
        J_.setZero(6, model_.nv);
        Jdot_.setZero(6, model_.nv-3);
        J_actuated_.setZero(6, model_.nv-3);

        // Initialize joint space dynamics
        M_.setZero(model_.nv, model_.nv);
        M_inv_.setZero(model_.nv, model_.nv);
        g_.setZero(model_.nv);       
        c_.setZero(model_.nv);       
        NLE_.setZero(model_.nv);   
        
        M_actuated_.setZero(model_.nv-3, model_.nv-3);
        M_inv_actuated_.setZero(model_.nv-3, model_.nv-3);
        g_actuated_.setZero(model_.nv-3);       
        c_actuated_.setZero(model_.nv-3);       
        NLE_actuated_.setZero(model_.nv-3);

        // Initialize selection matrix for virtual joints
        S_.setZero(model_.nv, model_.nv-3);
        S_.block(3, 0, 7, 7).setIdentity();
        S_.block(10, 7, 2, 2).setIdentity();

        J_tmp_.setZero(3, 2);
        Matrix<double, 3,3> J1;
        Matrix<double, 3,2> J2;
        J1 << 1, 0, -mobile2mani_y,
              0, 1,  mobile2mani_x,
              0, 0,  1;
        J2 << wheel_radius_/2.,               wheel_radius_/2.,
              0.,                             0.,
              -wheel_radius_/(mobile_width_), wheel_radius_/(mobile_width_);
        J_tmp_ = J1 * J2;
    }

    RobotData::~RobotData()
    {
    }

    bool RobotData::updateState(const VectorXd& q, const VectorXd& qdot)
    {
        q_ = q;
        qdot_ = qdot;
        q_actuated_ = q_.tail(model_.nq-4);
        qdot_actuated_ = qdot_.tail(model_.nv-3);

        if(!updateKinematics(q_, qdot_)) return false;
        if(!updateDynamics(q_, qdot_)) return false;
        return true;
    }

    bool RobotData::updateKinematics(const VectorXd& q, const VectorXd& qdot)
    {        
        if(q.size() != model_.nq)
        {
            std::cerr << "\033[1;31m" << "updateDynamics Error: size of q " << q.size() << " is not equal to model.nq size: " << "\033[0m" << model_.nq << "\033[0m" << std::endl;
            return false;
        }
        if(qdot.size() != model_.nv)
        {
            std::cerr << "\033[1;31m" << "updateDynamics Error: size of qdot " << qdot.size() << " is not equal to model.nv size: " << "\033[0m" << model_.nv << "\033[0m" << std::endl;
            return false;
        }
        
        pinocchio::FrameIndex link_index = model_.getFrameId(ee_name_);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << ee_name_ << " not found in URDF." << "\033[0m" << std::endl;
            return false;
        }
        
        double mobile_yaw = std::atan2(q(3), q(2)); // q(2) = cos(θ), q(3) = sin(θ)
        Matrix3d R_wolrd2Base;
        R_wolrd2Base << cos(mobile_yaw), -sin(mobile_yaw), 0,
                        sin(mobile_yaw),  cos(mobile_yaw), 0,
                        0,                0,               1;
        S_.block(0,7,3,2) = R_wolrd2Base * J_tmp_;
        
        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, qdot);
        x_ = getPose(ee_name_);
        J_ = getJacobian(ee_name_);
        J_actuated_ = J_ * S_;
        xdot_ = J_actuated_ * qdot.tail(model_.nv-3);
        Jdot_ = getJacobianTimeVariation();

        return true;
    }

    bool RobotData::updateDynamics(const VectorXd& q, const VectorXd& qdot)
    {
        if(q.size() != model_.nq)
        {
            std::cerr << "\033[1;31m" << "updateDynamics Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << "\033[0m" << std::endl;
            return false;
        }
        if(qdot.size() != model_.nv)
        {
            std::cerr << "\033[1;31m" << "updateDynamics Error: size of qdot " << qdot.size() << " is not equal to model.nv size: " << model_.nv << "\033[0m" << std::endl;
            return false;
        }
        pinocchio::crba(model_, data_, q);
        pinocchio::computeGeneralizedGravity(model_, data_, q);
        pinocchio::nonLinearEffects(model_, data_, q, qdot);

        // update joint space dynamics
        M_ = data_.M;
        M_ = M_.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba
        M_inv_ = M_.inverse();
        g_ = data_.g;
        NLE_ = data_.nle;
        c_ = NLE_ - g_;
        
        M_actuated_ = S_.transpose() * M_ * S_;
        M_inv_actuated_ = M_actuated_.inverse();
        g_actuated_ = S_.transpose() * g_;
        NLE_actuated_ = S_.transpose() * NLE_;
        c_actuated_ = NLE_actuated_ - g_actuated_;

        return true;
    }

    Matrix4d RobotData::computePose(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == model_.nq);

        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return Matrix4d::Identity();
        }
        pinocchio::Data tmp_data;
        pinocchio::framesForwardKinematics(model_, tmp_data, q);
        Matrix4d link_pose = tmp_data.oMf[link_index].toHomogeneousMatrix();

        return link_pose;
    }

    Matrix4d RobotData::getPose(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return Matrix4d::Identity();
        }
        Matrix4d link_pose = data_.oMf[link_index].toHomogeneousMatrix();

        return link_pose;
    }

    MatrixXd RobotData::computeJacobian(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == model_.nq);

        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        
        MatrixXd J;
        J.setZero(6, model_.nv);
        pinocchio::Data tmp_data;
        pinocchio::computeFrameJacobian(model_, tmp_data, q, link_index, J);

        return J;
    }

    MatrixXd RobotData::computeJacobianActuated(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == model_.nq);

        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return MatrixXd::Zero(6, model_.nv-3);
        }
        MatrixXd S_tmp;
        S_tmp.setZero(model_.nv, model_.nv-3);
        S_.block(3, 0, 7, 7).setIdentity();
        S_.block(10, 7, 2, 2).setIdentity();
        double mobile_yaw = std::atan2(q(3), q(2)); // q(2) = cos(θ), q(3) = sin(θ)
        Matrix3d R_wolrd2Base;
        R_wolrd2Base << cos(mobile_yaw), -sin(mobile_yaw), 0,
                        sin(mobile_yaw),  cos(mobile_yaw), 0,
                        0,                0,               1;
        S_.block(0,7,3,2) = R_wolrd2Base * J_tmp_;

        MatrixXd J = computeJacobian(q, link_name);
        J = J * S_tmp;

        return J;
    }

    MatrixXd RobotData::getJacobian(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd J = pinocchio::getFrameJacobian(model_, data_, link_index, pinocchio::ReferenceFrame::WORLD);

        return J;
    }

    MatrixXd RobotData::getJacobianActuated(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd J = pinocchio::getFrameJacobian(model_, data_, link_index, pinocchio::ReferenceFrame::WORLD);

        return J*S_;
    }

    MatrixXd RobotData::computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd Jdot;
        Jdot.setZero(6, model_.nv);
        pinocchio::Data tmp_data;
        pinocchio::computeJointJacobiansTimeVariation(model_, tmp_data, q, qdot_);
        pinocchio::getFrameJacobianTimeVariation(model_, tmp_data, link_index, pinocchio::ReferenceFrame::WORLD, Jdot);

        return Jdot;
    }

    MatrixXd RobotData::getJacobianTimeVariation(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd Jdot;
        Jdot.setZero(6, model_.nv);
        pinocchio::getFrameJacobianTimeVariation(model_, data_, link_index, pinocchio::ReferenceFrame::WORLD, Jdot);

        return Jdot;
    }

    Vector6d RobotData::computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        MatrixXd J = computeJacobianActuated(q, link_name);
        
        return J * qdot.tail(model_.nv-3);
    }

    Vector6d RobotData::getVelocity(const std::string& link_name)
    {
        MatrixXd J = getJacobianActuated(link_name);

        return J * qdot_.tail(model_.nv-3);
    }

    MatrixXd RobotData::computeMassMatrix(const VectorXd& q)
    {
        assert(q.size() == model_.nq);
        
        pinocchio::Data tmp_data;
        pinocchio::crba(model_, tmp_data, q);
        tmp_data.M = tmp_data.M.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba

        return tmp_data.M;
    }

    MatrixXd RobotData::computeMassMatrixActuated(const VectorXd& q)
    {
        assert(q.size() == model_.nq);
        
        MatrixXd S_tmp;
        S_tmp.setZero(model_.nq, model_.nq-3);
        S_tmp.block(3, 0, 2, 2).setIdentity();
        S_tmp.block(5, 2, 7, 7).setIdentity();
        double mobile_yaw = std::atan2(q(3), q(2)); // q(2) = cos(θ), q(3) = sin(θ)double mobile_yaw = q(2);
        Matrix3d R_wolrd2Base;
        R_wolrd2Base << cos(mobile_yaw), -sin(mobile_yaw), 0,
                        sin(mobile_yaw),  cos(mobile_yaw), 0,
                        0,                0,               1;
        S_.block(0,7,3,2) = R_wolrd2Base * J_tmp_;


        return S_tmp.transpose() *  computeMassMatrix(q) * S_tmp;
    }

    VectorXd RobotData::computeCoriolis(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);
        
        pinocchio::Data tmp_data;
        pinocchio::computeCoriolisMatrix(model_, tmp_data, q, qdot);

        return tmp_data.C * qdot;
    }

    VectorXd RobotData::computeCoriolisActuated(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        MatrixXd S_tmp;
        S_tmp.setZero(model_.nq, model_.nq-3);
        S_tmp.block(3, 0, 2, 2).setIdentity();
        S_tmp.block(5, 2, 7, 7).setIdentity();
        double mobile_yaw = std::atan2(q(3), q(2)); // q(2) = cos(θ), q(3) = sin(θ)double mobile_yaw = q(2);
        Matrix3d R_wolrd2Base;
        R_wolrd2Base << cos(mobile_yaw), -sin(mobile_yaw), 0,
                        sin(mobile_yaw),  cos(mobile_yaw), 0,
                        0,                0,               1;
        S_.block(0,7,3,2) = R_wolrd2Base * J_tmp_;
        
        return S_tmp.transpose() * (computeNonlinearEffects(q, qdot) - computeGravity(q));
    }

    VectorXd RobotData::computeGravity(const VectorXd& q)
    {
        assert(q.size() == model_.nq);
        
        pinocchio::Data tmp_data;
        pinocchio::computeGeneralizedGravity(model_, tmp_data, q);

        return tmp_data.g;
    }

    VectorXd RobotData::computeGravityActuated(const VectorXd& q)
    {
        assert(q.size() == model_.nq);
        
        MatrixXd S_tmp;
        S_tmp.setZero(model_.nq, model_.nq-3);
        S_tmp.block(3, 0, 2, 2).setIdentity();
        S_tmp.block(5, 2, 7, 7).setIdentity();
        double mobile_yaw = std::atan2(q(3), q(2)); // q(2) = cos(θ), q(3) = sin(θ)double mobile_yaw = q(2);
        Matrix3d R_wolrd2Base;
        R_wolrd2Base << cos(mobile_yaw), -sin(mobile_yaw), 0,
                        sin(mobile_yaw),  cos(mobile_yaw), 0,
                        0,                0,               1;
        S_.block(0,7,3,2) = R_wolrd2Base * J_tmp_;

        return S_tmp.transpose() * computeGravity(q);
    }

    VectorXd RobotData::computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        pinocchio::Data tmp_data;
        pinocchio::nonLinearEffects(model_, tmp_data, q, qdot);

        return tmp_data.nle;
    }

    VectorXd RobotData::computeNonlinearEffectsActuated(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        MatrixXd S_tmp;
        S_tmp.setZero(model_.nq, model_.nq-3);
        S_tmp.block(3, 0, 2, 2).setIdentity();
        S_tmp.block(5, 2, 7, 7).setIdentity();
        double mobile_yaw = std::atan2(q(3), q(2)); // q(2) = cos(θ), q(3) = sin(θ)double mobile_yaw = q(2);
        Matrix3d R_wolrd2Base;
        R_wolrd2Base << cos(mobile_yaw), -sin(mobile_yaw), 0,
                        sin(mobile_yaw),  cos(mobile_yaw), 0,
                        0,                0,               1;
        S_.block(0,7,3,2) = R_wolrd2Base * J_tmp_;

        return S_tmp.transpose() * computeNonlinearEffects(q, qdot);
    }
}