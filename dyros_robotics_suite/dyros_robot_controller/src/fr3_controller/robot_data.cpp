#include "fr3_controller/robot_data.h"


namespace FR3Controller
{
    RobotData::RobotData(const std::string& urdf_path, const bool verbose)
    {
        std::ifstream file(urdf_path);
        if (!file.good()) std::cout << "\033[1;31m" << "URDF file does not exist! : " << urdf_path << "\033[0m" << "\033[0m" << std::endl;
    
        pinocchio::urdf::buildModel(urdf_path, model_);
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

        // Initialize task space state
        x_ .setIdentity();
        xdot_.setZero();
        J_.setZero(6, model_.nv);
        Jdot_.setZero(6, model_.nv);

        // Initialize joint space dynamics
        M_.setZero(model_.nv, model_.nv);
        M_inv_.setZero(model_.nv, model_.nv);
        g_.setZero(model_.nv);       
        c_.setZero(model_.nv);       
        NLE_.setZero(model_.nv);       
    }

    RobotData::~RobotData()
    {
    }

    bool RobotData::updateState(const VectorXd& q, const VectorXd& qdot)
    {
        q_ = q;
        qdot_ = qdot;
        if(!updateKinematics(q_, qdot_)) return false;
        if(!updateDynamics(q_, qdot_)) return false;
        return true;
    }

    bool RobotData::updateKinematics(const VectorXd& q, const VectorXd& qdot)
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
        
        pinocchio::FrameIndex link_index = model_.getFrameId(ee_name_);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << ee_name_ << " not found in URDF." << "\033[0m" << std::endl;
            return false;
        }

        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, qdot);
        x_ = getPose(ee_name_);
        J_ = getJacobian(ee_name_);
        xdot_ = J_ * qdot;
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

        MatrixXd J = computeJacobian(q, link_name);
        
        return J * qdot;
    }

    Vector6d RobotData::getVelocity(const std::string& link_name)
    {
        MatrixXd J = getJacobian(link_name);

        return J * qdot_;
    }

    MatrixXd RobotData::computeMassMatrix(const VectorXd& q)
    {
        assert(q.size() == model_.nq);
        
        pinocchio::Data tmp_data;
        pinocchio::crba(model_, tmp_data, q);
        tmp_data.M = tmp_data.M.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba

        return tmp_data.M;
    }

    VectorXd RobotData::computeCoriolis(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);
        
        pinocchio::Data tmp_data;
        pinocchio::computeCoriolisMatrix(model_, tmp_data, q, qdot);

        return tmp_data.C * qdot;
    }

    VectorXd RobotData::computeGravity(const VectorXd& q)
    {
        assert(q.size() == model_.nq);
        
        pinocchio::Data tmp_data;
        pinocchio::computeGeneralizedGravity(model_, tmp_data, q);

        return tmp_data.g;
    }

    VectorXd RobotData::computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        pinocchio::Data tmp_data;
        pinocchio::nonLinearEffects(model_, tmp_data, q, qdot);

        return tmp_data.nle;
    }
}