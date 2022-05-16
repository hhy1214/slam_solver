#ifndef SOLVER_COSTIMUFUNCTION_H
#define SOLVER_COSTIMUFUNCTION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include <ros/assert.h>
#include <iostream>
#include <Eigen/Dense>
#include "integration_base.h"
#include "pose.h"
#include "motion.h"

namespace SLAM_Solver
{
    class costIMUFunction
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        POINTER_TYPEDEFS(costIMUFunction);

        //构造函数
        costIMUFunction(IntegrationBase *_pre_integration)
            :  pre_integration_(_pre_integration) 
        {
            
        }
        void set_curPose(Pose::Ptr curPose_)
        {
            m_curPose = curPose_;
        }

        void set_startPose(Pose::Ptr startPose_)
        {
            m_startPose = startPose_;
        }
        void set_curMotion(Motion::Ptr curMotion_)
        {
            m_curMotion = curMotion_;
        }
        void set_startMotion(Motion::Ptr startMotion_)
        {
            m_startMotion = startMotion_;
        }
        Pose::Ptr get_curPose()
        {
            return m_curPose;
        }

        Pose::Ptr get_startPose()
        {
            return m_startPose;
        }
        Motion::Ptr get_curMotion()
        {
            return m_curMotion;
        }
        Motion::Ptr get_startMotion()
        {
            return m_startMotion;
        }
        

        void computeResidual()
        {
            Eigen::Vector3d Pi, Vi, Bai, Bgi;
            Eigen::Vector3d Pj, Vj, Baj, Bgj;
            Eigen::Quaterniond Qj, Qi;
            Pi = get_startPose()->get_translation();
            Qi = get_startPose()->get_rotate();
            Vi = get_startMotion()->get_Vs();
            Bai = get_startMotion()->get_Ba();
            Bgi = get_startMotion()->get_Bg();



            Pj = get_curPose()->get_translation();
            Qj = get_curPose()->get_rotate();
            Vj = get_curMotion()->get_Vs();
            Baj = get_curMotion()->get_Ba();
            Bgj = get_curMotion()->get_Bg();
            residuals = pre_integration_->evaluate(Pi, Qi, Vi, Bai, Bgi,
                                               Pj, Qj, Vj, Baj, Bgj);

            // std::cout << "residuals: " << residuals << std::endl;

            // std::cout << "Pi: " << Pi << std::endl;
            // std::cout << "Qi.w(): " << Qi.w() << " Qi.x(): " << Qi.x() << " Qi.y(): " << Qi.y() << " Qi.z(): " << Qi.z() << std::endl;
            // std::cout << "Vi: " << Vi << std::endl;
            // std::cout << "Bai: " << Bai << std::endl;
            // std::cout << "Bgi: " << Bgi << std::endl;
            // std::cout << "Pj: " << Pj << std::endl;
            // std::cout << "Qj.w(): " << Qj.w() << " Qj.x(): " << Qj.x() << " Qj.y(): " << Qj.y() << " Qj.z(): " << Qj.z() << std::endl;
            // std::cout << "Vj: " << Vj << std::endl;
            // std::cout << "Baj: " << Baj << std::endl;
            // std::cout << "Bgj: " << Bgj << std::endl;

            // std::cout << "pre_integration_->jacobian: " << pre_integration_->jacobian << std::endl;

            Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration_->covariance.inverse()).matrixL().transpose();
            //sqrt_info.setIdentity();
            residuals = sqrt_info * residuals;

            

       
            double sum_dt = pre_integration_->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration_->jacobian.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration_->jacobian.template block<3, 3>(O_P, O_BG);

            Eigen::Matrix3d dq_dbg = pre_integration_->jacobian.template block<3, 3>(O_R, O_BG);

            Eigen::Matrix3d dv_dba = pre_integration_->jacobian.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = pre_integration_->jacobian.template block<3, 3>(O_V, O_BG);

            if (pre_integration_->jacobian.maxCoeff() > 1e8 || pre_integration_->jacobian.minCoeff() < -1e8)
            {
                ROS_WARN("numerical unstable in preintegration");
                //std::cout << pre_integration->jacobian << std::endl;
                ///                ROS_BREAK();
            }

            Eigen::Matrix<double, 15, 6, Eigen::RowMajor> jacobian_pose_i;
            jacobian_pose_i.setZero();

            jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
            jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * m_G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

    #if 0
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
    #else
            Eigen::Quaterniond corrected_delta_q = pre_integration_->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg));
            jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
    #endif

            jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (m_G * sum_dt + Vj - Vi));

            // std::cout << "jacobian_pose_i: " << jacobian_pose_i << std::endl;

            jacobian_pose_i = sqrt_info * jacobian_pose_i;

            if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
            {
                ROS_WARN("numerical unstable in preintegration");
                //std::cout << sqrt_info << std::endl;
                //ROS_BREAK();
            }

            Eigen::Matrix<double, 15, 9, Eigen::RowMajor> jacobian_speedbias_i;
            jacobian_speedbias_i.setZero();
            jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
            jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
            jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

    #if 0
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
    #else
            //Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
            //jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
            jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration_->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
    #endif

            jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
            jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
            jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

            jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

            jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

            // std::cout << "jacobian_speedbias_i: " << jacobian_speedbias_i << std::endl;

            jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

            //ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
            //ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);

            Eigen::Matrix<double, 15, 6, Eigen::RowMajor> jacobian_pose_j;
            jacobian_pose_j.setZero();

            jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

    #if 0
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
    #else
            jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
    #endif

            // std::cout << "jacobian_pose_j: " << jacobian_pose_j << std::endl;
            jacobian_pose_j = sqrt_info * jacobian_pose_j;

            //ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
            //ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);

            Eigen::Matrix<double, 15, 9, Eigen::RowMajor> jacobian_speedbias_j;
            jacobian_speedbias_j.setZero();

            jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

            jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

            jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

            // std::cout << "jacobian_speedbias_j: " << jacobian_speedbias_j << std::endl;

            jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;
                

            jacobians.resize(4);
            jacobians[0] = jacobian_pose_i;
            jacobians[1] = jacobian_speedbias_i;
            jacobians[2] = jacobian_pose_j;
            jacobians[3] = jacobian_speedbias_j;
        }
        std::vector<Eigen::MatrixXd> Jacobians()
        {
            return jacobians;
        }

        Eigen::Matrix<double, 15, 1> Residual() 
        {
            return residuals;
        }   
        double Chi2() {
            return residuals.transpose() * residuals;
        }
        
    private:
        IntegrationBase *pre_integration_;

        Pose::Ptr m_curPose;
        Pose::Ptr m_startPose;

        Motion::Ptr m_curMotion;
        Motion::Ptr m_startMotion;

        Eigen::Matrix<double, 15, 1> residuals;
        std::vector<Eigen::MatrixXd> jacobians;

        Eigen::Vector3d m_G{0.0, 0.0, 9.8};

    };
}



#endif
