#ifndef SOLVER_COSTTWOFRAMETWOCAMFUNCTION_H
#define SOLVER_COSTTWOFRAMETWOCAMFUNCTION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include "pose.h"
#include "feature.h"
#include "eigen_type.h"
#include "utility.h"
#include "parameters.h"

namespace SLAM_Solver
{
    class costTwoFrameTwoCamFunction
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        POINTER_TYPEDEFS(costTwoFrameTwoCamFunction);

        costTwoFrameTwoCamFunction(Eigen::Vector3d pti_, Eigen::Vector3d ptj_) : m_pti(pti_), m_ptj(ptj_) {}

        void set_curPose(Pose::Ptr curPose_)
        {
            m_curPose = curPose_;
        }

        void set_startPose(Pose::Ptr startPose_)
        {
            m_startPose = startPose_;
        }

        void set_curFeature(FeatureID::Ptr curFeature_)
        {
            m_curFeature = curFeature_;
        }
        Pose::Ptr get_curPose()
        {
            return m_curPose;
        }

        Pose::Ptr get_startPose()
        {
            return m_startPose;
        }

        FeatureID::Ptr get_curFeature()
        {
            return m_curFeature;
        }
        void SetLeftTranslationImuFromCamera(Eigen::Quaterniond &qic_, Eigen::Vector3d &tic_)
        {
            qicLeft = qic_;
            ticLeft = tic_;
        }
        void SetRightTranslationImuFromCamera(Eigen::Quaterniond &qic_, Eigen::Vector3d &tic_)
        {
            qicRight = qic_;
            ticRight = tic_;
        }
        Eigen::Vector3d getLeftTranslation()
        {
            return ticLeft;
        }
        Eigen::Vector3d getRightTranslation()
        {
            return ticRight;
        }
        Eigen::Quaterniond getLeftRotate()
        {
            return qicLeft;
        }
        Eigen::Quaterniond getRightRotate()
        {
            return qicRight;
        }



        void ComputeResidual()
        {
            double inv_dep_i = get_curFeature()->get_invdep();

            Eigen::Quaterniond Qi;
            Qi = m_startPose->get_rotate();
            Eigen::Vector3d Pi = m_startPose->get_translation();
            int Pi_Id = m_startPose->get_ID();

            Eigen::Quaterniond Qj;
            Qj = m_curPose->get_rotate();
            Eigen::Vector3d Pj = m_curPose->get_translation();
            int Pj_Id = m_curPose->get_ID();

            Eigen::Vector3d pts_i_;
            auto it = m_curFeature->get_FeatureMeasure(Pi_Id);
            pts_i_ = it->get_leftpoint();

            Eigen::Vector3d pts_j_;
            auto it1 = m_curFeature->get_FeatureMeasure(Pj_Id);
            pts_j_ = it1->get_rightpoint();

            Eigen::Vector3d tic = getLeftTranslation();
            Eigen::Quaterniond qic = getLeftRotate();

            Eigen::Vector3d tic2 = getRightTranslation();
            Eigen::Quaterniond qic2 = getRightRotate();

            Eigen::Vector3d pts_camera_i = pts_i_ / inv_dep_i;
            Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
            Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
            Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
            Eigen::Vector3d pts_camera_j = qic2.inverse() * (pts_imu_j - tic2);

            double dep_j = pts_camera_j.z();
            residual_ = (pts_camera_j / dep_j).head<2>() - pts_j_.head<2>();

            Eigen::Matrix2d sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
            residual_ = sqrt_info * residual_;

//jacobians
            Eigen::Matrix3d Ri = Qi.toRotationMatrix();
            Eigen::Matrix3d Rj = Qj.toRotationMatrix();
            Eigen::Matrix3d ric = qic.toRotationMatrix();
            Eigen::Matrix3d ric2 = qic2.toRotationMatrix();
            Eigen::Matrix<double, 2, 3> reduce(2, 3);

            reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
              0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
            reduce = sqrt_info * reduce;
           
           
            // jacobians pose_i
            Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian_pose_i;

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric2.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = ric2.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;

            // jacobians pose_j
            Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian_pose_j;

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric2.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric2.transpose() * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;

            // jacobians ex_pose_left
            Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian_ex_pose;
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = ric2.transpose() * Rj.transpose() * Ri; 
            jaco_ex.rightCols<3>() = ric2.transpose() * Rj.transpose() * Ri * ric * -Utility::skewSymmetric(pts_camera_i);
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;

            // jacobians ex_pose_right
            Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian_ex_pose1;
            Eigen::Matrix<double, 3, 6> jaco_ex1;
            jaco_ex1.leftCols<3>() = - ric2.transpose();
            jaco_ex1.rightCols<3>() = Utility::skewSymmetric(pts_camera_j);
            jacobian_ex_pose1.leftCols<6>() = reduce * jaco_ex1;

            // jacobians feature
            Eigen::Vector2d jacobian_feature;
            jacobian_feature = reduce * ric2.transpose() * Rj.transpose() * Ri * ric * pts_i_ * -1.0 / (inv_dep_i * inv_dep_i);

            jacobians_.resize(3);
            jacobians_[0] = jacobian_pose_i;
            jacobians_[1] = jacobian_pose_j;
            jacobians_[2] = jacobian_feature;
        }
        std::vector<Eigen::MatrixXd> Jacobians() {
            return jacobians_;
        }

        VecX Residual() {
            return residual_;
        }
        double Chi2() {
            return residual_.transpose() * residual_;
        }

    private:
        bool ifFeature;              // 是否为视觉点约束
        Pose::Ptr m_curPose;         // 当前帧
        Pose::Ptr m_startPose;       // 起始帧
        FeatureID::Ptr m_curFeature; // 观测点
        Eigen::Vector3d m_pti;
        Eigen::Vector3d m_ptj;

        Eigen::Quaterniond qicLeft;     // 左目外参
        Eigen::Vector3d ticLeft;        
        Eigen::Quaterniond qicRight;    // 右目外参
        Eigen::Vector3d ticRight;

        VecX residual_;                // 残差
        std::vector<Eigen::MatrixXd> jacobians_; 
    };
    
}



#endif
