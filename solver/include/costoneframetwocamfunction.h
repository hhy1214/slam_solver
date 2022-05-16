#ifndef SOLVER_COSTONEFRAMETWOCAMFUNCTION_H
#define SOLVER_COSTONEFRAMETWOCAMFUNCTION_H

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
    class costOneFrameTwoCamFunction
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        POINTER_TYPEDEFS(costOneFrameTwoCamFunction);

        costOneFrameTwoCamFunction(Eigen::Vector3d pti_, Eigen::Vector3d ptj_) : m_pti(pti_), m_ptj(ptj_) {}
        void set_curPose(Pose::Ptr curPose_)
        {
            m_curPose = curPose_;
        }
        void set_curFeature(FeatureID::Ptr curFeature_)
        {
            m_curFeature = curFeature_;
        }
        Pose::Ptr get_curPose()
        {
            return m_curPose;
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
            Eigen::Vector3d tic = getLeftTranslation();
            Eigen::Quaterniond qic = getLeftRotate();

            Eigen::Vector3d tic2 = getRightTranslation();
            Eigen::Quaterniond qic2 = getRightRotate();

            double inv_dep_i = get_curFeature()->get_invdep();

            int Pid = get_curPose()->get_ID();
            Eigen::Vector3d pts_i_ = get_curFeature()->get_FeatureMeasure(Pid)->get_leftpoint();
            Eigen::Vector3d pts_j_ = get_curFeature()->get_FeatureMeasure(Pid)->get_rightpoint();

            Eigen::Vector3d pts_camera_i = pts_i_ / inv_dep_i;
            Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
            Eigen::Vector3d pts_imu_j = pts_imu_i;
            Eigen::Vector3d pts_camera_j = qic2.inverse() * (pts_imu_j - tic2);
            
            double dep_j = pts_camera_j.z();
            residual_ = (pts_camera_j / dep_j).head<2>() - pts_j_.head<2>();
            
            Eigen::Matrix2d sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
            residual_ = sqrt_info * residual_;

// jacobians

            Eigen::Matrix3d ric = qic.toRotationMatrix();
            Eigen::Matrix3d ric2 = qic2.toRotationMatrix();
            Eigen::Matrix<double, 2, 3> reduce(2, 3);

            reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
                0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

            reduce = sqrt_info * reduce;

            // jacobians ex_poseleft
            Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian_ex_pose;
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = ric2.transpose(); 
            jaco_ex.rightCols<3>() = ric2.transpose() * ric * -Utility::skewSymmetric(pts_camera_i);
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;

            // jacobians ex_poseRight
            Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian_ex_pose1;
            Eigen::Matrix<double, 3, 6> jaco_ex1;
            jaco_ex1.leftCols<3>() = - ric2.transpose();
            jaco_ex1.rightCols<3>() = Utility::skewSymmetric(pts_camera_j);
            jacobian_ex_pose1.leftCols<6>() = reduce * jaco_ex1;

            // jacobians feature
            Eigen::Vector2d jacobian_feature;
            jacobian_feature = reduce * ric2.transpose() * ric * pts_i_ * -1.0 / (inv_dep_i * inv_dep_i);
        
            jacobians_.resize(1);
            jacobians_[0] = jacobian_feature;
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
