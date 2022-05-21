#ifndef SOLVER_COSTFUNCTION_H
#define SOLVER_COSTFUNCTION_H

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
    class costFunction
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        POINTER_TYPEDEFS(costFunction);
        
        // 输入当前帧以及起始帧的ID以及对应的帧上观测
        costFunction(Eigen::Vector3d pti, Eigen::Vector3d ptj) : m_pti(pti), m_ptj(ptj){};

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

        void ComputeResidual()
        {
            double inv_dep_i = m_curFeature->get_invdep();

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
            pts_j_ = it1->get_leftpoint();

            /* 从camera系到imu再到世界系 */
            Eigen::Vector3d pts_camera_i = pts_i_ / inv_dep_i;
            Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
            Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
            Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
            Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

            double dep_j = pts_camera_j.z();
            Eigen::Matrix2d sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
            residual_ = (pts_camera_j / dep_j).head<2>() - pts_j_.head<2>(); /// J^t * J * delta_x = - J^t * r
            residual_ = sqrt_info * residual_;
            //    residual_ = information_ * residual_;   // remove information here, we multi information matrix in problem solver
            // std::cout << "residual_: " << std::endl
            //           << residual_ << std::endl;

            Mat33 Ri = Qi.toRotationMatrix();
            Mat33 Rj = Qj.toRotationMatrix();
            Mat33 ric = qic.toRotationMatrix();
            Mat23 reduce(2, 3);
            reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
                0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
            reduce = sqrt_info * reduce;
            //    reduce = information_ * reduce;
            // std::cout << "reduce: " << std::endl
            //           << reduce << std::endl;

            Eigen::Matrix<double, 2, 6> jacobian_pose_i;
            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            // std::cout << "jacobian_pose_i: " << std::endl
            //           << jacobian_pose_i << std::endl;

            Eigen::Matrix<double, 2, 6> jacobian_pose_j;
            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_j);
            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            // std::cout << "jacobian_pose_j: " << std::endl
            //           << jacobian_pose_j << std::endl;

            Eigen::Vector2d jacobian_feature;
            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i_ * -1.0 / (inv_dep_i * inv_dep_i);
            // std::cout << "jacobian_feature: " << std::endl
            //           << jacobian_feature << std::endl;

            jacobians_.resize(3);
            jacobians_[0] = jacobian_feature;
            // std::cout << "jacobians_[0]: " << std::endl
            //           << jacobians_[0] << std::endl;
            jacobians_[1] = jacobian_pose_i;
            // std::cout << "jacobians_[1]: " << std::endl
            //           << jacobians_[1] << std::endl;
            jacobians_[2] = jacobian_pose_j;
            // std::cout << "jacobians_[2]: " << std::endl
            //           << jacobians_[2] << std::endl;            
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

        void SetTranslationImuFromCamera(Eigen::Quaterniond &qic_, Eigen::Vector3d &tic_)
        {
            qic = qic_;
            tic = tic_;
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
        Eigen::Quaterniond qic;
        Eigen::Vector3d tic;

        //保存的雅各比与残差
        VecX residual_;                // 残差
        std::vector<Eigen::MatrixXd> jacobians_; // 雅可比，每个雅可比维度是 residual x vertex[i]
    };

} // namespace SLAM_Solver

#endif
