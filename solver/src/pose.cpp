#include "pose.h"

namespace SLAM_Solver
{

    void Pose::set_T(Eigen::Quaterniond &pose_q, Eigen::Vector3d &pose_t) {
        m_q = pose_q;
        m_p = pose_t;
    };

    Eigen::Matrix4d Pose::get_T() {
        Eigen::Matrix4d m_T = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d m_R;
        m_R = m_q.normalized().toRotationMatrix();
        m_T.block(0, 0, 3, 3) = m_R;
        m_T.block(0, 3, 3, 1) = m_p;
        return m_T;
    };

    int Pose::get_ID() {
        return m_frame_id;
    }

    Eigen::Quaterniond Pose::get_rotate() {
        return m_q;
    }

    Eigen::Vector3d Pose::get_translation() {
        return m_p;
    }
    
} // namespace SLAM_Solver
