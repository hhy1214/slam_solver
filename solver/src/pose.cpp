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

    void Pose::Plus(VecX delta_) {

    m_p.head<3>() += delta_.head<3>(); //>>平移部分相加
    Eigen::Quaterniond q;
    q = m_q;
    q = q * Sophus::SO3d::exp(Vec3(delta_[3], delta_[4], delta_[5])).unit_quaternion();  // right multiplication with so3
    m_q = q.normalized();
//    Qd test = Sophus::SO3d::exp(Vec3(0.2, 0.1, 0.1)).unit_quaternion() * Sophus::SO3d::exp(-Vec3(0.2, 0.1, 0.1)).unit_quaternion();
//    std::cout << test.x()<<" "<< test.y()<<" "<<test.z()<<" "<<test.w() <<std::endl;
    }
    
} // namespace SLAM_Solver
