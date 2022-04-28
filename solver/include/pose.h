#ifndef SOLVER_SLAM_POSE_H
#define SOLVER_SLAM_POSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include "macro.h"
#include <unordered_map>
#include "eigen_type.h"
#include "utility.h"
#include "sophus/so3.hpp"

namespace SLAM_Solver
{
    class Pose
    {
    public:
        POINTER_TYPEDEFS(Pose);
        Pose(int poseId_) : m_frame_id(poseId_), localDimension(6){};

        void set_T(Eigen::Quaterniond &qic_, Eigen::Vector3d &tic_);
        Eigen::Matrix4d get_T();
        int get_ID();
        int get_localDimension()
        {
            return localDimension;
        }
        Eigen::Quaterniond get_rotate();
        Eigen::Vector3d get_translation();
        void Plus(VecX delta_);

    public:
        int m_frame_id;                         // frame id

    private:
        int localDimension;                     // 局部参数化维度
        
        std::vector<int> m_PointID;             // feature id
        std::vector<Eigen::Vector2d> m_pointxy; // 对应feature的2D坐标，即uv
        Eigen::Vector3d m_p;                    // frame 的 t
        Eigen::Quaterniond m_q;                 // frame 的 R
        // std::list<Feature> m_Keypoints;
        // IntegrationBase *pre_integration_;            //IMU预积分信息
    };

    

} // namespace SLAM_Solver

#endif