#ifndef SOLVER_SLAM_MOTION_H
#define SOLVER_SLAM_MOTION_H

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
    class Motion
    {
    public:
        POINTER_TYPEDEFS(Motion);
        Motion(int poseId_) : m_frame_id(poseId_), localDimension(9){};

        void set_Motion(Eigen::Vector3d _Vs, Eigen::Vector3d _Ba, Eigen::Vector3d _Bg);
        std::vector<Eigen::Vector3d> get_Motion();
        int get_ID();
        int get_localDimension()
        {
            return localDimension;
        }

        int OrderingId() const { return ordering_id_; }

        void SetOrderingId(unsigned long id) { ordering_id_ = id; };
        Eigen::Vector3d get_Vs();
        Eigen::Vector3d get_Ba();
        Eigen::Vector3d get_Bg();
        void Plus(VecX delta_);

    public:
        int m_frame_id;                         // frame id

    private:
        int localDimension;                     // 局部参数化维度
        Eigen::Vector3d Vs, Ba, Bg;  
        unsigned long ordering_id_ = 0;           
    };

    

} // namespace SLAM_Solver

#endif
