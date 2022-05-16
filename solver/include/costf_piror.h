#ifndef SOLVER_COSTF_PIROR_H
#define SOLVER_COSTF_PIROR_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include "pose.h"
#include "feature.h"
#include "eigen_type.h"
#include "utility.h"
#include <iostream>
#include "sophus/se3.hpp"

#define USE_SO3_JACOBIAN 1

namespace SLAM_Solver
{
    class costf_piror
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        costf_piror(const Vec3 &p, const Eigen::Quaterniond &q) :Pp_(p), Qp_(q) {}

        /// 计算残差
        void ComputeResidual() {
            
        }

        /// 计算雅可比
        void ComputeJacobians();

    private:
        Vec3 Pp_;                 // pose prior
        Eigen::Quaterniond Qp_;   // Rotation prior
    };
} // namespace SLAM_Solver
#endif