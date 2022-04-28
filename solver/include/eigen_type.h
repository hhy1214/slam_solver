#ifndef SOLVER_SLAM_EIGEN_TYPE_H
#define SOLVER_SLAM_EIGEN_TYPE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <map>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 2, 2> Mat22;
typedef Eigen::Matrix<double, 2, 3> Mat23;
typedef Eigen::Matrix<double, 3, 1> Vec3;

#endif