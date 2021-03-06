#ifndef SOLVER_SLAM_PARAMETERS_H
#define SOLVER_SLAM_PARAMETERS_H

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility.h"
#include "macro.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "glog/logging.h"
#include <fstream>
#include <map>

const double FOCAL_LENGTH = 460.0;
extern double ACC_N, ACC_W, GYR_N, GYR_W;
extern Eigen::Vector3d G;
enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};
enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};
enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

namespace SLAM_Solver
{
    struct Camera
    {
        std::string cameraName;
        int w;
        int h;
        double k1;
        double k2;
        double p1;
        double p2;
        double fx;
        double fy;
        double cx;
        double cy;
    };

    class Parameters
    {
    public:
        POINTER_TYPEDEFS(Parameters);
        void set_parameters(std::string config_yaml);

    public:

        int USE_IMU;
        int NUM_OF_CAM;
        int STEREO;

        int LARDMARK_TYPE;

        //camera parameters
        std::vector<Camera> Cameras;                //相机内参

        // image and imu topic
        std::vector<std::string> CAMERA_IMU_TOPIC;  //IMU话题
        std::vector<Eigen::Matrix4d> CAM_TIC;       //相机到IMU外参
        std::vector<std::string> CAM_NAMES;         
        // imu noise
        double ACC_N, ACC_W, GYR_N, GYR_W;
    };

} // namespace SLAM_Solver

#endif