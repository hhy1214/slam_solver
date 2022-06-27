#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <opencv2/core/eigen.hpp>
#include <vector>
#include <random>
#include <string>

//自定义库
#include "pose.h"
#include "motion.h"
#include "feature.h"
#include "slam_solver.h"
#include "costfunction.h"
#include "costIMUFunction.h"
#include "costoneframetwocamfunction.h"
#include "costTwoFrameTwoCamFunction.h"
#include "eigen_type.h"
#include "integration_base.h"

using namespace std;

struct Frame {
    Frame(double stamp_time_, Eigen::Matrix3d R, Eigen::Vector3d t) : stamp_time(stamp_time_), Rwc(R), qwc(R), twc(t) {}; // 构造函数初始化
    Frame(double stamp_time_, Eigen::Quaterniond q, Eigen::Vector3d t) : stamp_time(stamp_time_), qwc(q), twc(t) {
        Rwc = qwc.normalized().toRotationMatrix();
    }
    //当前帧到世界系下的变换
    double stamp_time; //时间戳
    Eigen::Matrix3d Rwc;      // 矩阵表示旋转
    Eigen::Quaterniond qwc;   // 四元数表旋转
    Eigen::Vector3d twc;    
    std::unordered_map<int, Eigen::Vector3d> featurePerId;  
};
struct Imu {
    Imu(double stamp_time_, Eigen::Vector3d acc_, Eigen::Vector3d gyr_) : stamp_time(stamp_time_), acc(acc_), gyr(gyr_) {}
    double stamp_time;
    Eigen::Vector3d acc, gyr;
};
// 配置文件路径、左右目相机位姿路径、IMU路径、3D点输出路径
std::string config_file;
std::string cam_left_file, cam_right_file;
std::string imu_file;
std::string point_file;
// IMU积分路径、相机位姿优化输出路径
std::string imu_int_file;
std::string opt_pose_file;
std::string gauss_pose_file;
int POSE_NUM, POINT_NUM, IMU_NUM;
int USE_IMU; //是否使用IMU数据
int STEREO;  //是否使用双目
// 相机到IMU的外参
Eigen::Vector3d tic0, tic1;     
Eigen::Quaterniond qic0, qic1;
Eigen::Matrix3d Ric0, Ric1;
double fx; //焦距

std::vector<Frame> cameraLeft, cameraRight;
std::vector<Frame> noise_cameraLeft, noise_cameraRight;
std::vector<Imu> imus, noise_imus;
std::vector<Eigen::Vector3d> points;


const int MAXN = 600 +10;
const int NUMCAM = 2;
// imu 预积分需要
bool first_imu = false;
Eigen::Vector3d acc_0, gyr_0;
SLAM_Solver::IntegrationBase *pre_integrations[MAXN];
std::vector<double> dt_buf[MAXN];
std::vector<Eigen::Vector3d> linear_acceleration_buf[MAXN];
std::vector<Eigen::Vector3d> angular_velocity_buf[MAXN];
Eigen::Vector3d Ps[MAXN];
Eigen::Vector3d Vs[MAXN];
Eigen::Matrix3d Rs[MAXN];
Eigen::Vector3d Bas[MAXN];
Eigen::Vector3d Bgs[MAXN];
Eigen::Vector3d g{0, 0, 9.805};
Eigen::Vector3d initPose, initVelcity;
Eigen::Matrix3d initR0;


void init()
{
    for(int i = 0; i < MAXN; i ++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
    }
    Ric0 = Eigen::Matrix3d::Identity();
    tic0 = Eigen::Vector3d::Zero();
    qic0 = Eigen::Quaterniond(Ric0);
}
void initFirstIMUPose()
{
    printf("init first imu pose\n");


    Eigen::Matrix3d Rwi = cameraLeft[0].Rwc * Ric0.transpose();
    Eigen::Vector3d twi = -cameraLeft[0].Rwc * Ric0.transpose() * tic0 + cameraLeft[0].twc;
    Eigen::Quaterniond qwi = Eigen::Quaterniond(Rwi);
    Rs[0] = Rwi;
    Ps[0] = twi;
    Vs[0] << -0 ,0.628319,0.15708;
    // Vs[0] = 
    std::cout << "Rs[0] : \n" << Rs[0] << std::endl << std::endl;
    std::cout << "Ps[0] : \n" << Ps[0] << std::endl << std::endl;
    std::cout << "Vs[0] : \n" << Vs[0] << std::endl << std::endl; 
}
void readConfig()
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "配置文件路径有错误!!!" << std::endl;
        return;
    }
    USE_IMU = fsSettings["imu"];
    STEREO = fsSettings["stereo"];
    fsSettings["cam_pose_left_file"] >> cam_left_file;
    fsSettings["point_file"] >> point_file;
    cv::FileNode n = fsSettings["cam0_left_projection_parameters"];
    fx = static_cast<double>(n["fx"]);
    fsSettings["opt_pose_file"] >> opt_pose_file;
    fsSettings["gauss_pose_file"] >> gauss_pose_file;
    std::cout << "USE_IMU : " << USE_IMU << std::endl;
    std::cout << "STEREO : " << STEREO << std::endl;
    std::cout << "cam_left_file : " << cam_left_file << std::endl;
    std::cout << "point_file : " << point_file << std::endl;
    std::cout << "fx : " << fx << std::endl;
    std::cout << "opt_pose_file: " << opt_pose_file << std::endl;

    
    if(USE_IMU) 
    {
        fsSettings["imu_int_file"] >> imu_int_file;
        fsSettings["imu_pose_file"] >> imu_file;
        cv::Mat cv_Tic0;
        Eigen::Matrix4d Tic0;
        fsSettings["imu_T_cam0_left"] >> cv_Tic0;
        cv::cv2eigen(cv_Tic0, Tic0);
        Ric0 = Tic0.block<3, 3>(0, 0);
        tic0 = Tic0.block<3, 1>(0, 3);
        qic0 = Eigen::Quaterniond(Ric0);

        std::cout << "Tic0 : \n" << cv_Tic0 << std::endl;
        std::cout << "Ric0 : \n" << Ric0 << std::endl;
        std::cout << "tic0 : \n" << tic0 << std::endl;
        std::cout << "imu_pose_file : " << imu_file << std::endl;
        std::cout << "imu_int_file : " << imu_int_file << std::endl;
    }
    if(STEREO)
    {
        fsSettings["cam_pose_right_file"] >> cam_right_file;
        std::cout << "cam_right_file : " << cam_right_file << std::endl;
        if(USE_IMU)
        {
            cv::Mat cv_Tic1;
            Eigen::Matrix4d Tic1;
            fsSettings["imu_T_cam0_right"] >> cv_Tic1;
            cv::cv2eigen(cv_Tic1, Tic1);
            Ric1 = Tic1.block<3, 3>(0, 0);
            tic1 = Tic1.block<3, 1>(0, 3);
            qic1 = Eigen::Quaterniond(Ric1);
            std::cout << "Tic1 : \n" << cv_Tic1 << std::endl;
            std::cout << "Ric1 : \n" << Ric1 << std::endl;
            std::cout << "tic1 : \n" << tic1 << std::endl;
        }
    }
}
void readCamPose(std::string filename, std::vector<Frame>& camereData)
{
    camereData.clear();
    std::fstream fcin(filename);
    if(!fcin)
    {
        std::cout << filename << " doesn't exist!!!" << std::endl;
        return;
    }
    /* 
        相机位姿以TUM数据格式存储
        time_stamp tx ty tz qx qy qz qw
    */
    while(!fcin.eof())
    {
        double time;
        Eigen::Vector3d t;
        Eigen::Quaterniond q;
        fcin >> time >> t.x() >> t.y() >> t.z() >> q.x() >> q.y() >> q.z() >> q.w();
        // std::cout << id << " " << q.w() << " " << q.x() <<  " " << q.y() << " " << q.z() << " " <<  t.x() << " " << t.y() << " " << t.z() << std::endl;
        Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        camereData.push_back(Frame(time, R, t));
    }
}
void readIMU(std::string filename, std::vector<Imu>& imuData)
{
    imuData.clear();
    std::fstream fcin(filename);
    if(!fcin)
    {
        std::cout << filename << " doesn't exist !!!" << std::endl;
        return;
    }
    /*
        IMU数据格式
        time_stamp gyr(0) gyr(1) gyr(2) acc(0) acc(1) acc(2)
    */
   while(!fcin.eof())
   {
       double time;
        Eigen::Vector3d t, acc, gyr;
        Eigen::Quaterniond q;
  
        fcin >> time >> gyr.x() >> gyr.y() >> gyr.z() >> acc.x() >> acc.y() >> acc.z();
        imus.push_back(Imu(time, acc, gyr));
   }
}
void outputCamPose(std::vector<Frame>& cameraData)
{
    // 
    for(int i = 0; i < cameraData.size(); i ++)
    {
        auto it = cameraData[i];
        Eigen::Quaterniond q = it.qwc;
        Eigen::Vector3d t = it.twc;
        std::cout << "i : " << i << " ";
        std::cout << std::setprecision(5) <<  it.stamp_time <<  " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() <<  " " << q.w() <<  std::endl;
    }
}
void outputIMU(std::vector<Imu>& imuData)
{
    for(int i = 0; i < imuData.size(); i ++)
    {
        auto it = imuData[i];
        Eigen::Vector3d acc, gyr;
        acc = it.acc; gyr = it.gyr;
        std::cout << "i : " << i << " ";
        std::cout << it.stamp_time << " " <<  gyr(0) << " " << gyr(1) << " " <<  gyr(2) << " " << acc(0) << " " << acc(1) << " " << acc(2) << " " << std::endl;
    }
}
void savePoseAsTum(std::string filename, std::vector<Frame>& cameraData)
{
    std::ofstream fout(filename);
    for(int i = 0; i < cameraData.size(); i ++)
    {
        auto it = cameraData[i];
        Eigen::Quaterniond q = it.qwc;
        Eigen::Vector3d t = it.twc;

        fout << std::setprecision(5) <<  it.stamp_time <<  " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() <<  " " << q.w() <<  std::endl;
    }
    fout.close();
}
void genPoint(std::string filename, std::vector<Eigen::Vector3d>& pointData)
{
    pointData.clear();
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::cout << "seed : " << seed << std::endl;
    std::mt19937 gen(seed);
    std::ofstream fout(filename);
    POINT_NUM = 100;
    for(int i = 0; i < POINT_NUM; i ++)
    {
        std::uniform_real_distribution<double> xy_rand(-4.0, 4.0);
        std::uniform_real_distribution<double> z_rand(4.0, 8.0);
        Eigen::Vector3d Pw = Eigen::Vector3d(xy_rand(gen), xy_rand(gen), z_rand(gen));
        fout << std::setprecision(20) << i << " " << Pw.x() << " " << Pw.y() << " " << Pw.z() << std::endl;
        pointData.push_back(Pw);
    }
}
void readData()
{
    readCamPose(cam_left_file, cameraLeft);
    POSE_NUM = cameraLeft.size();
    std::cout << "cameraLeft NUM : " << cameraLeft.size() << std::endl;
    // outputCamPose(cameraLeft);

    genPoint(point_file, points);
    std::cout << "points size : " << points.size() << std::endl;
    if(USE_IMU)
    {
        readIMU(imu_file, imus);
        IMU_NUM = imus.size();
        std::cout << "IMU_NUM : " << IMU_NUM << std::endl;
        // outputIMU(imus);
    }
    if(STEREO)
    {
        readCamPose(cam_right_file, cameraRight);
        std::cout << "cameraRight NUM : " << cameraRight.size() << std::endl;
    }
}
void addNoiseCam(std::vector<Frame>& cameraData, std::vector<Frame>& noise_cameraData, std::vector<Eigen::Vector3d>& pointData)
{
    noise_cameraData.clear();
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 gen(seed);
    for(int i = 0; i < cameraData.size(); i ++)
    {
        std::normal_distribution<double> noise_pdf(0, M_PI / 90.0);
        Eigen::Vector3d euler_angles = cameraData[i].qwc.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
        // std::cout << "cameraPoses 原本旋转角: " << euler_angles.z() << " " << euler_angles.y() << " " << euler_angles.x() << std::endl;
       
        //相机旋转添加高斯噪声
        double noise_z = noise_pdf(gen);
        double noise_y = noise_pdf(gen);
        double noise_x = noise_pdf(gen);

        // double noise_z = 0;
        // double noise_y = 0;
        // double noise_x = 0;
        Eigen::AngleAxisd rotation_noise_z(noise_z, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd rotation_noise_y(noise_y, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rotation_noise_x(noise_x, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q = rotation_noise_z * rotation_noise_y * rotation_noise_x;
        Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
        Eigen::Matrix3d rotationyan = cameraData[i].qwc.normalized().toRotationMatrix();
        Eigen::Matrix3d noise_rotation = rotationMatrix * rotationyan;
        
        Eigen::Vector3d t = cameraData[i].twc;
        std::normal_distribution<double> noise_xyz(0.0, 0.4);
        t[0] += noise_xyz(gen);
        t[1] += noise_xyz(gen);
        t[2] += noise_xyz(gen);

        // t[0] += 0;
        // t[1] += 0;
        // t[2] += 0;
        noise_cameraData.push_back(Frame(cameraData[i].stamp_time, noise_rotation, t));
    }
    for(int j = 0; j < pointData.size(); j ++)
    {
        std::normal_distribution<double> noise_uv(0.0, 2.0 / fx);
        Eigen::Vector3d Pw = pointData[j];
        for(int i = 0; i < cameraData.size(); i ++)
        {
            Eigen::Vector3d Pc = cameraData[i].Rwc.transpose() * (Pw - cameraData[i].twc);
            Pc = Pc / Pc.z();
            Pc[0] += noise_uv(gen);
            Pc[1] += noise_uv(gen);
            noise_cameraData[i].featurePerId.insert(std::make_pair(j, Pc));
        }
    }
}
void addNoise()
{
    addNoiseCam(cameraLeft, noise_cameraLeft, points);
    std::cout << "noise_cameraLeft size : " << noise_cameraLeft.size() << std::endl;
    savePoseAsTum(gauss_pose_file, noise_cameraLeft);
    if(STEREO)
    {
        addNoiseCam(cameraRight, noise_cameraRight, points);
    }
}
std::vector<Imu> getIMUData(double start_time, double last_time, std::vector<Imu>& imuData)
{
    std::vector<Imu> curImus;
    curImus.clear();
    for(int i = 0; i < imuData.size(); i ++)
    {
        double cur_time = imuData[i].stamp_time;
        if(cur_time >= start_time && cur_time < last_time)
        {
            curImus.push_back(imuData[i]);
        }
    }
    return curImus;
}
void processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity, int frame_count)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new SLAM_Solver::IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
      
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}
void syncCamIMU()
{
    double cur_time = -1;
    POSE_NUM --;
    std::cout << "solve : POSE NUM = " << POSE_NUM << std::endl;
    for(int i = 0; i < POSE_NUM; i ++)
    {
        double start_time;
        if(i == 0) start_time = 0;
        else 
        {
            start_time = cameraLeft[i - 1].stamp_time;
        }
        auto curImus = getIMUData(start_time, cameraLeft[i].stamp_time, imus);
        // std::cout << "i :  " << i <<  " ,curImus size : " << curImus.size() << std::endl;
        if(i == 0) initFirstIMUPose();
        for(auto it : curImus)
        {
            double t = it.stamp_time;
            if(cur_time < 0) cur_time = t;
            double dt = t - cur_time;
            // std::cout << "dt : " << dt << std::endl;
            cur_time = t;
            processIMU(dt, it.acc, it.gyr, i);
        }

        {
            int frame_count = i;
            if(frame_count < POSE_NUM)
            {
                frame_count++;
                int prev_frame = frame_count - 1;
                Ps[frame_count] = Ps[prev_frame];
                Vs[frame_count] = Vs[prev_frame];
                Rs[frame_count] = Rs[prev_frame];
                Bas[frame_count] = Bas[prev_frame];
                Bgs[frame_count] = Bgs[prev_frame];
            }
        }
    }
    std::ofstream fout(imu_int_file);
    for(int i = 0; i < POSE_NUM; i ++)
    {
        Eigen::Quaterniond q(Rs[i]);
        Eigen::Vector3d t(Ps[i]);
        fout << std::setprecision(20) << cameraLeft[i].stamp_time << " " << t[0] << " " << t[1] << " " << t[2] << " "
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    fout.close();
}
void optimization()
{
    SLAM_Solver::BA_problem solver;
    solver.initialStructure(config_file);
    

    std::cout << "Ric0 : \n" << Ric0 << std::endl;
    std::cout << "tic0 : \n" << tic0 << std::endl;
    
    for(int i = 0; i < POSE_NUM; i ++)
    {
        Eigen::Matrix3d Rwi = noise_cameraLeft[i].Rwc * Ric0.transpose();
        Eigen::Vector3d twi = -noise_cameraLeft[i].Rwc * Ric0.transpose() * tic0 + noise_cameraLeft[i].twc;
        Eigen::Quaterniond qwi = Eigen::Quaterniond(Rwi);

        // Eigen::Vector3d twi = noise_cameraLeft[i].twc;
        // Eigen::Quaterniond qwi = noise_cameraLeft[i].qwc;

        // Eigen::Vector3d twi = Ps[i];
        // Eigen::Quaterniond qwi = Eigen::Quaterniond(Rs[i]);

        solver.addPoseParameterBlock(twi, qwi, i);
        if(USE_IMU)
        {
            solver.addIMUParameterBlock(i, Vs[i], Bas[i], Bgs[i]);
        }
    }
    if(USE_IMU)
    {
        for(int i = 0; i < POSE_NUM - 1; i ++) 
        {
            // LOG(INFO) << "addIMUResidualBlock";
            solver.addIMUResidualBlock(i, i + 1, pre_integrations[i+1]);
        }
    }
    for(int i = 0; i < POINT_NUM; i ++) 
    {
        Eigen::Vector3d Pw = points[i];
        Eigen::Vector3d Pc_i = cameraLeft[0].Rwc.transpose() * (Pw - cameraLeft[0].twc);
        if(Pc_i.z() < 0) continue;
        double inverse_depth = 1. / Pc_i.z();
        solver.addFeatureParameterBlock(inverse_depth, i);
        //添加单目残差
        for(int j = 1; j < POSE_NUM; j ++)
        {
            Eigen::Vector3d Pc_j = cameraLeft[j].Rwc.transpose() * (Pw - cameraLeft[j].twc);
            if(Pc_j.z() < 0) continue;
            Eigen::Vector3d pt_i = noise_cameraLeft[0].featurePerId.find(i)->second; // 起始帧 0帧
            Eigen::Vector3d pt_j = noise_cameraLeft[j].featurePerId.find(i)->second; // 第j帧
            solver.addFeatureResidualBlock(0, j, i, pt_i, pt_j);
        }
        // 添加双目残差
        if(STEREO)
        {
            for(int j = 1; j < POSE_NUM; j ++)
            {   
                Eigen::Vector3d Pc_j = cameraRight[j].Rwc.trace() * (Pw - cameraRight[j].twc);
                if(Pc_j.z() < 0) continue;
                Eigen::Vector3d pt_i = noise_cameraLeft[0].featurePerId.find(i)->second;
                Eigen::Vector3d pt_j = noise_cameraRight[j].featurePerId.find(i)->second;
                solver.addStereoFeatureTwoFtwoCResidual(0, j, i, pt_i, pt_j);
            }
        }

    }
    //边缘化第一帧
    // solver.setMargin(1);
    // std::cout << "！！！！！！！！！！边缘化第一帧 ！！！！！！！！" << std::endl;

    // solver.setMargin(0);
    // std::cout << "！！！！！！！！！！边缘化第零帧 ！！！！！！！！" << std::endl;
    solver.solve();

    std::vector<Eigen::Quaterniond> q_opt;
    std::vector<Eigen::Vector3d> t_opt;
    solver.getPoseResults(q_opt, t_opt);

    
    std::ofstream fout(opt_pose_file);
    for(int i = 0; i < POSE_NUM; i ++) {
        Eigen::Quaterniond q = q_opt[i];
        Eigen::Vector3d t = t_opt[i];
        Eigen::Matrix3d r = q.normalized().toRotationMatrix();      
        fout << std::setprecision(20) << cameraLeft[i].stamp_time << " " <<  t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;  
    }
    fout.close(); 
}
void solve()
{
    if(USE_IMU)
    {
        syncCamIMU();
    }
    optimization();
}
int main(int argv, char** argc)
{
    if(argv != 2) 
    {
        std::cout << "请输入配置文件名!!!" << std::endl;
        return -1;
    }
    config_file = argc[1];
    init();
    readConfig();
    readData();
    addNoise();
    solve();

    return 0;
}