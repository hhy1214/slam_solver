#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <opencv2/core/eigen.hpp>
#include "pose.h"
#include "slam_solver.h"
#include "costfunction.h"
#include "eigen_type.h"

using namespace std;
Eigen::Matrix4d T_I_C;
double fx = 383.1009216308594, cx = 320.5055236816406, cy = 237.51809692382812;

struct Frame {
    Frame(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R), qwc(R), twc(t) {}; // 构造函数初始化
    Eigen::Matrix3d Rwc;      // 矩阵表示旋转
    Eigen::Quaterniond qwc;   // 四元数表旋转
    Eigen::Vector3d twc;      
    /*【unordered_map 哈希表对应的容器。哈希表是根据关键码值而直接进行访问的数据结构，也就是说，它通过把关键码值映射到表中一个位置来访问记录，
    以加快查找的速度，这个映射函数叫做散列函数】 */ 
    unordered_map<int, Eigen::Vector3d> featurePerId; // 该帧观测到的特征以及特征id，
};

/**
 * @brief  产生世界坐标系下的虚拟数据: 相机姿态, 特征点, 以及每帧观测
 * @note   
 * @param  &cameraPoses: 相机位姿
 * @param  &points:  路标点
 * @retval None
 */
void GetSimDataInWordFrame(vector<Frame> &cameraPoses, vector<Eigen::Vector3d> &points) {
    int featureNums = 10;  // 特征数目，假设每帧都能观测到所有的特征
    int poseNums = 10;     // 相机数目

    double radius = 8;
    for (int n = 0; n < poseNums; ++n) {
        double theta = n * 2 * M_PI / (poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()); // 角度和轴
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta)); // r*cos(theta) - r, r*sin(theta), 1*sin(2*theta)
        // std::cout << "44444444444444: " << t << std::endl;
        cameraPoses.push_back(Frame(R, t)); // 返回R t
    }

    // 随机数生成三维特征点
    std::default_random_engine generator;
    std::normal_distribution<double> noise_pdf(0., 1. / 1000.);  // 2pixel / focal 。噪声分布；越大
    for (int j = 0; j < featureNums; ++j) {
        std::uniform_real_distribution<double> xy_rand(-4, 4.0); // 随机生成数的范围
        std::uniform_real_distribution<double> z_rand(4., 8.);

        Eigen::Vector3d Pw(xy_rand(generator), xy_rand(generator), z_rand(generator)); // 随机生成3维特征点
        // std::cout << "1111111111111" << Pw << std::endl;
        points.push_back(Pw);

        // 在每一帧上的观测量
        for (int i = 0; i < poseNums; ++i) {
            Eigen::Vector3d Pc = cameraPoses[i].Rwc.transpose() * (Pw - cameraPoses[i].twc);           //相机坐标系？？？
            Pc = Pc / Pc.z();               // 归一化图像平面
            cameraPoses[i].featurePerId.insert(make_pair(j, Pc)); 
        }
    }
}

//对随机数产生的真实值添加噪声，作为之后需要引用的观测值。
void add_camera_pose_noise(vector<Frame> &cameraPoses, vector<Eigen::Vector3d> &points, vector<Frame> &cameranoisePoses) {

    // 生成随机数
    std::default_random_engine generator;    
    for(int i = 0; i < cameraPoses.size(); i++) {
        std::normal_distribution<double>noise_pdf(0, M_PI / 90);
        Eigen::Vector3d euler_angles = cameraPoses[i].qwc.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
        // std::cout << "cameraPoses原本的旋转角： " << euler_angles << std::endl;
        double noise_z = noise_pdf(generator);
        Eigen::AngleAxisd rotation_noise_z(noise_z, Eigen::Vector3d::UnitZ());
        double noise_y = noise_pdf(generator);
        Eigen::AngleAxisd rotation_noise_y(noise_y, Eigen::Vector3d::UnitY());
        double noise_x = noise_pdf(generator);
        Eigen::AngleAxisd rotation_noise_x(noise_x, Eigen::Vector3d::UnitX());
        Eigen::Quaternion<double> q = rotation_noise_z * rotation_noise_y * rotation_noise_x;
        Eigen::Matrix3d rotationMatrix = q.matrix();
        Eigen::Matrix3d rotationyan = cameraPoses[i].qwc.matrix();
        Eigen::Matrix3d noise_rotation = rotationMatrix * rotationyan;
        Eigen::Quaterniond q_noise = Eigen::Quaterniond(noise_rotation);

        Eigen::Vector3d noise_euler_angles = q_noise.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
        // std::cout << "cameraPoses噪声的旋转角： " << noise_euler_angles << std::endl;

        //构造平移误差
        Eigen::Vector3d t = cameraPoses[i].twc;
        Eigen::Vector3d t1 = cameraPoses[i].twc;
        // std::cout << "cameraPoses起始的平移： " << t << std::endl;
        std::normal_distribution<double> xyz_rand(0., 0.4); // 随机生成数的范围
        t[0] += xyz_rand(generator);
        t[1] += xyz_rand(generator);
        t[2] += xyz_rand(generator);
        // std::cout << "cameraPoses噪声的平移： " << t << std::endl;
        cameranoisePoses.push_back(Frame(noise_rotation, t));
        
    }

    for(int j = 0; j < points.size(); j++) {
        std::normal_distribution<double> noise_uv(0., 2. / 383.);
        Eigen::Vector3d Pw = points[j];
        for(int i = 0; i < cameraPoses.size(); i++) {
            Eigen::Vector3d Pc = cameraPoses[i].Rwc.transpose() * (Pw - cameraPoses[i].twc);
            Pc = Pc / Pc.z();
            Pc[0] += noise_uv(generator);
            Pc[1] += noise_uv(generator);
            cameranoisePoses[i].featurePerId.insert(make_pair(j, Pc)); 
        }

    }
}

void matrix()
{
    Eigen::MatrixXd H(Eigen::MatrixXd::Zero(300, 300));
    Eigen::MatrixXd b(Eigen::MatrixXd::Zero(300, 1));
    std::default_random_engine generator1;
    

    for(int i = 0; i < 300; i ++ ) {
        // std::cout << "i: " << i << std::endl;
        std::normal_distribution<double>q_rand(0, 0.005);
        for(int j = 0; j < i; j++) {   
            // std::cout << "j: " << j << std::endl;         
            double x = q_rand(generator1);
            // std::cout << "zhiwei: " << x << std::endl;
            H(i, j) += x;
            if(i != j){
                H(j, i) += x;
            }
        }
        b(i, 0) += q_rand(generator1);
    }

    std::ofstream fout("/home/hhy/bishe/SLAM_solver/src/SlamSolver/results/矩阵.txt");
    fout << "矩阵H差值为： " << std::endl << H << std::endl;
    fout << "矩阵b差值为： " << std::endl << b << std::endl;
    fout.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_seprator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // Glog
    FLAGS_colorlogtostderr = true;
    google::SetStderrLogging(google::GLOG_INFO);
    google::SetLogFilenameExtension("log");
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Main program start!";

    // 准备数据
    vector<Frame> cameras;
    vector<Eigen::Vector3d> points;
    GetSimDataInWordFrame(cameras, points); // 得到仿真数据（相机位姿和三维点）
    // matrix();

    vector<Frame> noise_cameras;
    vector<Eigen::Vector3d> noise_points;
    add_camera_pose_noise(cameras, points, noise_cameras);
    // add_point_noise(cameras, points, noise_cameras);
    Eigen::Quaterniond qic(1, 0, 0, 0);
    Eigen::Vector3d tic(0, 0, 0);


    // LOG(INFO) << "Parameter block add started! ! !";
    SLAM_Solver::BA_problem BAProblem;
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    BAProblem.initialStructure(config_file);
    for (int i = 0; i < noise_cameras.size(); ++i) {
        BAProblem.addPoseParameterBlock(noise_cameras[i].twc, noise_cameras[i].qwc, i);
    }
    // std::vector<SLAM_Solver::Pose::Ptr> vec_pose =  BAProblem.get_all_Pose();
    // std::cout << "00000000000000000000000000: " << vec_pose.size() << std::endl;
    LOG(INFO) << "Parameter block added! ! !";
    matrix();

    // std::default_random_engine generator;
    // std::normal_distribution<double> noise_pdf(0., 1.);
    // double noise = 0;
    // int feature_inx = 0;

    // LOG(INFO) << "Residual block add started! ! !";
    // for(int i = 0; i < points.size(); i++) {
    //     Eigen::Vector3d Pw = points[i];
    //     Eigen::Vector3d Pc = noise_cameras[0].Rwc.transpose() * (Pw - noise_cameras[0].twc); // 转到相机坐标
    //     noise = noise_pdf(generator);
    //     double inverse_depth = 1. / (Pc.z()); // 逆深度 + 噪声
    //     BAProblem.addFeatureParameterBlock(inverse_depth, i);
    //     for (int j = 1; j < noise_cameras.size(); ++j) {
    //         Eigen::Vector3d pt_i = noise_cameras[0].featurePerId.find(i)->second; // 起始帧 0帧
    //         Eigen::Vector3d pt_j = noise_cameras[j].featurePerId.find(i)->second; // 第j帧
    //         BAProblem.addFeatureResidualBlock(0, j, i, pt_i, pt_j);
    //         feature_inx++;
    //     }
    // }
    // LOG(INFO) << "Residual block added succedd! ! !";

    // LOG(INFO) << "costF_tests started! ! !";
    // std::vector<SLAM_Solver::costFunction::Ptr> costF_tests = BAProblem.get_all_costFunction();
    // int z = 1;
    // for(auto costF_test : costF_tests) {
    //     // LOG(INFO) << "costF_tests started the   " << i << "   tset !!!";

    //     costF_test->SetTranslationImuFromCamera(qic, tic);
    //     costF_test->ComputeResidual();
    //     z++; 
    // }

    // std::vector<int> vec_test;
    // if(vec_test.size() == 0)
    // {
    //     std::cout << "jjjjjjjjjjjjjjjjjjjjjjj" << std::endl;
    // }

    // BAProblem.test();

    // std::vector<int> poseidx;
    // poseidx = BAProblem.get_all_Poseidx();
    // std::cout << "0000000000000000000000000" << std::endl;
    // for(int i = 0; i < poseidx.size(); i++)
    // {
    //     std::cout << "push出vector中的第" << i << "ge  data: " << poseidx[i] << std::endl;
    // }
       
    // BAProblem.solve();
    // BAProblem.getSolveResults();

    // LOG(INFO) << "真实的位姿轨迹 ! ! !： ";
    // for(int i = 0; i < cameras.size(); i++) {
        
    //     Eigen::Vector3d t1 = cameras[i].twc;
    //     Eigen::Quaterniond q1 = cameras[i].qwc;
    //     std::cout << std::setprecision(20) << i << " " << t1[0] << " " << t1[1] << " " << t1[2] << " "
    //                << q1.x() << " " << q1.y() << " " << q1.z() << " " << q1.w() << std::endl;
    // }

    // LOG(INFO) << "加入噪声的位姿轨迹 ! ! !： ";
    // for(int i = 0; i < noise_cameras.size(); i++) {
        
    //     Eigen::Vector3d t1 = noise_cameras[i].twc;
    //     Eigen::Quaterniond q1 = noise_cameras[i].qwc;
    //     std::cout << std::setprecision(20) << i << " " << t1[0] << " " << t1[1] << " " << t1[2] << " "
    //                << q1.x() << " " << q1.y() << " " << q1.z() << " " << q1.w() << std::endl;
    // }

    // LOG(INFO) << "真实的逆深度 ! ! !： ";
    // for(int i = 0; i < points.size(); i++) {
    //     Eigen::Vector3d Pw = points[i];
    //     Eigen::Vector3d Pc = cameras[0].Rwc.transpose() * (Pw - cameras[0].twc); // 转到相机坐标
    //     double inverse_depth = 1. / (Pc.z()); // 逆深度 + 噪声
    //     std::cout << "第" << i << "点的逆深度为： " << inverse_depth << std::endl;
    // }


    
    // std::cout << "H矩阵为： " << std::endl << H << std::endl;
    // std::cout << "矩阵b为： " << std::endl << b << std::endl;
    
    return 0;
}
