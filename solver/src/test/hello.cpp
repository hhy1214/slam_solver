#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <opencv2/core/eigen.hpp>
#include "pose.h"
#include "slam_solver.h"
#include "costfunction.h"



using namespace std;
Eigen::Matrix4d T_I_C;

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

    //起始帧
    int ID = 0;
    SLAM_Solver::Pose::Ptr posei(new SLAM_Solver::Pose(ID));
    Eigen::Quaterniond q_i(1, 0, 0, 0);
    Eigen::Vector3d t_i(0.0, 0.0, 0.0);
    posei->set_T(q_i, t_i);

    //第一帧
    // int ID1 = 1;
    // SLAM_Solver::Pose::Ptr posej(new SLAM_Solver::Pose(ID));
    // Eigen::Quaterniond q_j(0.92388, 0.0, 0.0, 0.382683);
    // Eigen::Vector3d t_j(-2.34315, 5.65685, 1);
    // posei->set_T(q_j, t_j);
    int ID1 = 1;
    SLAM_Solver::Pose::Ptr posej(new SLAM_Solver::Pose(ID1));
    Eigen::Quaterniond q_j(0.965926, 0.0, 0.0, 0.258819);
    Eigen::Vector3d t_j(-1.0718, 4, 0.866025);
    posej->set_T(q_j, t_j);

    //第二帧
    int ID2 = 2;
    SLAM_Solver::Pose::Ptr posez(new SLAM_Solver::Pose(ID2));
    Eigen::Quaterniond q_z(0.866025, 0.0, 0.0, 0.5);
    Eigen::Vector3d t_z(-4, 6.9282, 0.866025);
    posez->set_T(q_z, t_z);

    //feature
    int ID_f = 1;
    double depinv = 0.227057;

    Eigen::Vector3d test1(-0.496282, -0.072897, 1);
    Eigen::Vector3d test2(-0.87053, -0.863969, 1);
    Eigen::Vector3d test3(-1.4785, -1.40608, 1);

    Eigen::Quaterniond qic(1, 0, 0, 0);
    Eigen::Vector3d tic(0, 0, 0);

    LOG(INFO) << "Parameter block add started! ! !";
    SLAM_Solver::BA_problem BAProblem;
    BAProblem.initialStructure();
    BAProblem.addPoseParameterBlock(t_i, q_i, ID);
    BAProblem.addFeatureParameterBlock(depinv, ID_f);
    BAProblem.addPoseParameterBlock(t_j, q_j, ID1);
    BAProblem.addPoseParameterBlock(t_z, q_z, ID2);
    LOG(INFO) << "Parameter block added! ! !";

    LOG(INFO) << "Residual block add started! ! !";
    BAProblem.addFeatureResidualBlock(ID, ID1, ID_f, test1, test2);
    BAProblem.addFeatureResidualBlock(ID, ID2, ID_f, test1, test3);
    LOG(INFO) << "Residual block added! ! !";

    SLAM_Solver::Pose::Ptr pose_test = BAProblem.get_Pose(ID);
    SLAM_Solver::FeatureID::Ptr feature_test = BAProblem.get_FeatureID(ID_f);
    SLAM_Solver::Pose::Ptr pose_cur = BAProblem.get_Pose(ID1);

    LOG(INFO) << "costF_tests started! ! !";
    std::vector<SLAM_Solver::costFunction::Ptr> costF_tests = BAProblem.get_all_costFunction();
    int i = 1;
    for(auto costF_test : costF_tests) {
        LOG(INFO) << "costF_tests started the   " << i << "   tset !!!";
        SLAM_Solver::Pose::Ptr cue_cost_pose = costF_test->get_curFeature();
        costF_test->SetTranslationImuFromCamera(qic, tic);
        costF_test->ComputeResidual();
        i++; 
    }
       
    
    
    return 0;
}

    // std::vector<double> a;
    // a.push_back(q_i.w());
    // a.push_back(q_i.x());
    // a.push_back(q_i.y());
    // a.push_back(q_i.z());
    // a.push_back(t_i[0]);
    // a.push_back(t_i[1]);
    // a.push_back(t_i[2]);

    // for(int i = 0; i < a.size(); i++) {
    //     std::cout << "111: " << a[i] << std::endl; 
    // }
    // Eigen::Matrix4d cur_cost_T_pose = cue_cost_pose->get_T();
    // std::cout << "cur_cost_T_pose: " << cur_cost_T_pose << std::endl;
