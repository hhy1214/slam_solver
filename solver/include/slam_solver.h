#ifndef SOLVER_SLAM_SOLVER_H
#define SOLVER_SLAM_SOLVER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include <glog/logging.h>
#include <iostream>
#include <vector>
#include "pose.h"
#include "feature.h"
#include <unordered_map>
#include <mutex>
#include "costfunction.h"
#include "parameters.h"
#include "motion.h"
#include "integration_base.h"
#include "costIMUFunction.h"
#include "costoneframetwocamfunction.h"
#include "costTwoFrameTwoCamFunction.h"

namespace SLAM_Solver{
    typedef unsigned long ulong;

class BA_problem
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    enum SolverType { 
        LM = 0,
        PCG = 1,
        DOGLEG = 2,
    };
    void setCameraIntrinsics(std::vector<double>& K, int flag) {
        // 左目
        if(flag == 0) {
            cameraIntrinsics = K;
        }
        else { // 右目
            rightCameraIntrinsics = K;
        }
    }
    void setIsIncremental(bool flag) {
        isIncremental = flag;
    }
    void setSolverType(SolverType type) {
        solverType = type;
    }

// 外部接口
    void initialStructure(std::string config_yaml);
    bool addPoseParameterBlock(Eigen::Vector3d m_p, Eigen::Quaterniond m_q, int ID);               // 添加优化变量,相机位姿
    bool addFeatureParameterBlock(double invDepth, int ID);               // 添加优化变量,相机位姿
    bool addIMUParameterBlock(int ID, Eigen::Vector3d _Vs, Eigen::Vector3d _Ba, Eigen::Vector3d _Bg);
    void addIMUResidualBlock(int lastposeIdx, int curposeIdx, IntegrationBase *_pre_integration);                    // 添加IMU残差块 IntegrationBase *_pre_integration 
    void addFeatureResidualBlock(int start_poseIdx, int cur_poseIdx, int featureIdx, Eigen::Vector3d pti, Eigen::Vector3d ptj);  // 添加视觉残差块
     void addStereoFeatureOneFtwoCResidual(int poseIdx, int featureIdx, Eigen::Vector3d pti, Eigen::Vector3d ptj);  // 添加双目视觉残差块
    void addStereoFeatureTwoFtwoCResidual(int start_poseIdx, int cur_poseIdx, int featureIdx, Eigen::Vector3d pti, Eigen::Vector3d ptj);  // 添加双目视觉残差块
    // void addStereoFeatureResidualBlock(int featureIdx, int numOfMeasure);
    void solve();                                                                 // 优化求解
    void test();


    // void addPriorBlock();                                                         // 添加边缘化信息
    void setMargin(int poseIdx);
    void getSolveResults();                                                       // 获取优化结果

    Pose::Ptr get_Pose(int poseIdx);
    FeatureID::Ptr get_FeatureID(int featureIdx);
    std::vector<int> get_all_Poseidx();
    std::vector<Pose::Ptr> get_all_Pose();
    std::vector<FeatureID::Ptr> get_all_FeatureID();
    std::vector<costFunction::Ptr> get_all_costFunction();

private:

    
    // void computeIMUJacobians();            // 计算 IMU Jacobians 与残差 
    // void cpmputeFeatureJacobians();        // 计算视觉 Jacobians 与残差
    // void updateSchurComplement();          // 跟新 Schur 举证块
    // void solveSchurComplement();           // 求解 Schur 方程，获得相机位姿增量
    // void solveInverseDepth();              // 计算逆深度
    void setOrdering();
    void readParameters(std::string config_file);
    void makeHession();

    void ComputeLambdaInitLM();
    void SolveLinearSystem();
    VecX PCGSolver(const MatXX &A, const VecX &b, int maxIter);
    void UpdateStates();
    bool IsGoodStepInLM();
    void RollbackStates();

    //对应的各种配置参数
    Parameters::Ptr parameters; // 从yaml文件中读取的参数
    Eigen::Vector3d tic0, tic1;
    Eigen::Quaterniond qic0, qic1;
    Eigen::Matrix4d TIC0, TIC1;
    Eigen::Matrix3d RIC0, RIC1;

    
    std::vector<double> cameraIntrinsics, rightCameraIntrinsics;    // 左右目相机内参
    bool isIncremental;                                             // 是否增量化
    int interatorNum;                                               // 迭代次数

    std::vector<int> PoseIdxs;                                      // 对应Pose索引
    std::unordered_map<int, Pose::Ptr> m_Poses;                     // 所包含的Pose优化量
    std::vector<int> MotionIdxs;                                    // 对应Motion索引
    std::unordered_map<int, Motion::Ptr> m_Motions;                 // 所包含的Motion优化量
    std::vector<int> FeatureIDIdxs;                                 // 对应feature索引
    std::unordered_map<int, FeatureID::Ptr> m_FeatureIDs;           // 所包含的feature优化量

    std::unordered_multimap<int, costFunction::Ptr> HashPoseIdTocostFunction;
    std::vector<costFunction::Ptr> m_costFunctions;                 // 残差项，对应视觉残差

    std::unordered_multimap<int, costOneFrameTwoCamFunction::Ptr> HashPoseIdTocostOneFrameTwoCamFunction;
    std::vector<costOneFrameTwoCamFunction::Ptr> m_costOneFrameTwoCamFunctions;                 // 残差项，对应视觉残差

    std::unordered_multimap<int, costTwoFrameTwoCamFunction::Ptr> HashPoseIdTocostTwoFrameTwoCamFunction;
    std::vector<costTwoFrameTwoCamFunction::Ptr> m_costTwoFrameTwoCamFunctions;                 // 残差项，对应视觉残差

    std::unordered_multimap<int, costIMUFunction::Ptr> HashPoseIdTocostIMUFunction;
    std::vector<costIMUFunction::Ptr> m_costIMUFunctions;                 // 残差项，对应IMU残差
 
    SolverType solverType;

    std::mutex mutex_map;

    /// 整个信息矩阵
    MatXX Hessian_;
    VecX b_;
    VecX delta_x_;

    double currentLambda_;
    double currentChi_;
    double stopThresholdLM_;    // LM 迭代退出阈值条件
    double ni_;                 //控制 Lambda 缩放大小

    /// 先验部分信息
    MatXX H_prior_;
    VecX b_prior_;
    VecX b_prior_backup_;
    VecX err_prior_backup_;

    MatXX Jt_prior_inv_;
    VecX err_prior_;

    /// SBA的Pose部分
    MatXX H_pp_schur_;
    VecX b_pp_schur_;
    // Heesian 的 Landmark 和 pose 部分
    MatXX H_pp_;
    VecX b_pp_;
    MatXX H_ll_;
    VecX b_ll_;

    /// Ordering related
    ulong ordering_poses_ = 0;
    ulong ordering_landmarks_ = 0;
    ulong ordering_generic_ = 0;
};

}




#endif