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

namespace SLAM_Solver{

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
    // 一次性设置多个参数
    void setParameters() {

    }

// 外部接口
    void initialStructure();
    bool addPoseParameterBlock(Eigen::Vector3d m_p, Eigen::Quaterniond m_q, int ID);               // 添加优化变量,相机位姿
    bool addFeatureParameterBlock(double invDepth, int ID);               // 添加优化变量,相机位姿
    // void addIMUResidualBlock(int lastposeIdx, int curposeIdx);                    // 添加IMU残差块 IntegrationBase *_pre_integration 
    void addFeatureResidualBlock(int start_poseIdx, int cur_poseIdx, int featureIdx, Eigen::Vector3d pti, Eigen::Vector3d ptj);  // 添加视觉残差块
    // void addStereoFeatureResidualBlock(int featureIdx, int numOfMeasure);
    void solve();                                                                 // 优化求解


    // void addPriorBlock();                                                         // 添加边缘化信息
    // void setMargin(int poseIdx);
    // void getSolveResults();                                                       // 获取优化结果

    Pose::Ptr get_Pose(int poseIdx);
    FeatureID::Ptr get_FeatureID(int featureIdx);
    std::vector<costFunction::Ptr> get_all_costFunction();

private:

    
    // void computeIMUJacobians();            // 计算 IMU Jacobians 与残差 
    // void cpmputeFeatureJacobians();        // 计算视觉 Jacobians 与残差
    // void updateSchurComplement();          // 跟新 Schur 举证块
    // void solveSchurComplement();           // 求解 Schur 方程，获得相机位姿增量
    // void solveInverseDepth();              // 计算逆深度
    void makeHession();
    

    

    std::vector<double> cameraIntrinsics, rightCameraIntrinsics;    // 左右目相机内参
    bool isIncremental;                                             // 是否增量化
    int interatorNum;                                               // 迭代次数

    std::vector<int> PoseIdxs;                                      // 对应Pose索引
    std::unordered_map<int, Pose::Ptr> m_Poses;                     // 所包含的Pose优化量
    std::vector<int> FeatureIDIdxs;                                 // 对应feature索引
    std::unordered_map<int, FeatureID::Ptr> m_FeatureIDs;           // 所包含的feature优化量
    std::vector<costFunction::Ptr> m_costFunctions;                 // 所包含的feature优化量
    SolverType solverType;

    std::mutex mutex_map;

    /// 整个信息矩阵
    MatXX Hessian_;
    VecX b_;
    VecX delta_x_;
};

}




#endif