#include "slam_solver.h"

using namespace SLAM_Solver;

bool BA_problem::addPoseParameterBlock(Eigen::Vector3d m_p, Eigen::Quaterniond m_q, int ID)
{
    std::lock_guard<std::mutex> lock(mutex_map);
    Pose::Ptr pose_ptr(new SLAM_Solver::Pose(ID));
    pose_ptr->set_T(m_q, m_p);
    int poseID = pose_ptr->get_ID();
    auto res = m_Poses.insert(std::make_pair(ID, pose_ptr));
    if(res.second) {
        PoseIdxs.push_back(ID);
    }
    return res.second;    
}

bool BA_problem::addFeatureParameterBlock(double invDepth, int ID) {
    std::lock_guard<std::mutex> lock(mutex_map);
    FeatureID::Ptr feature_ptr(new SLAM_Solver::FeatureID(ID));
    feature_ptr->set_invdep(invDepth);
    int featureID = feature_ptr->get_ID();
    auto res = m_FeatureIDs.insert(std::make_pair(ID, feature_ptr));
    if(res.second) {
        FeatureIDIdxs.push_back(ID);
    }
    return res.second; 
}

// pti 与 ptj 为归一化平面坐标，它与像素坐标之间存在相机内参与畸变系数   pti在起始帧上 而ptj为当前帧的观测
void BA_problem::addFeatureResidualBlock(int start_poseIdx, int cur_poseIdx, int featureIdx, Eigen::Vector3d pti, Eigen::Vector3d ptj) {
    std::lock_guard<std::mutex> lock(mutex_map);
    costFunction::Ptr costFuti(new SLAM_Solver::costFunction(pti, ptj));
    LOG(INFO) << "addFeatureResidualBlock started!!!!!";

    //将点的观测进行构造，即对FeatureID::Ptr中的FeatureMeasure进行构造
    auto it = m_FeatureIDs.find(featureIdx);
    auto featureid_ = it->second;
    // auto featureid_ = this->get_FeatureID(featureIdx);
    featureid_->set_startframe(start_poseIdx);
    featureid_->addFeatureMeasure(start_poseIdx, pti);
    featureid_->addFeatureMeasure(cur_poseIdx, ptj);
    LOG(INFO) << "featureid_ succeeded!!!!!";

    //将观测投入到costFunction类中，将残差进行构建并保存
    auto it1 = m_Poses.find(start_poseIdx);
    auto it2 = m_Poses.find(cur_poseIdx);
    auto posei_ = it1->second;
    auto posej_ = it2->second;
    costFuti->set_startPose(posei_);
    costFuti->set_curPose(posej_);
    costFuti->set_curFeature(featureid_);
    LOG(INFO) << "posei_ posej_ succeeded!!!!!";

    m_costFunctions.push_back(costFuti);
    // std::cout << "m_costFunctions.size(): " << m_costFunctions.size();

}

void BA_problem::initialStructure() {
    m_Poses.clear();
    m_FeatureIDs.clear();
    LOG(INFO) << "BA_problem Initialization succeeded!!!!!";
}

Pose::Ptr BA_problem::get_Pose(int poseIdx) {
    std::lock_guard<std::mutex> lock(mutex_map);
    auto it = m_Poses.find(poseIdx);
    if (it != m_Poses.end()) {
        return it->second;
    }
    return static_cast<Pose::Ptr>(nullptr);
}

FeatureID::Ptr BA_problem::get_FeatureID(int featureIdx) {
    std::lock_guard<std::mutex> lock(mutex_map);
    auto it = m_FeatureIDs.find(featureIdx);
    if (it != m_FeatureIDs.end()) {
        return it->second;
    }
    return static_cast<FeatureID::Ptr>(nullptr);
}

std::vector<costFunction::Ptr> BA_problem::get_all_costFunction() {
    return m_costFunctions;
}

void BA_problem::solve() {
    makeHession();
}

void BA_problem::makeHession() {
    int size, pose_size, feature_size;
    pose_size = m_Poses.size();
    feature_size = m_FeatureIDs.size();
    size = pose_size * 6 + feature_size;
    int H_pose_size = pose_size * 6;

    // 直接构造大的 H 矩阵
    MatXX H(MatXX::Zero(size, size));
    VecX b(VecX::Zero(size));
    std::cout << H.cols() << std::endl;
    LOG(INFO) << "信息矩阵的列: " << H.cols() << "以及行： " << H.rows();
    LOG(INFO) << "残差的列: " << b.cols() << "以及行： " << b.rows();

    for(auto m_costFunction : m_costFunctions) {
        SLAM_Solver::Pose::Ptr start_cost_pose = m_costFunction->get_startPose();
        SLAM_Solver::Pose::Ptr cur_cost_pose = m_costFunction->get_curPose();
        SLAM_Solver::FeatureID::Ptr cur_cost_feature = m_costFunction->get_curFeature();
        m_costFunction->ComputeResidual();
        auto jacobians = m_costFunction->Jacobians();
        auto residual = m_costFunction->Residual();

        int start_frameIdx = start_cost_pose->get_ID();
        int cur_frameIdx = cur_cost_pose->get_ID();
        int cur_featureIdx = cur_cost_feature->get_ID();

        int index_start = find(PoseIdxs.begin(), PoseIdxs.end(), start_frameIdx) - PoseIdxs.begin();
        int index_cur = find(PoseIdxs.begin(), PoseIdxs.end(), cur_frameIdx) - PoseIdxs.begin();
        int index_feature = find(FeatureIDIdxs.begin(), FeatureIDIdxs.end(), cur_featureIdx) - FeatureIDIdxs.begin();
        // std::cout << "构造残差创建的每一个帧索引 index_start: " << index_start << " index_cur: " << index_cur << std::endl;
        // std::cout << "构造残差创建的每一个图像索引 index_feature: " << index_feature << std::endl;

        int index_H_posei = index_start * 6;
        int index_H_posej = index_cur * 6;
        int index_H_feature = H_pose_size + index_feature;
        std::vector<int> index_H;
        index_H.push_back(index_H_feature);
        index_H.push_back(index_H_posei);
        index_H.push_back(index_H_posej);
        std::vector<int> index_dim;
        index_dim.push_back(1);
        index_dim.push_back(6);
        index_dim.push_back(6);
        for(int i = 0; i < jacobians.size(); i++) {
            auto jacobians_i = jacobians[i];
            int index_i = index_H[i];
            int index_dim_i = index_dim[i];
            for(int j = i; j < jacobians.size(); j++) {
                auto jacobians_j = jacobians[j];
                int index_j = index_H[j];
                int index_dim_j = index_dim[j]; 
                MatXX hessian = jacobians_i.transpose() * jacobians_j;
                // std::cout << "构建的信息矩阵 i: " << i << " j: " << j << hessian << std::endl;
                H.block(index_i, index_j, index_dim_i, index_dim_j).noalias() += hessian;
                if(j != i) {
                    H.block(index_j, index_i, index_dim_j, index_dim_i).noalias() += hessian.transpose();
                }
            }

            b.segment(index_i, index_dim_i).noalias() -= jacobians_i.transpose() * residual;
        }

    }

    // Hessian_ = H;
    std::cout << "ceres所构建的H is :" << std::endl << H << std::endl;
    // b_ = b;
    std::cout << "ceres所构建的b is :" << std::endl << b << std::endl;

    LOG(INFO) << "BA_problem make Hession succeeded!!!!!";

}