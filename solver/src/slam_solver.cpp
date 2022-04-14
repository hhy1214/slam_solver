#include "slam_solver.h"

using namespace SLAM_Solver;

bool BA_problem::addPoseParameterBlock(Eigen::Vector3d m_p, Eigen::Quaterniond m_q, int ID)
{
    std::lock_guard<std::mutex> lock(mutex_map);
    Pose::Ptr pose_ptr(new SLAM_Solver::Pose(ID));
    pose_ptr->set_T(m_q, m_p);
    int poseID = pose_ptr->get_ID();
    auto res = m_Poses.insert(std::make_pair(ID, pose_ptr));
    return res.second;    
}

bool BA_problem::addFeatureParameterBlock(double invDepth, int ID) {
    std::lock_guard<std::mutex> lock(mutex_map);
    FeatureID::Ptr feature_ptr(new SLAM_Solver::FeatureID(ID));
    feature_ptr->set_invdep(invDepth);
    int featureID = feature_ptr->get_ID();
    auto res = m_FeatureIDs.insert(std::make_pair(ID, feature_ptr));
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