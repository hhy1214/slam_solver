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
    // LOG(INFO) << "addFeatureResidualBlock started!!!!!";

    //将点的观测进行构造，即对FeatureID::Ptr中的FeatureMeasure进行构造
    auto it = m_FeatureIDs.find(featureIdx);
    auto featureid_ = it->second;
    // auto featureid_ = this->get_FeatureID(featureIdx);
    featureid_->set_startframe(start_poseIdx);
    featureid_->addFeatureMeasure(start_poseIdx, pti);
    featureid_->addFeatureMeasure(cur_poseIdx, ptj);
    // LOG(INFO) << "featureid_ succeeded!!!!!";

    //将观测投入到costFunction类中，将残差进行构建并保存
    auto it1 = m_Poses.find(start_poseIdx);
    auto it2 = m_Poses.find(cur_poseIdx);
    auto posei_ = it1->second;
    auto posej_ = it2->second;
    costFuti->set_startPose(posei_);
    costFuti->set_curPose(posej_);
    costFuti->set_curFeature(featureid_);
    // LOG(INFO) << "posei_ posej_ succeeded!!!!!";

    m_costFunctions.push_back(costFuti);
    // std::cout << "m_costFunctions.size(): " << m_costFunctions.size();

}

void BA_problem::initialStructure(std::string config_yaml) {
    LOG(INFO) << "BA_problem Initialization started!!!!!";
    parameters::Ptr parameters_ptr(new SLAM_Solver::parameters());
    parameters_ptr->set_parameters(config_yaml);
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

    ComputeLambdaInitLM();

    bool stop = false;
    int iter = 0;
    int iterations = 15;
    double last_chi_ = 1e20;
    while (!stop && (iter < iterations)) {
        LOG(INFO) << "目前迭代的次数: " << iter << "对应的currentChi_： " << currentChi_ << "currentLambda_: " << currentLambda_ ;

        bool oneStepSuccess = false;
        int false_cnt = 0;
        while(!oneStepSuccess) {

            SolveLinearSystem();

            // 优化退出条件1： delta_x_ 很小则退出
            if (delta_x_.squaredNorm() <= 1e-6 || false_cnt > 10) {
                stop = true;
                break;
            }
            // 更新状态量
            UpdateStates();
            // 判断当前步是否可行以及 LM 的 lambda 怎么更新
            oneStepSuccess = IsGoodStepInLM();
            // 后续处理，
            if (oneStepSuccess) {
                // 在新线性化点 构建 hessian
                makeHession();
                // TODO:: 这个判断条件可以丢掉，条件 b_max <= 1e-12 很难达到，这里的阈值条件不应该用绝对值，而是相对值
//                double b_max = 0.0;
//                for (int i = 0; i < b_.size(); ++i) {
//                    b_max = max(fabs(b_(i)), b_max);
//                }
//                // 优化退出条件2： 如果残差 b_max 已经很小了，那就退出
//                stop = (b_max <= 1e-12);
                false_cnt = 0;
            } else {
                false_cnt ++;
                RollbackStates();   // 误差没下降，回滚
            }
            // oneStepSuccess = true;
        }
        iter++;

        if(sqrt(currentChi_) <= stopThresholdLM_)
        {
            std::cout << "sqrt(currentChi_) <= stopThresholdLM_" << std::endl;
            stop = true;
        }
        last_chi_ = currentChi_;
    }
}

void BA_problem::RollbackStates() {
    for (auto m_Pose: m_Poses) {
        int start_frameIdx = m_Pose.second->get_ID();
        int index_start = find(PoseIdxs.begin(), PoseIdxs.end(), start_frameIdx) - PoseIdxs.begin();
        ulong idx = index_start * 6;
        ulong dim = 6;
        VecX delta = delta_x_.segment(idx, dim);
        m_Pose.second->Plus(-delta);
    } for(auto m_FeatureID : m_FeatureIDs) {
        int start_featureIdx = m_Poses.size() * 6;
        int start_feaIdx = m_FeatureID.second->get_ID();
        int index_feature = find(FeatureIDIdxs.begin(), FeatureIDIdxs.end(), start_feaIdx) - FeatureIDIdxs.begin();
        index_feature += start_featureIdx;
        ulong dim = 1;
        VecX delta = delta_x_.segment(index_feature, dim);
        m_FeatureID.second->Plus(-delta);
    }
}

bool BA_problem::IsGoodStepInLM() {
    double scale = 0;
    scale = 0.5 * delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
    scale += 1e-6;    // make sure it's non-zero :)

    // recompute residuals after update state
    // TODO:: get robustChi2() instead of Chi2()
    double tempChi = 0.0;
    for (auto m_costFunction : m_costFunctions) {
        m_costFunction->ComputeResidual();
        tempChi += m_costFunction->Chi2();
    }

    double rho = (currentChi_ - tempChi) / scale;
    if (rho > 0 && std::isfinite(tempChi))   // last step was good, 误差在下降
    {
        double alpha = 1. - pow((2 * rho - 1), 3);
        alpha = std::min(alpha, 2. / 3.);
        double scaleFactor = (std::max)(1. / 3., alpha);
        currentLambda_ *= scaleFactor;
        ni_ = 2;
        currentChi_ = tempChi;
        return true;
    } else {
        currentLambda_ *= ni_;
        ni_ *= 2;
        return false;
    }
}

void BA_problem::UpdateStates() {
    for (auto m_Pose: m_Poses) {
        int start_frameIdx = m_Pose.second->get_ID();
        int index_start = find(PoseIdxs.begin(), PoseIdxs.end(), start_frameIdx) - PoseIdxs.begin();
        ulong idx = index_start * 6;
        ulong dim = 6;
        VecX delta = delta_x_.segment(idx, dim);
        m_Pose.second->Plus(delta);
    } for(auto m_FeatureID : m_FeatureIDs) {
        int start_featureIdx = m_Poses.size() * 6;
        int start_feaIdx = m_FeatureID.second->get_ID();
        int index_feature = find(FeatureIDIdxs.begin(), FeatureIDIdxs.end(), start_feaIdx) - FeatureIDIdxs.begin();
        index_feature += start_featureIdx;
        ulong dim = 1;
        VecX delta = delta_x_.segment(index_feature, dim);
        m_FeatureID.second->Plus(delta);
    }
}

void BA_problem::SolveLinearSystem() {
    int reserve_size, marg_size;
    reserve_size = m_Poses.size() * 6;
    marg_size = m_FeatureIDs.size() * 1;

    // TODO:: home work. 完成矩阵块取值，Hmm，Hpm，Hmp，bpp，bmm  //>>??
    MatXX Hmm = Hessian_.block(reserve_size, reserve_size, marg_size, marg_size); // 对应landmark
    MatXX Hpm = Hessian_.block(0, reserve_size, reserve_size, marg_size);
    MatXX Hmp = Hessian_.block(reserve_size, 0, marg_size, reserve_size);
    VecX bpp = b_.segment(0, reserve_size);
    VecX bmm = b_.segment(reserve_size, marg_size);

    // Hmm 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
    MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
    int size = 1;
    int idx = 0;
    while(idx < marg_size) {
        Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
        idx += size;
    }
    // LOG(INFO) << "Hmm_inv: " << Hmm_inv ;
    // TODO:: home work. 完成舒尔补 Hpp, bpp 代码
    MatXX tempH = Hpm * Hmm_inv;
    H_pp_schur_ = Hessian_.block(0, 0, reserve_size, reserve_size) - tempH * Hmp;  //??
    b_pp_schur_ = bpp - tempH * bmm; // 边际概率

    // step2: solve Hpp * delta_x = bpp
    VecX delta_x_pp(VecX::Zero(reserve_size));
    // PCG Solver
    for (ulong i = 0; i < reserve_size; ++i) {
        H_pp_schur_(i, i) += currentLambda_;
    }

    // int n = H_pp_schur_.rows() * 2;                       // 迭代次数
    // delta_x_pp = PCGSolver(H_pp_schur_, b_pp_schur_, n);  // pcg 求解
    delta_x_pp =  H_pp_schur_.ldlt().solve(b_pp_schur_);
    delta_x_.head(reserve_size) = delta_x_pp;
    // LOG(INFO) << " PCG 求解完成 ！！！" ;
    // std::cout << "delta_x_: " << delta_x_.transpose() << std::endl;  // ??? 结果相差
    // std::cout << "delta_x_pp: " << delta_x_pp.transpose() << std::endl;  // ??? 结果相差
    
    // TODO:: home work. step3: solve landmark
    VecX delta_x_ll(marg_size);
    delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp);  //?
    delta_x_.tail(marg_size) = delta_x_ll;
    // std::cout << "delta_x_: " << delta_x_.transpose() << std::endl;  // ??? 结果相差
    // delta_x_ll: -0.0058278 -0.0335995
}

VecX BA_problem::PCGSolver(const MatXX &A, const VecX &b, int maxIter) {
     assert(A.rows() == A.cols() && "PCG solver ERROR: A is not a square matrix");
    int rows = b.rows();
    int n = maxIter < 0 ? rows : maxIter;
    VecX x(VecX::Zero(rows));
    MatXX M_inv = A.diagonal().asDiagonal().inverse();
    VecX r0(b);  // initial r = b - A*0 = b
    VecX z0 = M_inv * r0;
    VecX p(z0);
    VecX w = A * p;
    double r0z0 = r0.dot(z0);
    double alpha = r0z0 / p.dot(w);
    VecX r1 = r0 - alpha * w;
    int i = 0;
    double threshold = 1e-6 * r0.norm();
    while (r1.norm() > threshold && i < n) {
        i++;
        VecX z1 = M_inv * r1;
        double r1z1 = r1.dot(z1);
        double belta = r1z1 / r0z0;
        z0 = z1;
        r0z0 = r1z1;
        r0 = r1;
        p = belta * p + z1;
        w = A * p;
        alpha = r1z1 / p.dot(w);
        x += alpha * p;
        r1 -= alpha * w;
    }
    return x;
}

void BA_problem::ComputeLambdaInitLM() {
    ni_ = 2.;
    currentLambda_ = -1.;
    currentChi_ = 0.0;
    // TODO:: robust cost chi2
    for (auto m_costFunction : m_costFunctions) {
        currentChi_ += m_costFunction->Chi2();
    }
    LOG(INFO) << "currentChi_: " << currentChi_ ;

    stopThresholdLM_ = 1e-6 * currentChi_;          // 迭代条件为 误差下降 1e-6 倍

    double maxDiagonal = 0;
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (ulong i = 0; i < size; ++i) {
        maxDiagonal = std::max(fabs(Hessian_(i, i)), maxDiagonal);
    }
    double tau = 1e-5;
    currentLambda_ = tau * maxDiagonal;
    LOG(INFO) << "currentLambda_: " << currentLambda_ ;
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

    Hessian_ = H;
    // std::cout << "ceres所构建的H is :" << std::endl << Hessian_ << std::endl;
    b_ = b;
    // std::cout << "ceres所构建的b is :" << std::endl << b_ << std::endl;

    LOG(INFO) << "BA_problem make Hession succeeded!!!!!";
    delta_x_ = VecX::Zero(size);  // initial delta_x = 0_n;

}

void BA_problem::getSolveResults() {
    std::vector<Pose::Ptr> res;
    for (auto pose : m_Poses){
        res.push_back(pose.second);
    }
    res.shrink_to_fit();
    std::sort(res.begin(), res.end(), [](Pose::Ptr ptr1, Pose::Ptr ptr2) {
        return ptr1->m_frame_id < ptr2->m_frame_id;
    }); 
    for (auto m_Pose: res) {
        int start_frameIdx = m_Pose->get_ID();
        Eigen::Quaterniond q1 = m_Pose->get_rotate();
        Eigen::Vector3d t1 = m_Pose->get_translation();
        std::cout << std::setprecision(20) << start_frameIdx << " " << t1[0] << " " << t1[1] << " " << t1[2] << " "
                   << q1.x() << " " << q1.y() << " " << q1.z() << " " << q1.w() << std::endl;
        // int start_frameIdx = m_Pose.second->get_ID();
        // Eigen::Quaterniond q1 = m_Pose->get_rotate();
        // Eigen::Vector3d t1 = m_Pose->get_translation();
        // std::cout << std::setprecision(20) << start_frameIdx << " " << t1[0] << " " << t1[1] << " " << t1[2] << " "
        //            << q1.x() << " " << q1.y() << " " << q1.z() << " " << q1.w() << std::endl;
        // std::cout << "优化第 " << start_frameIdx << "帧的位姿R为： " << m_Q.x() << "y: " << m_Q.y() << "z: " << m_Q.z() << "w: " << m_Q.w() << std::endl;
    } 

    // std::vector<FeatureID::Ptr> fea_res;
    // for (auto FeatureID : m_FeatureIDs){
    //     fea_res.push_back(FeatureID.second);
    // }
    // fea_res.shrink_to_fit();
    // std::sort(fea_res.begin(), fea_res.end(), [](FeatureID::Ptr ptr1, FeatureID::Ptr ptr2) {
    //     return ptr1->featureId < ptr2->featureId;
    // });
    for(auto m_FeatureID : m_FeatureIDs) {
        int start_feaIdx = m_FeatureID.second->get_ID();
        double m_faeture_p = m_FeatureID.second->get_invdep();
        // std::cout << "优化第 " << start_feaIdx << "逆深度的值为： " << m_faeture_p << std::endl;
    }
}

void BA_problem::readParameters(std::string config_file) {
    
}

std::vector<Pose::Ptr> BA_problem::get_all_Pose() {
    std::lock_guard<std::mutex> lock(mutex_map);
    std::vector<Pose::Ptr> res;
    for (auto pose : m_Poses){
        res.push_back(pose.second);
    }
    res.shrink_to_fit();
    std::sort(res.begin(), res.end(), [](Pose::Ptr ptr1, Pose::Ptr ptr2) {
        return ptr1->m_frame_id < ptr2->m_frame_id;
    });

    return res;
}

std::vector<FeatureID::Ptr> BA_problem::get_all_FeatureID() {
    std::lock_guard<std::mutex> lock(mutex_map);
    std::vector<FeatureID::Ptr> res;
    for (auto FeatureID : m_FeatureIDs){
        res.push_back(FeatureID.second);
    }
    res.shrink_to_fit();
    std::sort(res.begin(), res.end(), [](FeatureID::Ptr ptr1, FeatureID::Ptr ptr2) {
        return ptr1->featureId < ptr2->featureId;
    });

    return res;
}