#include "slam_solver.h"

using namespace SLAM_Solver;

bool BA_problem::addPoseParameterBlock(Eigen::Vector3d m_p, Eigen::Quaterniond m_q, int ID)
{
    std::lock_guard<std::mutex> lock(mutex_map);
    Pose::Ptr pose_ptr(new SLAM_Solver::Pose(ID));
    pose_ptr->set_T(m_q, m_p);
    int poseID = pose_ptr->get_ID();
    auto res = m_Poses.insert(std::make_pair(ID, pose_ptr));
    if (res.second)
    {
        PoseIdxs.push_back(ID);
    }
    return res.second;
}

bool BA_problem::addFeatureParameterBlock(double invDepth, int ID)
{
    std::lock_guard<std::mutex> lock(mutex_map);
    FeatureID::Ptr feature_ptr(new SLAM_Solver::FeatureID(ID));
    feature_ptr->set_invdep(invDepth);
    int featureID = feature_ptr->get_ID();
    auto res = m_FeatureIDs.insert(std::make_pair(ID, feature_ptr));
    if (res.second)
    {
        FeatureIDIdxs.push_back(ID);
    }
    return res.second;
}

//@@@IMu的每一个id与pose的ID一一对应
bool BA_problem::addIMUParameterBlock(int ID, Eigen::Vector3d _Vs, Eigen::Vector3d _Ba, Eigen::Vector3d _Bg)
{
    std::lock_guard<std::mutex> lock(mutex_map);
    Motion::Ptr Motion_ptr(new SLAM_Solver::Motion(ID));
    Motion_ptr->set_Motion(_Vs, _Ba, _Bg);
    int featureID = Motion_ptr->get_ID();
    auto res = m_Motions.insert(std::make_pair(ID, Motion_ptr));
    if (res.second)
    {
        MotionIdxs.push_back(ID);
    }
    return res.second;
}

void BA_problem::addStereoFeatureoneFtwoCResidual(int poseIdx, int featureIdx, Eigen::Vector3d pti, Eigen::Vector3d ptj)
{
    if (parameters->STEREO)
    {
        std::lock_guard<std::mutex> lock(mutex_map);
        costOneFrameTwoCamFunction::Ptr costFuti(new SLAM_Solver::costOneFrameTwoCamFunction(pti, ptj));

        ///@@@此处需注意，原本的观测为单目观测
        ///@@@由于此时为双目观测，需要将其中featureID中出现的双目观测代码进行
        ///@@@修改，同时此地的添加观测信息也需要进行修改
        ///@@@此地未添加观测信息，需要之后自行添加

        auto it1 = m_Poses.find(poseIdx);
        auto posei_ = it1->second;
        int pose_id = posei_->get_ID();
        auto it2 = m_FeatureIDs.find(featureIdx);
        auto featurei_ = it2->second;
        costFuti->set_curPose(posei_);
        costFuti->set_curFeature(featurei_);

        //////@@@@@@@@@@@添加外参

        m_costOneFrameTwoCamFunctions.push_back(costFuti);
    }
    else
    {
        std::cout << "设定为单目，试图加入双目！！！！！！！！！！！" << std::endl;
    }
}

// pti 与 ptj 为归一化平面坐标，它与像素坐标之间存在相机内参与畸变系数   pti在起始帧上 而ptj为当前帧的观测
void BA_problem::addFeatureResidualBlock(int start_poseIdx, int cur_poseIdx, int featureIdx, Eigen::Vector3d pti, Eigen::Vector3d ptj)
{
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

    //将pose的对应costFuti加入到对应的hash表中
    HashPoseIdTocostFunction.insert(std::pair<int, costFunction::Ptr>(start_poseIdx, costFuti));
    HashPoseIdTocostFunction.insert(std::pair<int, costFunction::Ptr>(cur_poseIdx, costFuti));
}

void BA_problem::addStereoFeaturetwoFtwoCResidual(int start_poseIdx, int cur_poseIdx, int featureIdx, Eigen::Vector3d pti, Eigen::Vector3d ptj)
{
    if (parameters->STEREO)
    {
        std::lock_guard<std::mutex> lock(mutex_map);
        costTwoFrameTwoCamFunction::Ptr costFuti(new SLAM_Solver::costTwoFrameTwoCamFunction(pti, ptj));
        // LOG(INFO) << "addFeatureResidualBlock started!!!!!";

        //将点的观测进行构造，即对FeatureID::Ptr中的FeatureMeasure进行构造
        auto it = m_FeatureIDs.find(featureIdx);
        auto featureid_ = it->second;

        ///@@@此处需注意，原本的观测为单目观测
        ///@@@由于此时为双目观测，需要将其中featureID中出现的双目观测代码进行
        ///@@@修改，同时此地的添加观测信息也需要进行修改
        // featureid_->set_startframe(start_poseIdx);
        // featureid_->addFeatureMeasure(start_poseIdx, pti);
        // featureid_->addFeatureMeasure(cur_poseIdx, ptj);
        // LOG(INFO) << "featureid_ succeeded!!!!!";

        //将观测投入到costFunction类中，将残差进行构建并保存
        auto it1 = m_Poses.find(start_poseIdx);
        auto it2 = m_Poses.find(cur_poseIdx);
        auto posei_ = it1->second;
        auto posej_ = it2->second;
        costFuti->set_startPose(posei_);
        costFuti->set_curPose(posej_);
        costFuti->set_curFeature(featureid_);
        ///@@@此处需注意
        ///@@@此时此地的添加左右目信息也需要进行修改
        ///@@@costFuti->SetLeftTranslationImuFromCamera（）???

        HashPoseIdTocostTwoFrameTwoCamFunction.insert(std::pair<int, costTwoFrameTwoCamFunction::Ptr>(start_poseIdx, costFuti));
        HashPoseIdTocostTwoFrameTwoCamFunction.insert(std::pair<int, costTwoFrameTwoCamFunction::Ptr>(cur_poseIdx, costFuti));

        // LOG(INFO) << "posei_ posej_ succeeded!!!!!";
    }
    else
    {
        std::cout << "设定为单目，试图加入双目！！！！！！！！！！！" << std::endl;
    }
}

void BA_problem::addIMUResidualBlock(int lastposeIdx, int curposeIdx, IntegrationBase *_pre_integration)
{
    if (parameters->USE_IMU)
    {
        std::lock_guard<std::mutex> lock(mutex_map);
        SLAM_Solver::costIMUFunction::Ptr edgeIMU(new SLAM_Solver::costIMUFunction(_pre_integration));
        // LOG(INFO) << "addIMUResidualBlock started!!!!!";

        auto it1 = m_Poses.find(lastposeIdx);
        auto it2 = m_Poses.find(curposeIdx);

        auto it1_motion = m_Motions.find(lastposeIdx);
        auto it2_motion = m_Motions.find(curposeIdx);

        edgeIMU->set_startPose(it1->second);
        edgeIMU->set_startMotion(it1_motion->second);

        edgeIMU->set_curPose(it2->second);
        edgeIMU->set_curMotion(it2_motion->second);
        edgeIMU->computeResidual();

        m_costIMUFunctions.push_back(edgeIMU);

        HashPoseIdTocostIMUFunction.insert(std::pair<int, costIMUFunction::Ptr>(lastposeIdx, edgeIMU));
        HashPoseIdTocostIMUFunction.insert(std::pair<int, costIMUFunction::Ptr>(curposeIdx, edgeIMU));
    }
    else
    {
        std::cout << "设定为无IMU，试图加入IMU！！！！！！！！！！！" << std::endl;
    }
}

void BA_problem::initialStructure(std::string config_yaml)
{
    LOG(INFO) << "BA_problem Initialization started!!!!!";
    Parameters::Ptr parameters_ptr(new SLAM_Solver::Parameters());
    parameters_ptr->set_parameters(config_yaml);
    int i = parameters_ptr->CAM_TIC.size();
    parameters = parameters_ptr;
    // std::cout << "输出的IMU到相机的外参： " << Parameters->CAM_TIC[0] << std::endl;
    // std::cout << "输出的是否为双目： " << Parameters->STEREO << std::endl;
    // std::cout << "输出的是否有IMU： " << Parameters->USE_IMU << std::endl;

    RIC0 = Eigen::Matrix3d::Identity();
    tic0 = Eigen::Vector3d::Zero();
    qic0 = Eigen::Quaterniond(RIC0);

    if (parameters->USE_IMU)
    {
        TIC0 = parameters->CAM_TIC[0];
        RIC0 = TIC0.block<3, 3>(0, 0);
        tic0 = TIC0.block<3, 1>(0, 3);
        qic0 = Eigen::Quaterniond(RIC0);
        LOG(INFO) << "STEREO";
        if (parameters->STEREO)
        {
            TIC1 = parameters->CAM_TIC[1];
            RIC1 = TIC1.block<3, 3>(0, 0);
            tic1 = TIC1.block<3, 1>(0, 3);
            qic1 = Eigen::Quaterniond(RIC1);
        }
    }
    m_Poses.clear();
    m_FeatureIDs.clear();
    m_Motions.clear();
    m_costFunctions.clear();
    m_costIMUFunctions.clear();
    m_costOneFrameTwoCamFunctions.clear();
    m_costTwoFrameTwoCamFunctions.clear();
    LOG(INFO) << "BA_problem Initialization succeeded!!!!!";
}

Pose::Ptr BA_problem::get_Pose(int poseIdx)
{
    std::lock_guard<std::mutex> lock(mutex_map);
    auto it = m_Poses.find(poseIdx);
    if (it != m_Poses.end())
    {
        return it->second;
    }
    return static_cast<Pose::Ptr>(nullptr);
}

FeatureID::Ptr BA_problem::get_FeatureID(int featureIdx)
{
    std::lock_guard<std::mutex> lock(mutex_map);
    auto it = m_FeatureIDs.find(featureIdx);
    if (it != m_FeatureIDs.end())
    {
        return it->second;
    }
    return static_cast<FeatureID::Ptr>(nullptr);
}

std::vector<costFunction::Ptr> BA_problem::get_all_costFunction()
{
    return m_costFunctions;
}

void BA_problem::solve()
{
    setOrdering();

    makeHession();

    ComputeLambdaInitLM();

    bool stop = false;
    int iter = 0;
    int iterations = 15;
    double last_chi_ = 1e20;
    while (!stop && (iter < iterations))
    {
        LOG(INFO) << "目前迭代的次数: " << iter << "对应的currentChi_： " << currentChi_ << "currentLambda_: " << currentLambda_;

        bool oneStepSuccess = false;
        int false_cnt = 0;
        while (!oneStepSuccess)
        {

            SolveLinearSystem();

            // 优化退出条件1： delta_x_ 很小则退出
            if (delta_x_.squaredNorm() <= 1e-6 || false_cnt > 10)
            {
                stop = true;
                LOG(INFO) << "!!!!!!!!!!!! 求解优化完成  !!!!!!！！！";
                break;
            }
            // 更新状态量
            UpdateStates();
            // 判断当前步是否可行以及 LM 的 lambda 怎么更新
            oneStepSuccess = IsGoodStepInLM();
            // 后续处理，
            if (oneStepSuccess)
            {
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
            }
            else
            {
                false_cnt++;
                RollbackStates(); // 误差没下降，回滚
            }
            // oneStepSuccess = true;
        }
        iter++;

        if (sqrt(currentChi_) <= stopThresholdLM_)
        {
            std::cout << "sqrt(currentChi_) <= stopThresholdLM_" << std::endl;
            stop = true;
        }
        last_chi_ = currentChi_;
    }
}

void BA_problem::RollbackStates()
{
    for (auto m_Pose : m_Poses)
    {
        ulong idx = m_Pose.second->OrderingId();
        ulong dim = 6;
        VecX delta = delta_x_.segment(idx, dim);
        m_Pose.second->Plus(-delta);
    }
    if (parameters->USE_IMU)
    {
        if (m_Motions.size() != 0)
        {
            for (auto m_Motion : m_Motions)
            {
                ulong idx_motion = m_Motion.second->OrderingId();
                ulong dim = 6;
                VecX delta = delta_x_.segment(idx_motion, dim);
                m_Motion.second->Plus(-delta);
            }
        }
    }
    for (auto m_FeatureID : m_FeatureIDs)
    {
        int index_feature = m_FeatureID.second->OrderingId();
        ulong dim = 1;
        VecX delta = delta_x_.segment(index_feature, dim);
        m_FeatureID.second->Plus(-delta);
    }
}

bool BA_problem::IsGoodStepInLM()
{
    double scale = 0;
    scale = 0.5 * delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
    scale += 1e-6; // make sure it's non-zero :)

    // recompute residuals after update state
    // TODO:: get robustChi2() instead of Chi2()
    double tempChi = 0.0;
    for (auto m_costFunction : m_costFunctions)
    {
        m_costFunction->ComputeResidual();
        tempChi += m_costFunction->Chi2();
    }
    if (parameters->USE_IMU)
    {
        for (auto m_costIMUFunction : m_costIMUFunctions)
        {
            m_costIMUFunction->computeResidual();
            tempChi += m_costIMUFunction->Chi2();
        }
    }

    double rho = (currentChi_ - tempChi) / scale;
    if (rho > 0 && std::isfinite(tempChi)) // last step was good, 误差在下降
    {
        double alpha = 1. - pow((2 * rho - 1), 3);
        alpha = std::min(alpha, 2. / 3.);
        double scaleFactor = (std::max)(1. / 3., alpha);
        currentLambda_ *= scaleFactor;
        ni_ = 2;
        currentChi_ = tempChi;
        return true;
    }
    else
    {
        currentLambda_ *= ni_;
        ni_ *= 2;
        return false;
    }
}

void BA_problem::UpdateStates()
{
    LOG(INFO) << " 开始更新变量 ！！！";
    for (auto m_Pose : m_Poses)
    {
        ulong idx = m_Pose.second->OrderingId();
        ulong dim = 6;
        VecX delta = delta_x_.segment(idx, dim);
        m_Pose.second->Plus(delta);
    }
    if (parameters->USE_IMU)
    {
        if (m_Motions.size() != 0)
        {
            for (auto m_Motion : m_Motions)
            {
                ulong idx_motion = m_Motion.second->OrderingId();
                ulong dim = 9;
                VecX delta = delta_x_.segment(idx_motion, dim);
                m_Motion.second->Plus(delta);
            }
        }
    }
    for (auto m_FeatureID : m_FeatureIDs)
    {
        int index_feature = m_FeatureID.second->OrderingId();
        ulong dim = 1;
        VecX delta = delta_x_.segment(index_feature, dim);
        m_FeatureID.second->Plus(delta);
    }
}

void BA_problem::SolveLinearSystem()
{
    int reserve_size, marg_size;
    reserve_size = m_Poses.size() * 6;
    marg_size = m_FeatureIDs.size() * 1;

    if (parameters->USE_IMU)
    {
        reserve_size = m_Poses.size() * 15;
        marg_size = m_FeatureIDs.size() * 1;
    }

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
    while (idx < marg_size)
    {
        Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
        idx += size;
    }
    // LOG(INFO) << "Hmm_inv: " << Hmm_inv ;
    // TODO:: home work. 完成舒尔补 Hpp, bpp 代码
    MatXX tempH = Hpm * Hmm_inv;
    H_pp_schur_ = Hessian_.block(0, 0, reserve_size, reserve_size) - tempH * Hmp; //??
    b_pp_schur_ = bpp - tempH * bmm;                                              // 边际概率

    // step2: solve Hpp * delta_x = bpp
    VecX delta_x_pp(VecX::Zero(reserve_size));
    // PCG Solver
    for (ulong i = 0; i < reserve_size; ++i)
    {
        H_pp_schur_(i, i) += currentLambda_;
    }

    // int n = H_pp_schur_.rows() * 2;                       // 迭代次数
    // delta_x_pp = PCGSolver(H_pp_schur_, b_pp_schur_, n);  // pcg 求解
    delta_x_pp = H_pp_schur_.ldlt().solve(b_pp_schur_);
    delta_x_.head(reserve_size) = delta_x_pp;
    LOG(INFO) << " 位姿 求解迭代完成 ！！！";
    // std::cout << "delta_x_: " << delta_x_.transpose() << std::endl;  // ??? 结果相差
    // std::cout << "delta_x_pp: " << delta_x_pp.transpose() << std::endl;  // ??? 结果相差

    // TODO:: home work. step3: solve landmark
    VecX delta_x_ll(marg_size);
    delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp); //?
    delta_x_.tail(marg_size) = delta_x_ll;
    LOG(INFO) << " 线性矩阵3D点 求解迭代完成 ！！！";
    // std::cout << "delta_x_: " << delta_x_.transpose() << std::endl;  // ??? 结果相差
    // delta_x_ll: -0.0058278 -0.0335995
}

VecX BA_problem::PCGSolver(const MatXX &A, const VecX &b, int maxIter)
{
    assert(A.rows() == A.cols() && "PCG solver ERROR: A is not a square matrix");
    int rows = b.rows();
    int n = maxIter < 0 ? rows : maxIter;
    VecX x(VecX::Zero(rows));
    MatXX M_inv = A.diagonal().asDiagonal().inverse();
    VecX r0(b); // initial r = b - A*0 = b
    VecX z0 = M_inv * r0;
    VecX p(z0);
    VecX w = A * p;
    double r0z0 = r0.dot(z0);
    double alpha = r0z0 / p.dot(w);
    VecX r1 = r0 - alpha * w;
    int i = 0;
    double threshold = 1e-6 * r0.norm();
    while (r1.norm() > threshold && i < n)
    {
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

void BA_problem::ComputeLambdaInitLM()
{
    ni_ = 2.;
    currentLambda_ = -1.;
    currentChi_ = 0.0;
    // TODO:: robust cost chi2
    for (auto m_costFunction : m_costFunctions)
    {
        currentChi_ += m_costFunction->Chi2();
    }
    if (parameters->USE_IMU)
    {
        for (auto m_costIMUFunction : m_costIMUFunctions)
        {
            currentChi_ += m_costIMUFunction->Chi2();
        }
    }
    LOG(INFO) << "currentChi_: " << currentChi_;

    stopThresholdLM_ = 1e-6 * currentChi_; // 迭代条件为 误差下降 1e-6 倍

    double maxDiagonal = 0;
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (ulong i = 0; i < size; ++i)
    {
        maxDiagonal = std::max(fabs(Hessian_(i, i)), maxDiagonal);
    }
    double tau = 1e-5;
    currentLambda_ = tau * maxDiagonal;
    LOG(INFO) << "currentLambda_: " << currentLambda_;
}

void BA_problem::setOrdering()
{
    // 每次重新计数
    ordering_poses_ = 0;
    ordering_generic_ = 0;
    ordering_landmarks_ = 0;

    //将对应的所需要加的pose以及imu统一加上,同时检测是否有IMU
    for (int i = 0; i < PoseIdxs.size(); i++)
    {
        auto it = m_Poses.find(PoseIdxs[i]);
        if (it != m_Poses.end())
        {
            ordering_generic_ += it->second->get_localDimension();
            it->second->SetOrderingId(ordering_poses_);
            ordering_poses_ += it->second->get_localDimension();
        }

        if (parameters->USE_IMU)
        {
            auto iter = m_Motions.find(PoseIdxs[i]);
            if (iter != m_Motions.end())
            {
                ordering_generic_ += iter->second->get_localDimension();
                iter->second->SetOrderingId(ordering_poses_);
                ordering_poses_ += iter->second->get_localDimension();
            }
        }
    }

    ordering_landmarks_ = ordering_poses_;
    for (int i = 0; i < FeatureIDIdxs.size(); i++)
    {
        auto it = m_FeatureIDs.find(FeatureIDIdxs[i]);
        if (it != m_FeatureIDs.end())
        {
            ordering_generic_ += it->second->get_localDimension();
            it->second->SetOrderingId(ordering_landmarks_);
            ordering_landmarks_ += it->second->get_localDimension();
        }
    }
}

void BA_problem::makeHession()
{
    // int size, pose_size, feature_size;
    // pose_size = m_Poses.size();
    // feature_size = m_FeatureIDs.size();
    // size = pose_size * 6 + feature_size;
    // int H_pose_size = pose_size * 6;

    int size = ordering_generic_;
    std::cout << "H矩阵大小： " << size << std::endl;

    // 直接构造大的 H 矩阵
    MatXX H(MatXX::Zero(size, size));
    VecX b(VecX::Zero(size));
    std::cout << H.cols() << std::endl;
    LOG(INFO) << "信息矩阵的列: " << H.cols() << "以及行： " << H.rows();
    LOG(INFO) << "残差的列: " << b.cols() << "以及行： " << b.rows();

    for (auto m_costFunction : m_costFunctions)
    {
        SLAM_Solver::Pose::Ptr start_cost_pose = m_costFunction->get_startPose();
        SLAM_Solver::Pose::Ptr cur_cost_pose = m_costFunction->get_curPose();
        SLAM_Solver::FeatureID::Ptr cur_cost_feature = m_costFunction->get_curFeature();
        m_costFunction->ComputeResidual();
        auto jacobians = m_costFunction->Jacobians();
        auto residual = m_costFunction->Residual();

        int start_frameIdx = start_cost_pose->get_ID();
        int cur_frameIdx = cur_cost_pose->get_ID();
        int cur_featureIdx = cur_cost_feature->get_ID();

        int order_start_frame_idx = start_cost_pose->OrderingId();
        int order_cur_frame_idx = cur_cost_pose->OrderingId();
        int order_cur_feature_idx = cur_cost_feature->OrderingId();
        // std::cout << "66666666666666666666666" << std::endl;
        // std::cout << "构造残差创建的每一个帧索引 index_start: " << order_start_frame_idx << " index_cur: " << order_cur_frame_idx << std::endl;
        // std::cout << "构造残差创建的每一个图像索引 index_feature: " << order_cur_feature_idx << std::endl;

        int index_start = find(PoseIdxs.begin(), PoseIdxs.end(), start_frameIdx) - PoseIdxs.begin();
        int index_cur = find(PoseIdxs.begin(), PoseIdxs.end(), cur_frameIdx) - PoseIdxs.begin();
        int index_feature = find(FeatureIDIdxs.begin(), FeatureIDIdxs.end(), cur_featureIdx) - FeatureIDIdxs.begin();
        // std::cout << "构造残差创建的每一个帧索引 index_start: " << index_start << " index_cur: " << index_cur << std::endl;
        // std::cout << "构造残差创建的每一个图像索引 index_feature: " << index_feature << std::endl;

        // int index_H_posei = index_start * 6;
        // int index_H_posej = index_cur * 6;
        // int index_H_feature = H_pose_size + index_feature;
        std::vector<int> index_H;
        // index_H.push_back(index_H_feature);
        // index_H.push_back(index_H_posei);
        // index_H.push_back(index_H_posej);

        index_H.push_back(order_cur_feature_idx);
        index_H.push_back(order_start_frame_idx);
        index_H.push_back(order_cur_frame_idx);
        std::vector<int> index_dim;
        index_dim.push_back(1);
        index_dim.push_back(6);
        index_dim.push_back(6);
        for (int i = 0; i < jacobians.size(); i++)
        {
            auto jacobians_i = jacobians[i];
            int index_i = index_H[i];
            int index_dim_i = index_dim[i];
            for (int j = i; j < jacobians.size(); j++)
            {
                auto jacobians_j = jacobians[j];
                int index_j = index_H[j];
                int index_dim_j = index_dim[j];
                MatXX hessian = jacobians_i.transpose() * jacobians_j;
                // std::cout << "构建的信息矩阵 i: " << i << " j: " << j << hessian << std::endl;
                H.block(index_i, index_j, index_dim_i, index_dim_j).noalias() += hessian;
                if (j != i)
                {
                    H.block(index_j, index_i, index_dim_j, index_dim_i).noalias() += hessian.transpose();
                }
            }

            b.segment(index_i, index_dim_i).noalias() -= jacobians_i.transpose() * residual;
        }
    }
    LOG(INFO) << "BA_problem make 视觉左目 Hession succeeded!!!!!";

    if (parameters->STEREO)
    {
        if (m_costOneFrameTwoCamFunctions.size() != 0)
        {
            for (auto m_costOneFrameTwoCamFunction : m_costOneFrameTwoCamFunctions)
            {
                SLAM_Solver::FeatureID::Ptr cur_cost_feature = m_costOneFrameTwoCamFunction->get_curFeature();

                int order_start_frame_idx = cur_cost_feature->OrderingId();
                int order_dim = cur_cost_feature->get_localDimension();
                auto jacobians = m_costOneFrameTwoCamFunction->Jacobians();
                auto residual = m_costOneFrameTwoCamFunction->Residual();

                for (int i = 0; i < jacobians.size(); i++)
                {
                    auto jacobians_i = jacobians[i];
                    int index_i = order_start_frame_idx;
                    int index_dim_i = order_dim;
                    for (int j = i; j < jacobians.size(); j++)
                    {
                        auto jacobians_j = jacobians[j];
                        int index_j = order_start_frame_idx;
                        int index_dim_j = order_dim;
                        MatXX hessian = jacobians_i.transpose() * jacobians_j;
                        H.block(index_i, index_j, index_dim_i, index_dim_j).noalias() += hessian;
                        if (j != i)
                        {
                            H.block(index_j, index_i, index_dim_j, index_dim_i).noalias() += hessian.transpose();
                        }
                    }

                    b.segment(index_i, index_dim_i).noalias() -= jacobians_i.transpose() * residual;
                }
            }
        }

        if (m_costTwoFrameTwoCamFunctions.size() != 0)
        {
            std::cout << "5555555555555" << std::endl;
            for (auto m_costTwoFrameTwoCamFunction : m_costTwoFrameTwoCamFunctions)
            {
                SLAM_Solver::Pose::Ptr start_cost_pose = m_costTwoFrameTwoCamFunction->get_startPose();
                SLAM_Solver::Pose::Ptr cur_cost_pose = m_costTwoFrameTwoCamFunction->get_curPose();
                SLAM_Solver::FeatureID::Ptr cur_cost_feature = m_costTwoFrameTwoCamFunction->get_curFeature();

                m_costTwoFrameTwoCamFunction->ComputeResidual();
                auto jacobians = m_costTwoFrameTwoCamFunction->Jacobians();
                auto residual = m_costTwoFrameTwoCamFunction->Residual();

                int order_start_frame_idx = start_cost_pose->OrderingId();
                int order_cur_frame_idx = cur_cost_pose->OrderingId();
                int order_cur_feature_idx = cur_cost_feature->OrderingId();
                std::vector<int> index_H;

                index_H.push_back(order_start_frame_idx);
                index_H.push_back(order_cur_frame_idx);
                index_H.push_back(order_cur_feature_idx);
                std::vector<int> index_dim;
                index_dim.push_back(6);
                index_dim.push_back(6);
                index_dim.push_back(1);
                for (int i = 0; i < jacobians.size(); i++)
                {
                    auto jacobians_i = jacobians[i];
                    int index_i = index_H[i];
                    int index_dim_i = index_dim[i];
                    for (int j = i; j < jacobians.size(); j++)
                    {
                        auto jacobians_j = jacobians[j];
                        int index_j = index_H[j];
                        int index_dim_j = index_dim[j];
                        MatXX hessian = jacobians_i.transpose() * jacobians_j;
                        // std::cout << "构建的信息矩阵 i: " << i << " j: " << j << hessian << std::endl;
                        H.block(index_i, index_j, index_dim_i, index_dim_j).noalias() += hessian;
                        if (j != i)
                        {
                            H.block(index_j, index_i, index_dim_j, index_dim_i).noalias() += hessian.transpose();
                        }
                    }

                    b.segment(index_i, index_dim_i).noalias() -= jacobians_i.transpose() * residual;
                }
            }
        }
    }

    if (parameters->USE_IMU)
    {
        LOG(INFO) << "H矩阵开始计算对应的IMU约束信息!!!!!";
        if (m_costIMUFunctions.size() != 0)
        {
            for (auto m_costIMUFunction : m_costIMUFunctions)
            {
                SLAM_Solver::Pose::Ptr start_cost_pose = m_costIMUFunction->get_startPose();
                SLAM_Solver::Pose::Ptr cur_cost_pose = m_costIMUFunction->get_curPose();
                SLAM_Solver::Motion::Ptr curMotion = m_costIMUFunction->get_curMotion();
                SLAM_Solver::Motion::Ptr startMotion = m_costIMUFunction->get_startMotion();

                auto jacobians = m_costIMUFunction->Jacobians();
                auto residual = m_costIMUFunction->Residual();

                int start_pose_index = start_cost_pose->OrderingId();
                int cur_pose_index = cur_cost_pose->OrderingId();
                int start_motion_index = startMotion->OrderingId();
                int cur_motion_index = curMotion->OrderingId();

                int start_pose_dim = start_cost_pose->get_localDimension();
                int cur_pose_dim = cur_cost_pose->get_localDimension();
                int start_motion_dim = startMotion->get_localDimension();
                int cur_motion_dim = curMotion->get_localDimension();

                std::vector<int> index_H;
                index_H.push_back(start_pose_index);
                index_H.push_back(start_motion_index);
                index_H.push_back(cur_pose_index);
                index_H.push_back(cur_motion_index);
                std::vector<int> index_dim;
                index_dim.push_back(start_pose_dim);
                index_dim.push_back(start_motion_dim);
                index_dim.push_back(cur_pose_dim);
                index_dim.push_back(cur_motion_dim);

                for (int i = 0; i < jacobians.size(); i++)
                {
                    auto jacobians_i = jacobians[i];
                    int index_i = index_H[i];
                    int index_dim_i = index_dim[i];
                    for (int j = i; j < jacobians.size(); j++)
                    {
                        auto jacobians_j = jacobians[j];
                        int index_j = index_H[j];
                        int index_dim_j = index_dim[j];
                        MatXX hessian = jacobians_i.transpose() * jacobians_j;
                        // std::cout << "构建的信息矩阵 i: " << i << " j: " << j << hessian << std::endl;
                        H.block(index_i, index_j, index_dim_i, index_dim_j).noalias() += hessian;
                        if (j != i)
                        {
                            H.block(index_j, index_i, index_dim_j, index_dim_i).noalias() += hessian.transpose();
                        }
                    }

                    b.segment(index_i, index_dim_i).noalias() -= jacobians_i.transpose() * residual;
                }
            }
        }
    }

    Hessian_ = H;
    // std::cout << "ceres所构建的H is :" << std::endl << Hessian_ << std::endl;
    b_ = b;
    // std::cout << "ceres所构建的b is :" << std::endl << b_ << std::endl;

    LOG(INFO) << "BA_problem make Hession succeeded!!!!!";
    delta_x_ = VecX::Zero(size); // initial delta_x = 0_n;
}

void BA_problem::getSolveResults()
{
    std::vector<Pose::Ptr> res;
    for (auto pose : m_Poses)
    {
        res.push_back(pose.second);
    }
    res.shrink_to_fit();
    std::sort(res.begin(), res.end(), [](Pose::Ptr ptr1, Pose::Ptr ptr2) {
        return ptr1->m_frame_id < ptr2->m_frame_id;
    });
    for (auto m_Pose : res)
    {
        int start_frameIdx = m_Pose->get_ID();
        Eigen::Quaterniond q1 = m_Pose->get_rotate();
        Eigen::Vector3d t1 = m_Pose->get_translation();
        std::cout << std::setprecision(20) << start_frameIdx << " " << t1[0] << " " << t1[1] << " " << t1[2] << " "
                  << q1.x() << " " << q1.y() << " " << q1.z() << " " << q1.w() << std::endl;
    }

    std::vector<FeatureID::Ptr> fea_res;
    for (auto FeatureID : m_FeatureIDs)
    {
        fea_res.push_back(FeatureID.second);
    }
    fea_res.shrink_to_fit();
    std::sort(fea_res.begin(), fea_res.end(), [](FeatureID::Ptr ptr1, FeatureID::Ptr ptr2) {
        return ptr1->featureId < ptr2->featureId;
    });
    for (auto m_FeatureID : fea_res)
    {
        int start_feaIdx = m_FeatureID->get_ID();
        double m_faeture_p = m_FeatureID->get_invdep();
        std::cout << "优化第 " << start_feaIdx << "逆深度的值为： " << m_faeture_p << std::endl;
    }
}

void BA_problem::readParameters(std::string config_file)
{
}

std::vector<Pose::Ptr> BA_problem::get_all_Pose()
{
    std::lock_guard<std::mutex> lock(mutex_map);
    std::vector<Pose::Ptr> res;
    for (auto pose : m_Poses)
    {
        res.push_back(pose.second);
    }
    res.shrink_to_fit();
    std::sort(res.begin(), res.end(), [](Pose::Ptr ptr1, Pose::Ptr ptr2) {
        return ptr1->m_frame_id < ptr2->m_frame_id;
    });

    return res;
}

std::vector<int> BA_problem::get_all_Poseidx()
{
    std::lock_guard<std::mutex> lock(mutex_map);
    std::vector<int> res;
    for (int i = 0; i < PoseIdxs.size(); i++)
    {
        res.push_back(PoseIdxs[i]);
        std::cout << "push进vector中的第" << i << "ge  data: " << PoseIdxs[i] << std::endl;
    }
    return res;
}

std::vector<FeatureID::Ptr> BA_problem::get_all_FeatureID()
{
    std::lock_guard<std::mutex> lock(mutex_map);
    std::vector<FeatureID::Ptr> res;
    for (auto FeatureID : m_FeatureIDs)
    {
        res.push_back(FeatureID.second);
    }
    res.shrink_to_fit();
    std::sort(res.begin(), res.end(), [](FeatureID::Ptr ptr1, FeatureID::Ptr ptr2) {
        return ptr1->featureId < ptr2->featureId;
    });

    return res;
}

void BA_problem::setMargin(int poseIdx)
{
    setOrdering();

    /// pose and IMU
    /// TODO
    /// pose_dim++
    int pose_dim;
    pose_dim = m_Poses.size() * 6;
    pose_dim += 9 * 2;

    auto range = HashPoseIdTocostFunction.equal_range(poseIdx);
    std::vector<costFunction::Ptr> marg_costFunction;

    // auto range = HashPoseIdTocostFunction.equal_range(poseIdx);
    // std::vector<costFunction::Ptr> marg_costFunction;
    for (auto iter = range.first; iter != range.second; ++iter)
    {

        // // 并且这个edge还需要存在，而不是已经被remove了
        // if (m_costFunctions.find(iter->second) == m_costFunctions.end())
        //     continue;

        marg_costFunction.emplace_back(iter->second);
    }

    std::unordered_map<int, FeatureID::Ptr> margLandmark;
    // 构建 Hessian 的时候 pose 的顺序不变，landmark的顺序要重新设定
    int marg_landmark_size = 0;
    //    std::cout << "\n marg edge 1st id: "<< marg_edges.front()->Id() << " end id: "<<marg_edges.back()->Id()<<std::endl;
    for (size_t i = 0; i < marg_costFunction.size(); ++i)
    {
        //        std::cout << "marg edge id: "<< marg_edges[i]->Id() <<std::endl;
        auto marg_lardmark = marg_costFunction[i]->get_curFeature();
        marg_lardmark->SetOrderingId(pose_dim + marg_landmark_size);
        margLandmark.insert(make_pair(marg_lardmark->get_ID(), marg_lardmark));
        marg_landmark_size += marg_lardmark->get_localDimension();
    }

    int cols = pose_dim + marg_landmark_size;
    /// 构建误差 H 矩阵 H = H_marg + H_pp_prior
    MatXX H_marg(MatXX::Zero(cols, cols));
    VecX b_marg(VecX::Zero(cols));
    int ii = 0;

    for (auto m_costpose : marg_costFunction)
    {
        m_costpose->ComputeResidual();
        auto jacobians = m_costpose->Jacobians();
        auto residual = m_costpose->Residual();
        ii++;

        SLAM_Solver::Pose::Ptr start_cost_pose = m_costpose->get_startPose();
        SLAM_Solver::Pose::Ptr cur_cost_pose = m_costpose->get_curPose();
        SLAM_Solver::FeatureID::Ptr cur_cost_feature = m_costpose->get_curFeature();

        int index_H_posei = start_cost_pose->OrderingId();
        int index_H_posej = cur_cost_pose->OrderingId();
        int index_H_feature = cur_cost_feature->OrderingId();
        std::vector<int> index_H;
        index_H.push_back(index_H_feature);
        index_H.push_back(index_H_posei);
        index_H.push_back(index_H_posej);
        std::vector<int> index_dim;
        index_dim.push_back(1);
        index_dim.push_back(6);
        index_dim.push_back(6);

        //@@@@@@@@@@  信息矩阵的添加
        // MatXX robustInfo(m_costpose->Information().rows(),m_costpose->Information().cols());

        for (int i = 0; i < jacobians.size(); i++)
        {
            auto jacobians_i = jacobians[i];
            int index_i = index_H[i];
            int index_dim_i = index_dim[i];
            for (int j = i; j < jacobians.size(); j++)
            {
                auto jacobians_j = jacobians[j];
                int index_j = index_H[j];
                int index_dim_j = index_dim[j];

                //@@@@@@@@@@  信息矩阵的添加
                MatXX hessian = jacobians_i.transpose() * jacobians_j;
                // std::cout << "构建的信息矩阵 i: " << i << " j: " << j << hessian << std::endl;
                H_marg.block(index_i, index_j, index_dim_i, index_dim_j).noalias() += hessian;
                if (j != i)
                {
                    H_marg.block(index_j, index_i, index_dim_j, index_dim_i).noalias() += hessian.transpose();
                }
            }

            b_marg.segment(index_i, index_dim_i).noalias() -= jacobians_i.transpose() * residual;
        }
    }

    /// marg landmark
    int reserve_size = pose_dim;
    if (marg_landmark_size > 0)
    {
        int marg_size = marg_landmark_size;
        MatXX Hmm = H_marg.block(reserve_size, reserve_size, marg_size, marg_size);
        MatXX Hpm = H_marg.block(0, reserve_size, reserve_size, marg_size);
        MatXX Hmp = H_marg.block(reserve_size, 0, marg_size, reserve_size);
        VecX bpp = b_marg.segment(0, reserve_size);
        VecX bmm = b_marg.segment(reserve_size, marg_size);

        // Hmm 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
        MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
        // TODO:: use openMP
        for (auto iter : margLandmark)
        {
            int idx = iter.second->OrderingId() - reserve_size;
            int size = iter.second->get_localDimension();
            Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
        }

        MatXX tempH = Hpm * Hmm_inv;
        MatXX Hpp = H_marg.block(0, 0, reserve_size, reserve_size) - tempH * Hmp;
        bpp = bpp - tempH * bmm;
        H_marg = Hpp;
        b_marg = bpp;
    }

    VecX b_prior_before = b_prior_;
    if (H_prior_.rows() > 0)
    {
        H_marg += H_prior_;
        b_marg += b_prior_;
    }

    /// marg frame and speedbias
    int marg_dim = 0;

    {
        auto it1 = m_Motions.find(poseIdx);
        auto marg_motion = it1->second;

        int idx = marg_motion->OrderingId();
        int dim = marg_motion->get_localDimension();
        //        std::cout << k << " "<<idx << std::endl;
        marg_dim += dim;
        // move the marg pose to the Hmm bottown right
        // 将 row i 移动矩阵最下面
        Eigen::MatrixXd temp_rows = H_marg.block(idx, 0, dim, reserve_size);
        Eigen::MatrixXd temp_botRows = H_marg.block(idx + dim, 0, reserve_size - idx - dim, reserve_size);
        H_marg.block(idx, 0, reserve_size - idx - dim, reserve_size) = temp_botRows;
        H_marg.block(reserve_size - dim, 0, dim, reserve_size) = temp_rows;

        // 将 col i 移动矩阵最右边
        Eigen::MatrixXd temp_cols = H_marg.block(0, idx, reserve_size, dim);
        Eigen::MatrixXd temp_rightCols = H_marg.block(0, idx + dim, reserve_size, reserve_size - idx - dim);
        H_marg.block(0, idx, reserve_size, reserve_size - idx - dim) = temp_rightCols;
        H_marg.block(0, reserve_size - dim, reserve_size, dim) = temp_cols;

        Eigen::VectorXd temp_b = b_marg.segment(idx, dim);
        Eigen::VectorXd temp_btail = b_marg.segment(idx + dim, reserve_size - idx - dim);
        b_marg.segment(idx, reserve_size - idx - dim) = temp_btail;
        b_marg.segment(reserve_size - dim, dim) = temp_b;
    }

    {
        auto it1 = m_Poses.find(poseIdx);
        auto marg_pose = it1->second;

        int idx = marg_pose->OrderingId();
        int dim = marg_pose->get_localDimension();
        //        std::cout << k << " "<<idx << std::endl;
        marg_dim += dim;
        // move the marg pose to the Hmm bottown right
        // 将 row i 移动矩阵最下面
        Eigen::MatrixXd temp_rows = H_marg.block(idx, 0, dim, reserve_size);
        Eigen::MatrixXd temp_botRows = H_marg.block(idx + dim, 0, reserve_size - idx - dim, reserve_size);
        H_marg.block(idx, 0, reserve_size - idx - dim, reserve_size) = temp_botRows;
        H_marg.block(reserve_size - dim, 0, dim, reserve_size) = temp_rows;

        // 将 col i 移动矩阵最右边
        Eigen::MatrixXd temp_cols = H_marg.block(0, idx, reserve_size, dim);
        Eigen::MatrixXd temp_rightCols = H_marg.block(0, idx + dim, reserve_size, reserve_size - idx - dim);
        H_marg.block(0, idx, reserve_size, reserve_size - idx - dim) = temp_rightCols;
        H_marg.block(0, reserve_size - dim, reserve_size, dim) = temp_cols;

        Eigen::VectorXd temp_b = b_marg.segment(idx, dim);
        Eigen::VectorXd temp_btail = b_marg.segment(idx + dim, reserve_size - idx - dim);
        b_marg.segment(idx, reserve_size - idx - dim) = temp_btail;
        b_marg.segment(reserve_size - dim, dim) = temp_b;
    }

    double eps = 1e-8;
    int m2 = marg_dim;
    int n2 = reserve_size - marg_dim; // marg pose
    Eigen::MatrixXd Amm = 0.5 * (H_marg.block(n2, n2, m2, m2) + H_marg.block(n2, n2, m2, m2).transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() *
                              saes.eigenvectors().transpose();

    Eigen::VectorXd bmm2 = b_marg.segment(n2, m2);
    Eigen::MatrixXd Arm = H_marg.block(0, n2, n2, m2);
    Eigen::MatrixXd Amr = H_marg.block(n2, 0, m2, n2);
    Eigen::MatrixXd Arr = H_marg.block(0, 0, n2, n2);
    Eigen::VectorXd brr = b_marg.segment(0, n2);
    Eigen::MatrixXd tempB = Arm * Amm_inv;
    H_prior_ = Arr - tempB * Amr;
    b_prior_ = brr - tempB * bmm2;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H_prior_);
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd(
        (saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    Jt_prior_inv_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    err_prior_ = -Jt_prior_inv_ * b_prior_;

    MatXX J = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    H_prior_ = J.transpose() * J;
    MatXX tmp_h = MatXX((H_prior_.array().abs() > 1e-9).select(H_prior_.array(), 0));
    H_prior_ = tmp_h;

    // std::cout << "my marg b prior: " <<b_prior_.rows()<<" norm: "<< b_prior_.norm() << std::endl;
    // std::cout << "    error prior: " <<err_prior_.norm() << std::endl;

    // remove vertex and remove edge
    ///@@@@@
}

void BA_problem::test()
{
    setOrdering();
    for (auto m_Pose : m_Poses)
    {
        int start_frameIdx = m_Pose.second->get_ID();
        int index_start = find(PoseIdxs.begin(), PoseIdxs.end(), start_frameIdx) - PoseIdxs.begin();
        ulong idx = index_start * 6;
        int pose_idx = m_Pose.second->OrderingId();
        std::cout << "对应的每一帧的第 " << start_frameIdx << " 帧的idx: " << idx << " 的pose_idx" << pose_idx << std::endl;
        // ulong dim = 6;
        // VecX delta = delta_x_.segment(idx, dim);
        // m_Pose.second->Plus(delta);
    }
    for (auto m_FeatureID : m_FeatureIDs)
    {
        int start_featureIdx = m_Poses.size() * 6;
        int start_feaIdx = m_FeatureID.second->get_ID();
        int index_feature = find(FeatureIDIdxs.begin(), FeatureIDIdxs.end(), start_feaIdx) - FeatureIDIdxs.begin();
        index_feature += start_featureIdx;
        int feature_idx = m_FeatureID.second->OrderingId();
        std::cout << "对应的每一帧的第 " << start_feaIdx << " 帧的idx: " << index_feature << " 的pose_idx" << feature_idx << std::endl;
        // ulong dim = 1;
        // VecX delta = delta_x_.segment(index_feature, dim);
        // m_FeatureID.second->Plus(delta);
    }
}