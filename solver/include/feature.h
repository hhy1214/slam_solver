// #ifndef SOLVER_SLAM_FEATURE_H
// #define SOLVER_SLAM_FEATURE_H

// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <vector>
// #include <map>
// #include "macro.h"
// #include <unordered_map>

// namespace SLAM_Solver
// {
//         class FeatureMeasure
//     {
//     public:
//         POINTER_TYPEDEFS(FeatureMeasure);
//         FeatureMeasure() : isDouble(false){};
//         void set_leftpoint(Eigen::Vector3d m_leftpoint_)
//         {
//             m_leftpoint = m_leftpoint_;
//         }
//         void set_rightpoint(Eigen::Vector3d m_rightpoint_)
//         {
//             isDouble = true;
//             m_rightpoint = m_rightpoint_;
//         }
//         Eigen::Vector3d get_leftpoint() {
//             return m_leftpoint;
//         }
//         Eigen::Vector3d get_rightpoint() {
//             return m_rightpoint;
//         }
         

//     private:
//         bool isDouble;
//         Eigen::Vector3d m_leftpoint;
//         Eigen::Vector2d m_leftuv;
//         Eigen::Vector3d m_rightpoint;
//         Eigen::Vector2d m_rightuv;
//     };


//     class FeatureID
//     {
//     public:
//         POINTER_TYPEDEFS(FeatureID);
//         FeatureID(int featureid_) : featureId(featureid_), isMargin(false), localDimension(1){};

//         void set_startframe(int frameIdx)
//         {
//             start_frame = frameIdx;
//         }

//         int get_startframe()
//         {
//             return start_frame;
//         }

//         void set_invdep(double inv_Dep)
//         {
//             invDep = inv_Dep;
//         }

//         int get_ID()
//         {
//             return featureId;
//         }

//         int get_localDimension()
//         {
//             return localDimension;
//         }

//         double get_invdep()
//         {
//             return invDep;
//         }

//         bool addFeatureMeasure(int poseIdx, Eigen::Vector3d m_leftpoint_)
//         {
//             FeatureMeasure::Ptr feature_ptr(new SLAM_Solver::FeatureMeasure());
//             feature_ptr->set_leftpoint(m_leftpoint_);
//             auto res = m_measures.insert(std::make_pair(poseIdx, feature_ptr));
//             return res.second;
//         }

//         FeatureMeasure::Ptr get_FeatureMeasure(int poseIdx)
//         {
//             auto it = m_measures.find(poseIdx);
//             if (it != m_measures.end())
//             {
//                 return it->second;
//             }
//             return static_cast<FeatureMeasure::Ptr>(nullptr);
//         }

//         void Plus(VecX delta_) {
//             invDep += delta_[0];
//         }

//         int OrderingId() const { return ordering_id_; }
//         void SetOrderingId(unsigned long id) { ordering_id_ = id; };

//         int marginOrderingId() const { return margin_ordering_id_; }
//         void SetmarginOrderingId(unsigned long id) { margin_ordering_id_ = id; isMargin = true;};

//     public:
//         int featureId;            // feature  id
//         bool isMargin;
//         unsigned long margin_ordering_id_ = 0;

//     private:
//         int start_frame;          // 起始帧 id
//         int localDimension;       // 局部参数化维度
//         double invDep;            // 逆深度
//         Eigen::Vector3d start_uv; // 起始帧归一化坐标
//         std::unordered_map<int, FeatureMeasure::Ptr> m_measures;

//         /// ordering id是在problem中排序后的id，用于寻找雅可比对应块
//         /// ordering id带有维度信息，例如ordering_id=6则对应Hessian中的第6列
//         /// 从零开始
//         unsigned long ordering_id_ = 0;
//     };
// } // namespace SLAM_Solver


// #endif

#ifndef SOLVER_SLAM_FEATURE_H
#define SOLVER_SLAM_FEATURE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include "macro.h"
#include <unordered_map>

namespace SLAM_Solver
{
        class FeatureMeasure
    {
    public:
        POINTER_TYPEDEFS(FeatureMeasure);
        FeatureMeasure() : isDouble(false){};
        void set_leftpoint(Eigen::Vector3d m_leftpoint_)
        {
            m_leftpoint = m_leftpoint_;
        }
        void set_rightpoint(Eigen::Vector3d m_rightpoint_)
        {
            isDouble = true;
            m_rightpoint = m_rightpoint_;
        }
        Eigen::Vector3d get_leftpoint() {
            return m_leftpoint;
        }
        Eigen::Vector3d get_rightpoint() {
            return m_rightpoint;
        }
         

    private:
        bool isDouble;
        Eigen::Vector3d m_leftpoint;
        Eigen::Vector2d m_leftuv;
        Eigen::Vector3d m_rightpoint;
        Eigen::Vector2d m_rightuv;
    };


    class FeatureID
    {
    public:
        POINTER_TYPEDEFS(FeatureID);
        FeatureID(int featureid_) : featureId(featureid_), localDimension(1){};

        void set_startframe(int frameIdx)
        {
            start_frame = frameIdx;
        }

        int get_startframe()
        {
            return start_frame;
        }

        void set_invdep(double inv_Dep)
        {
            invDep = inv_Dep;
        }

        int get_ID()
        {
            return featureId;
        }

        int get_localDimension()
        {
            return localDimension;
        }

        double get_invdep()
        {
            return invDep;
        }
        // 添加左目观测
        bool addFeatureMeasure(int poseIdx, Eigen::Vector3d m_leftpoint_)
        {
            if(m_measures.find(poseIdx) == m_measures.end())
            {
                FeatureMeasure::Ptr feature_ptr(new SLAM_Solver::FeatureMeasure());
                feature_ptr->set_leftpoint(m_leftpoint_);
                auto res = m_measures.insert(std::make_pair(poseIdx, feature_ptr));
                return res.second;
            }
            else 
            {
                auto feature_ptr = m_measures[poseIdx];
                feature_ptr->set_leftpoint(m_leftpoint_);
                return true;
            }
        }
        // 添加右目观测
        bool addRightFeatureMeasure(int poseIdx, Eigen::Vector3d m_rightpoint_)
        {
            if(m_measures.find(poseIdx) == m_measures.end())
            {
                FeatureMeasure::Ptr feature_ptr(new SLAM_Solver::FeatureMeasure());
                feature_ptr->set_rightpoint(m_rightpoint_);
                auto res = m_measures.insert(std::make_pair(poseIdx, feature_ptr));
                return res.second;
            }
            else {
                auto feature_ptr = m_measures[poseIdx];
                feature_ptr->set_rightpoint(m_rightpoint_);
                return true;
            }
        }


        FeatureMeasure::Ptr get_FeatureMeasure(int poseIdx)
        {
            auto it = m_measures.find(poseIdx);
            if (it != m_measures.end())
            {
                return it->second;
            }
            return static_cast<FeatureMeasure::Ptr>(nullptr);
        }

        void Plus(VecX delta_) {
            invDep += delta_[0];
        }

        int OrderingId() const { return ordering_id_; }
        void SetOrderingId(unsigned long id) { ordering_id_ = id; };

    public:
        int featureId;            // feature  id

    private:
        int start_frame;          // 起始帧 id
        int localDimension;       // 局部参数化维度
        double invDep;            // 逆深度
        Eigen::Vector3d start_uv; // 起始帧归一化坐标
        std::unordered_map<int, FeatureMeasure::Ptr> m_measures;

        /// ordering id是在problem中排序后的id，用于寻找雅可比对应块
        /// ordering id带有维度信息，例如ordering_id=6则对应Hessian中的第6列
        /// 从零开始
        unsigned long ordering_id_ = 0;
    };
} // namespace SLAM_Solver


#endif