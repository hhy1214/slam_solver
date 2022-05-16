#include "motion.h"

namespace SLAM_Solver
{

    void Motion::set_Motion(Eigen::Vector3d _Vs, Eigen::Vector3d _Ba, Eigen::Vector3d _Bg)
    {
        Vs = _Vs;
        Ba = _Ba;
        Bg = _Bg;
    }
    std::vector<Eigen::Vector3d> Motion::get_Motion()
    {
        std::vector<Eigen::Vector3d> motion;
        motion.push_back(Vs);
        motion.push_back(Ba);
        motion.push_back(Bg);
        return motion;
    }
    int Motion::get_ID() 
    {
        return m_frame_id;
    }
    Eigen::Vector3d Motion::get_Vs() 
    {
        return Vs;
    }
    Eigen::Vector3d Motion::get_Ba()
    {
        return Ba;
    }
    Eigen::Vector3d Motion::get_Bg()
    {
        return Bg;
    }

    void Motion::Plus(VecX delta_) {
        Vs += delta_.head(3);
        Ba += delta_.segment(3, 3);
        Bg += delta_.tail(3);
    }
    
} // namespace SLAM_Solver
