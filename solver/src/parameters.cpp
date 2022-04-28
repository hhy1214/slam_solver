#include "parameters.h"

namespace SLAM_Solver
{

    void parameters::set_parameters(std::string config_yaml)
    {
        LOG(INFO) << "Start readParameters!";
        FILE *fh = fopen(config_yaml.c_str(), "r");
        if (fh == NULL)
        {
            ROS_WARN("config_file dosen't exist; wrong config_file path");
            ROS_BREAK();
            return;
        }
        fclose(fh);

        cv::FileStorage fsSettings(config_yaml, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
        }

        USE_IMU = fsSettings["imu"];
        NUM_OF_CAM = fsSettings["num_of_cam"];
        STEREO = fsSettings["stereo"];
        std::cout << "USE_IMU: " << USE_IMU << std::endl;
        std::cout << "NUM_OF_CAM: " << NUM_OF_CAM << std::endl;
        std::cout << "STEREO: " << STEREO << std::endl;

        LARDMARK_TYPE = fsSettings["landmarktype"];
        std::cout << "LARDMARK_TYPE: " << LARDMARK_TYPE << std::endl;

        if (NUM_OF_CAM < 1)
        {
            printf("num_of_cam should be biger than 0!\n");
            // assert(0);
        }
        // multi camera image and imu topic
        Cameras.resize(NUM_OF_CAM);
        if (STEREO)
        {
            Cameras.resize(2 * NUM_OF_CAM);
        }
        if (USE_IMU)
        {
            CAM_TIC.resize(2 * NUM_OF_CAM);
            CAM_NAMES.resize(2 * NUM_OF_CAM);
        }
        
        fsSettings["camera0_left_topic"] >> Cameras[0].cameraName;
        Cameras[0].w = static_cast<int>(fsSettings["image_width"]);
        Cameras[0].h = static_cast<int>(fsSettings["image_height"]);
        cv::FileNode n = fsSettings["distortion_parameters"];
        Cameras[0].k1 = static_cast<double>(n["k1"]);
        Cameras[0].k2 = static_cast<double>(n["k2"]);
        Cameras[0].p1 = static_cast<double>(n["p1"]);
        Cameras[0].p2 = static_cast<double>(n["p2"]);

        n = fsSettings["projection_parameters"];
        Cameras[0].fx = static_cast<double>(n["fx"]);
        Cameras[0].fy = static_cast<double>(n["fy"]);
        Cameras[0].cx = static_cast<double>(n["cx"]);
        Cameras[0].cy = static_cast<double>(n["cy"]);
        std::cout << "Cameras[0].cameraName: " << Cameras[0].cameraName << std::endl;
        std::cout << "Cameras[0].k1: " << Cameras[0].k1 << std::endl;
        std::cout << "Cameras[0].k2: " << Cameras[0].k2 << std::endl;
        std::cout << "Cameras[0].p1: " << Cameras[0].p1 << std::endl;
        std::cout << "Cameras[0].p2: " << Cameras[0].p2 << std::endl;
        std::cout << "Cameras[0].fx: " << Cameras[0].fx << std::endl;
        std::cout << "Cameras[0].fy: " << Cameras[0].fy << std::endl;
        std::cout << "Cameras[0].cx: " << Cameras[0].cx << std::endl;
        std::cout << "Cameras[0].cy: " << Cameras[0].cy << std::endl;
        cv::Mat cv_T;
        Eigen::Matrix4d T;

        int pn = config_yaml.find_last_of('/');
        std::string configPath = config_yaml.substr(0, pn);

        if (USE_IMU)
        {
            if (NUM_OF_CAM > 0)
            {
                // cam0 left extrinsic param
                fsSettings["imu_T_cam0_left"] >> cv_T;
                cv::cv2eigen(cv_T, T);
                CAM_TIC[0] = T;
            }
        }
    }

} // namespace SLAM_Solver