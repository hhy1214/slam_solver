#include "parameters.h"

double ACC_N = 0.258563568032;
double ACC_W = 0.000466069600446;
double GYR_N = 0.0377318721769;
double GYR_W = 3.44012936063e-05;
Eigen::Vector3d G{0, 0, 9.805};

namespace SLAM_Solver
{

    void Parameters::set_parameters(std::string config_yaml)
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

        // 相机左目内参
        for(int i = 0; i < NUM_OF_CAM; i ++) 
        {
            Camera camera;
            std::string cameraName = "camera" + std::to_string(i) + "_left_topic"; 
            fsSettings[cameraName] >> camera.cameraName;
            camera.w = static_cast<int>(fsSettings["image_width"]);
            camera.h = static_cast<int>(fsSettings["image_height"]);
            std::string cam_left_dist_para_name = "cam" + std::to_string(i) + "_left_distortion_parameters";
            cv::FileNode n = fsSettings[cam_left_dist_para_name];
            camera.k1 = static_cast<double>(n["k1"]);
            camera.k2 = static_cast<double>(n["k2"]);
            camera.p1 = static_cast<double>(n["p1"]);
            camera.p2 = static_cast<double>(n["p2"]);

            std::string cam_left_proj_para_name = "cam" + std::to_string(i) + "_left_projection_parameters";
            n = fsSettings[cam_left_proj_para_name];
            camera.fx = static_cast<double>(n["fx"]);
            camera.fy = static_cast<double>(n["fy"]);
            camera.cx = static_cast<double>(n["cx"]);
            camera.cy = static_cast<double>(n["cy"]);

            Cameras.push_back(camera);
        }
        if(STEREO)
        {
            for(int i = 0; i < NUM_OF_CAM; i ++)
            {
                Camera camera;
                std::string cameraName = "camera" + std::to_string(i) + "_right_topic"; 
                fsSettings[cameraName] >> camera.cameraName;
                camera.w = static_cast<int>(fsSettings["image_width"]);
                camera.h = static_cast<int>(fsSettings["image_height"]);
                std::string cam_right_dist_para_name = "cam" + std::to_string(i) + "_right_distortion_parameters";
                cv::FileNode n = fsSettings[cam_right_dist_para_name];
                camera.k1 = static_cast<double>(n["k1"]);
                camera.k2 = static_cast<double>(n["k2"]);
                camera.p1 = static_cast<double>(n["p1"]);
                camera.p2 = static_cast<double>(n["p2"]);

                std::string cam_right_proj_para_name = "cam" + std::to_string(i) + "_right_projection_parameters";
                n = fsSettings[cam_right_proj_para_name];
                camera.fx = static_cast<double>(n["fx"]);
                camera.fy = static_cast<double>(n["fy"]);
                camera.cx = static_cast<double>(n["cx"]);
                camera.cy = static_cast<double>(n["cy"]);

                Cameras.push_back(camera);
                
            }
        }

        if(USE_IMU)
        {
            for(int i = 0; i < NUM_OF_CAM; i ++) 
            {
                std::string imu_topic = "camera" + std::to_string(i) + "_imu_topic";
                std::string IMU_TOPIC;
                fsSettings[imu_topic] >> IMU_TOPIC;
                CAMERA_IMU_TOPIC.push_back(IMU_TOPIC);
                
                cv::Mat cv_T;
                Eigen::Matrix4d T;
                std::string cam_imu_left = "imu_T_cam" + std::to_string(i) + "_left";
                fsSettings[cam_imu_left] >> cv_T;
                cv::cv2eigen(cv_T, T);
                CAM_TIC.push_back(T);

            }
            if(STEREO)
            {
                for(int i = 0; i < NUM_OF_CAM; i ++) 
                {
                    cv::Mat cv_T;
                    Eigen::Matrix4d T;
                    std::string cam_imu_right = "imu_T_cam" + std::to_string(i) + "_right";
                    fsSettings[cam_imu_right] >> cv_T;
                    cv::cv2eigen(cv_T, T);
                    CAM_TIC.push_back(T);
                }
            }
        }

    }

} 