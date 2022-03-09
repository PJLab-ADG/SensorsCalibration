#pragma once
#include<string>
#include <opencv2/opencv.hpp>
namespace calib{
// common parameters
// support: 1 imu 1 cam; 1 imu 2 cam: 2 cam; 
int imu = 1;        
int num_of_cam= 1;  

std::string imu_topic= "/imu0";
std::string image0_topic= "/cam0/image_raw";
std::string image1_topic= "/cam1/image_raw";
std::string output_path= "./output/";

std::string cam0_calib= "cam0_pinhole.yaml";
std::string cam1_calib= "cam1_mei.yaml";
int image_width= 1920;
int image_height= 1200;
   

// Extrinsic parameter between IMU and Camera.
int estimate_extrinsic= 2;   // 0  Have an accurate extrinsic parameters. We will trust the following imu^R_cam, imu^T_cam, don't change it.
                        // 1  Have an initial guess about extrinsic parameters. We will optimize around your initial guess.

int estimate_extrinsict=1; //estimate extrinsic include position
cv::Mat body_T_cam0  = (cv::Mat_<double>(3, 3)<<1, 0, 0, 0.01050946,
                                                0, 0, 1,  1.96065607,
                                                0, -1, 0, 0.97362614,
                                                0, 0, 0, 1); 

// hand-eye calibration parameters
double hand_eye_cov= 0.4;

//Multiple thread support
int multiple_thread= 1;

//feature traker paprameters
int max_cnt= 200;            // max feature number in feature tracking
int min_dist= 30;            // min distance between two features 
int freq= 10;               // frequence (Hz) of publish tracking result. At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
double F_threshold= 0.8;        // ransac threshold (pixel)
int show_track= 1;           // publish tracking image as topic
int flow_back= 1;           // perform forward and backward optical flow to improve feature tracking accuracy

//optimization parameters
double max_solver_time= 0.04;  // max solver itration time (ms), to guarantee real time
int max_num_iterations= 8;   // max solver itrations, to guarantee real time
double keyframe_parallax= 10.0; // keyframe selection threshold (pixel)

//imu parameters       The more accurate parameters you provide, the better performance
double acc_n= 0.1;          // accelerometer measurement noise standard deviation. 
double gyr_n= 0.01;         // gyroscope measurement noise standard deviation.     
double acc_w= 0.001;        // accelerometer bias random work noise standard deviation.  
double gyr_w= 0.0001;       // gyroscope bias random work noise standard deviation.     
double g_norm= 9.81007;     // gravity magnitude

//unsynchronization parameters
int estimate_td= 1;                     // online estimate time offset between camera and imu
double td= 0.0;                             // initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)

//loop closure parameters
int load_previous_pose_graph= 0;       // load and reuse previous pose graph; load from 'pose_graph_save_path'
std::string pose_graph_save_path= "./output/pose_graph/"; // save and load path
bool save_image= 1;                   // save image in pose graph for visualization prupose; you can close this function by setting 0 

}