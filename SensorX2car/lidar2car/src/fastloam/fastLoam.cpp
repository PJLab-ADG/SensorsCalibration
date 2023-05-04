#include <fastLoam.h>

void SavePose(std::vector<Eigen::Matrix4d> lidar_pose, std::string outpath){
    std::ofstream file;
    file.open(outpath);
    for (unsigned int i = 0; i < lidar_pose.size(); i++)
    {
        Eigen::Matrix4d T=lidar_pose[i];
        file << T(0, 0) << " " << T(0, 1) << " " << T(0, 2) << " " << T(0, 3) << " "
             << T(1, 0) << " " << T(1, 1) << " " << T(1, 2) << " " << T(1, 3) << " "
             << T(2, 0) << " " << T(2, 1) << " " << T(2, 2) << " " << T(2, 0) << std::endl;
    }
    file.close();   
}

bool RunFastLoam(const std::string dataset_folder, const std::string output_dir, std::vector<Eigen::Matrix4d> &lidar_pose, 
                    int start_frame, int end_frame)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    std::cout << "Reading dataset from " << dataset_folder << std::endl;

    const std::string lidar_path = dataset_folder;
    std::vector<std::string> lidar_files;
    DIR *dir;
    struct dirent *ptr;
    if ((dir = opendir(dataset_folder.c_str())) == NULL)
    {
        printf("Open dir error !");
        exit(1);
    }
    while ((ptr = readdir(dir)) != NULL)
    {
        std::string name = ptr->d_name;
        if (name.size() > 4 && name.substr(name.size() - 4) == ".pcd")
            lidar_files.emplace_back(ptr->d_name);
        ptr++;
    }
    if (lidar_files.size() == 0)
    {
        printf("no file under parsed lidar dir.");
        exit(1);
    }
    closedir(dir);
    std::sort(lidar_files.begin(), lidar_files.end());

    std::vector<std::string> lidar_files_selected;
    for (int i = 0; i < int(lidar_files.size()); i++)
    {
        if(i < start_frame)
            continue;
        else if(i > end_frame)
            break;
        lidar_files_selected.push_back(lidar_files[i]);
    }

    int total_frame = 0;

    // std::vector<Eigen::Quaterniond> pose_q;
    // std::vector<Eigen::Vector3d> pose_t;

    //lidar params
    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period = 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    Lidar lidar_param;
    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    LaserProcessingClass LaserProcessing;
    LaserProcessing.init(lidar_param);
    
    bool is_odom_inited=false;
    OdomEstimationClass odomEstimation;
    odomEstimation.init(lidar_param, map_resolution);
    for (unsigned int i = 0; i < lidar_files_selected.size(); i += 1)
    {
        std::cout << "<-------------------Start running Lidar SLAM" << i
                  << " --------------------->";
                  
        // load point cloud
        std::string pcd_path = lidar_path + '/' + lidar_files_selected[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile(pcd_path, *cloud) < 0)
        {
            std::cout << "cannot open pcd_file: " << lidar_path;
            return 1;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

        LaserProcessing.featureExtraction(cloud, pointcloud_edge, pointcloud_surf);
        std::cout << "edge:" << pointcloud_edge->points.size() << std::endl;
        std::cout << "surf:" << pointcloud_surf->points.size() << std::endl;

        if(is_odom_inited == false){
            odomEstimation.initMapWithPoints(pointcloud_edge, pointcloud_surf);
            is_odom_inited = true;
        }
        else{
            odomEstimation.updatePointsToMap(pointcloud_edge, pointcloud_surf);
        }
        Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
        Eigen::Vector3d t_current = odomEstimation.odom.translation();
        // std::cout << "q_current:" << q_current.coeffs().transpose() << std::endl;
        // std::cout << "t_current:" << t_current.transpose() << std::endl;
        // pose_q.push_back(q_current);
        // pose_t.push_back(t_current);
        Eigen::Matrix4d T_current = Eigen::Matrix4d::Identity();
        T_current.block<3, 3>(0, 0) = q_current.matrix();
        T_current.block<3, 1>(0, 3) = t_current;
        lidar_pose.push_back(T_current);

        total_frame++;
        
    }
    std::string outpath = output_dir + "pose.txt";
    SavePose(lidar_pose, outpath);
    end = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = end - start;
    std::cout << "LidarSLAM cost time:" << elapsed_seconds.count() << "s" << std::endl
              << "---------------------------------------------------" << std::endl;

    return true;
}
