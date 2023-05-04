#include <rpzCalib.h>

using namespace SPLINTER;

RPCalib::RPCalib(std::string output_dir)
{
    output_dir_ = output_dir;
    ground_extractor_.reset(new GroundExtractor);
}

bool RPCalib::LoadData(std::string dataset_folder, const std::vector<Eigen::Matrix4d> &lidar_pose, int start_frame, int end_frame)
{
    std::cout << "Reading dataset from " << dataset_folder << std::endl;
    lidar_path_ = dataset_folder;
    std::vector<std::string> lidar_files;
    DIR *dir;
    struct dirent *ptr;
    
    if ((dir = opendir(dataset_folder.c_str())) == NULL)
    {
        std::cout << "Open dir error !" << std::endl;
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            lidar_files.emplace_back(ptr->d_name);
        ptr++;
    }
    if (lidar_files.size() == 0)
    {
        std::cout << "no file under parsed lidar dir." << std::endl;
        exit(1);
    }
    closedir(dir);
    std::sort(lidar_files.begin(), lidar_files.end());

    for (int i = 0; i < int(lidar_files.size()); i++)
    {
        if(i < start_frame)
            continue;
        else if(i >= end_frame)
            break;
        lidar_files_.push_back(lidar_files[i]);
    }

    file_num_ = lidar_files_.size();

    for (unsigned int i = 0; i < lidar_pose.size(); i++)
    {
        Eigen::Matrix4d T = lidar_pose[i];
        lidar_pose_y_.push_back(T(1, 3));
    }

    return true;
}

bool RPCalib::Calibrate(Eigen::Matrix4d &extrinsic){
    // calib lidar to ground plane
    std::cout << "-----------start calibrating roll and pitch-----------" << std::endl;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    std::vector<double> trans, rollset, pitchset;    

    for (unsigned int i = 0; i < lidar_files_.size(); i += down_sample_)
    {
        std::cout << "filename: " <<lidar_files_[i] << std::endl;

        // load pcd
        std::string pcd_path = lidar_path_ + '/' + lidar_files_[i];
        PointCloudPtr cloud(new PointCloud());
        if (pcl::io::loadPCDFile(pcd_path, *cloud) < 0)
        {
            std::cout << "cannot open pcd_file: " << pcd_path << std::endl;
            exit(1);
        }
        
        // min range filter
        PointCloudPtr cloud_out(new PointCloud());
        PCUtil::MinRangeFilter(cloud, filter_min_range_, cloud_out);

        // max range filter
        PointCloudPtr master_cloud(new PointCloud());
        PCUtil::MaxRangeFilter(cloud_out, filter_master_max_range_, master_cloud);
        
        
        PlaneParam master_gplane;
        PointCloudPtr master_gcloud(new PointCloud());
        PointCloudPtr master_ngcloud(new PointCloud());

        // pcl
        // GroundPlaneExtraction(cloud, master_gcloud, master_ngcloud, master_gplane);

        // rr method
        ground_extractor_->RandomRansacFitting(
            master_cloud, master_gcloud, master_ngcloud, &master_gplane);
        
        Eigen::Vector3d master_z(0, 0, 1);
        Eigen::Vector3d rot_axis = master_gplane.normal.cross(master_z);
        rot_axis.normalize();
        double alpha = -std::acos(master_gplane.normal.dot(master_z));
        
        // extrinsic: plane to master lidar
        Eigen::Matrix3d R_mp;
        R_mp = Eigen::AngleAxisd(alpha, rot_axis);
        Eigen::Vector3d t_mp(0, 0,
                         -master_gplane.intercept / master_gplane.normal(2));
        Eigen::Matrix4d T_pm = Util::GetMatrix(t_mp, R_mp).inverse();

        double roll = Util::GetRoll(T_pm);
        double pitch = Util::GetPitch(T_pm);
        std::cout << "roll = " << rad2deg(roll) << " degree" << std::endl;
        std::cout << "pitch = " << rad2deg(pitch) << " degree" << std::endl;
        std::cout << "t = (" << t_mp(0) << ", " << t_mp(1) << ", " << t_mp(2) << ")" << std::endl;
        
        trans.push_back(-t_mp(2));
        rollset.push_back(roll);
        pitchset.push_back(pitch);
    }

    if(trans.size() <= 5)
        return false;
    std::vector<double> newrollset, newpitchset, newtrans;
    Util::DeleteOutliers(rollset, newrollset, 5);
    Util::DeleteOutliers(pitchset, newpitchset, 5);
    Util::DeleteOutliers(trans, newtrans, 5);

    double roll_mean = Util::Mean(newrollset);
    double pitch_mean = Util::Mean(newpitchset);
    double trans_mean = Util::Mean(newtrans);
    // double roll_std = Util::Std(rollset);
    // double pitch_std = Util::Std(pitchset);
    // double trans_std = Util::Std(trans);

    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(pitch_mean, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll_mean, Eigen::Vector3d::UnitX());

    extrinsic.block<3, 3>(0, 0) = rot;
    extrinsic(2, 3) = trans_mean;

    std::cout << "Using " << rollset.size() << " frames from " << lidar_files_.size() << " lidar frames in total." << std::endl;
    end = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = end - start;
    std::cout << "cost time: " << elapsed_seconds.count() << "s" << std::endl;
    // std::cout << "roll range: (" << rad2deg(*min_element(rollset.begin(), rollset.end())) << ", "
    //           << rad2deg(*max_element(rollset.begin(), rollset.end())) << ") " << std::endl;
    // std::cout << "pitch range: (" << rad2deg(*min_element(pitchset.begin(), pitchset.end())) << ", " 
    //           << rad2deg(*max_element(pitchset.begin(), pitchset.end())) << ") " << std::endl;

    return true;
}

bool RPCalib::GroundPlaneExtraction(const PointCloudPtr &in_cloud,
                                    PointCloudPtr g_cloud,
                                    PointCloudPtr ng_cloud, PlaneParam &plane) 
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.03);
    seg.setMaxIterations(2000);
    seg.setInputCloud(in_cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return false;
    }
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(in_cloud);
    extract.setIndices(inliers);
    extract.filter(*g_cloud);
    extract.setNegative(true);
    extract.filter(*ng_cloud);
    plane.normal(0) = coefficients->values[0];
    plane.normal(1) = coefficients->values[1];
    plane.normal(2) = coefficients->values[2];
    plane.intercept = coefficients->values[3];
    return true;
}

RPCalib::~RPCalib() {}
