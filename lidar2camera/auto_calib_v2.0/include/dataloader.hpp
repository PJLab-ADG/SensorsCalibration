#pragma once
#include <utility.hpp>

class DataLoader {
    public:
        DataLoader() = default;
        ~DataLoader() = default;

        static void LoadMaskFile(const std::string mask_dir, const Eigen::MatrixXf intrinsic, const std::vector<double> dist, cv::Mat &masks, std::vector<int> &mask_point_num)
        {
            std::vector<std::string> mask_files;
            DIR *dir;
            struct dirent *ptr;
            if ((dir = opendir(mask_dir.c_str())) == NULL)
            {
                std::cout << "Open dir " << mask_dir << " error !" << std::endl;
                exit(1);
            }
            while ((ptr = readdir(dir)) != NULL)
            {
                std::string name = ptr->d_name;
                if (name.size() > 4 && name.substr(name.size() - 4) == ".png")
                    mask_files.emplace_back(ptr->d_name);
                ptr++;
            }
            if (mask_files.size() == 0)
            {
                std::cout << "No valid mask file under dir." << std::endl;
                exit(1);
            }
            sort(mask_files.begin(), mask_files.end());
            
            cv::Mat mask;
            int N = mask_files.size();
            if (N > 255){
                std::cout << "Mask file number is larger than 255. Redundant masks are discarded." << std::endl;
                N = 255;
            }
            for (int n = 0; n < N; n++)
            {
                std::string file = mask_files[n];
                mask = cv::imread(mask_dir + '/' + file, cv::IMREAD_GRAYSCALE);
                assert(mask.size() == masks.size());
                if(intrinsic.cols() == 3)
                {
                    Util::UndistImg(mask, intrinsic, dist);
                }
                int n_white = 0;
                auto it2 = masks.begin<cv::Vec4b>();
                for (auto it = mask.begin<uchar>(); it != mask.end<uchar>(); it++)
                {
                    if (*it == 255)
                    {
                        n_white++;
                        for (int c = 0; c < 4; c++)
                        {
                            if ((*it2)[c] == 0)
                            {
                                (*it2)[c] = n + 1;
                                break;
                            }
                        }
                    }
                    it2++;
                }
                // std::cout << n_white << std::endl;
                mask_point_num.push_back(n_white);
            }
        }

        static void LoadCalibFile(
            const std::string filename, 
            Eigen::MatrixXf& intrinsic, 
            Eigen::Matrix4f& extrinsic,
            std::vector<double>& dist) 
        {
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cout << "open file " << filename << " failed." << std::endl;
                exit(1);
            }
            std::string line, tmpStr;
            getline(file, line);
            std::stringstream ss(line);
            std::vector<float> elements;
            std::string elem;
            getline(ss, elem, ' ');
            while (getline(ss, elem, ' '))
            {
                elements.emplace_back(stof(elem));
            }
            if(elements.size() == 9)
            {
                intrinsic = Eigen::Map<Eigen::MatrixXf>(elements.data(), 3, 3).transpose();
            }
            else if(elements.size() == 12)
            {
                intrinsic = Eigen::Map<Eigen::MatrixXf>(elements.data(), 4, 3).transpose();
            }
            else{
                std::cout << "Wrong intrinsic parameter number." << std::endl;
                exit(1);
            }

            getline(file, line);
            ss = std::stringstream(line);
            getline(ss, elem, ' ');
            while (getline(ss, elem, ' '))
            {
                dist.emplace_back(stod(elem));
            }

            getline(file, line);
            ss = std::stringstream(line);
            ss >> tmpStr >> extrinsic(0, 0) >> extrinsic(0, 1) >> extrinsic(0, 2) >> extrinsic(0, 3)
                >> extrinsic(1, 0) >> extrinsic(1, 1) >> extrinsic(1, 2) >> extrinsic(1, 3)
                >> extrinsic(2, 0) >> extrinsic(2, 1) >> extrinsic(2, 2) >> extrinsic(2, 3);
        }


        static void LoadLidarFile(const std::string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
        {
            if (pcl::io::loadPCDFile(filename, *pc) < 0)
            {
                std::cout << "[ERROR] cannot open pcd_file: " << filename << "\n";
                exit(1);
            }
        }

    private:
};