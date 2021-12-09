#include "IntrinsicCalibration.hpp"
#include "AutoImagePicker.hpp"
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

bool IntrinsicCalibration::Calibrate(const std::string &img_dir_path, 
                                     const int &grid_size, // in milimeter 50mm 
                                     const int &corner_width, 
                                     const int &corner_height) 
{
    // read image files
    std::vector<std::string> file_names;
    std::vector<std::string> selected_file_names;
    if (img_dir_path.rfind('/') != img_dir_path.size() - 1){
        img_dir_path_ = img_dir_path + "/";
    }
    else{
        img_dir_path_ = img_dir_path;
    }
    undistort_image_path_ = img_dir_path_ + "undistorted/";
    selected_image_path_ = img_dir_path_ + "selected/";
    std::cout << "image path: " << img_dir_path_ << std::endl;
    DIR *dir;
    struct dirent *ptr;
    if ((dir = opendir(img_dir_path_.c_str())) == NULL) {
        std::cerr << "[ERROR]Open Image Folder Failed." << std::endl;
        exit(1);
    }
    while ((ptr = readdir(dir)) != NULL) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0 &&
            opendir((img_dir_path_ + ptr->d_name).c_str()) == nullptr)
            file_names.push_back(ptr->d_name);
    }
    if (file_names.size() == 0) {
        std::cerr << "[ERROR]No Calibration Image Files.";
        return false;
    }
    closedir(dir);
    if (opendir(selected_image_path_.c_str()) == nullptr){
        char command[1024];
        sprintf(command, "mkdir -p %s", selected_image_path_.c_str());
        system(command);
        printf("Create dir: %s\n", selected_image_path_.c_str());       
    }

    grid_size_ = grid_size;
    corner_size_ = cv::Size(corner_width, corner_height);
    std::vector<cv::Point3f> object_corners;
    // generate 3D corner points
    for (int i = 0; i < corner_size_.height; i++) {
        for (int j = 0; j < corner_size_.width; j++) {
            object_corners.push_back(cv::Point3f(i, j, 0.0f));
        }
    }

    // pick image
    cv::Mat init_img = cv::imread(img_dir_path_+file_names[0], 0);
    img_size_ = init_img.size();
    AutoImagePicker image_selector(img_size_.width, img_size_.height,
                                   corner_width, corner_height);
    // detect chessboard corner
    int total_image_num = file_names.size();
    int detected_image_num = 0;
    for (int i = 0; i < file_names.size(); i++){
        cv::Mat input_image = cv::imread(img_dir_path_+file_names[i], 0);
        // img_size_ = input_image.size();
        // detect corner
        std::vector<cv::Point2f> image_corners;
        bool whether_found = cv::findChessboardCorners(input_image,
                                                       corner_size_,
                                                       image_corners);
        // refining pixel coordinates for given 2d points
        if (whether_found){
            // whether select this image for calibration
            if (image_selector.addImage(image_corners)){
                std::cout << "Select image " << file_names[i] << std::endl;
                selected_file_names.push_back(file_names[i]);
                cv::imwrite(selected_image_path_+file_names[i], 
                            cv::imread(img_dir_path_+file_names[i]));
                cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 
                                        grid_size_, 0.001);
                cv::cornerSubPix(input_image, image_corners, cv::Size(5, 5), 
                                cv::Size(-1, -1), criteria);
                // check whether the corner detect is complete 
                // if (image_corners.size() == corner_size_.area()) {
                // for (int pt = 0; pt < image_corners.size(); pt++){
                //     if (pt % 15 == 0) std::cout << "==\n";
                //     std::cout << pt%15 << ": " << image_corners[pt].x << ", "
                //                     << image_corners[pt].y << std::endl;
                // }
                addPoints(image_corners, object_corners);
                detected_image_num ++;
                // display corner detect
                cv::drawChessboardCorners(input_image, corner_size_, 
                                        image_corners, whether_found);
                cv::imshow(file_names[i], input_image);
                // cv::waitKey(50);
                cv::waitKey(50);
            }
            if (image_selector.status()) break;
        }
    }
    cv::destroyAllWindows();
    // calibrate the camera
    cv::calibrateCamera(object_points_, image_points_, img_size_,
                        camera_intrinsic_, camera_dist_, R_mats_, t_mats_);
    
    // check min valid num of images
    if (detected_image_num < MIN_CALI_IMAGE_NUM) {
        std::cerr << "[WARNING]Detected image num less than minium requirement.\n";
    }

    // print result
    // std::cout << "\n==>CALIBRATION RESULT\n";
    std::cout << "input " << total_image_num << " images\n";
    std::cout << "detected " << detected_image_num << " images\n";
    std::cout << "\ncamera intrinsic: \n";
    std::cout << camera_intrinsic_.at<double>(0, 0) << "\t" << camera_intrinsic_.at<double>(0, 1) << "\t" << camera_intrinsic_.at<double>(0, 2) << std::endl;
    std::cout << camera_intrinsic_.at<double>(1, 0) << "\t" << camera_intrinsic_.at<double>(1, 1) << "\t" << camera_intrinsic_.at<double>(1, 2) << std::endl;
    std::cout << camera_intrinsic_.at<double>(2, 0) << "\t" << camera_intrinsic_.at<double>(2, 1) << "\t" << camera_intrinsic_.at<double>(2, 2) << std::endl;
    std::cout << "\ndist: \n";
    for (int i = 0; i < camera_dist_.rows; i++)
        for (int j = 0; j < camera_dist_.cols; j++)
            std::cout << camera_dist_.at<double>(i, j) << ", ";
    std::cout << std::endl;
    
    // undistort and save images
    undistortImages(selected_file_names);

    return true;
}

bool IntrinsicCalibration::undistortSingleImage(const std::string &image_path,
    const std::string &output_image_path) 
{
    cv::Mat input_image = cv::imread(image_path);
    if (img_size_ != input_image.size()) {
        std::cerr << "[ERROR]Input Image Size Mismatch Calibration Image Size\n";
        return false;
    }
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(camera_intrinsic_, camera_dist_, cv::Mat(),
                                camera_intrinsic_, img_size_, CV_32FC1, map1, map2);
    cv::Mat undistorted_image;
    cv::remap(input_image, undistorted_image, map1, map2, cv::INTER_LINEAR);         

    cv::imwrite(output_image_path, undistorted_image);
    return true;
}

bool IntrinsicCalibration::undistortImages (const std::vector<std::string> &image_names) 
{
    if (opendir(undistort_image_path_.c_str()) == nullptr){
        char command[1024];
        sprintf(command, "mkdir -p %s", undistort_image_path_.c_str());
        system(command);
        printf("Create dir: %s\n", undistort_image_path_.c_str());       
    }

    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(camera_intrinsic_, camera_dist_, cv::Mat(),
                                camera_intrinsic_, img_size_, CV_32FC1, map1, map2);
    for (int i = 0; i < image_names.size(); i++){
        cv::Mat input_image = cv::imread(img_dir_path_ + image_names[i]);
        std::string output_file_path = undistort_image_path_ + image_names[i];
        cv::Mat undistorted_image;
        cv::remap(input_image, undistorted_image, map1, map2, cv::INTER_LINEAR);         

        cv::imwrite(output_file_path, undistorted_image);
    }
}
