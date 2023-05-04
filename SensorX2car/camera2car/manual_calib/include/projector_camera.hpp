#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

class Projector {
public:
    bool init(cv::Mat img, Eigen::Matrix3d K, std::vector<double> dist)
    {
        img_ = img;
        rows_ = img.rows;
        cols_ = img.cols;
        K_C_ = (cv::Mat_<double>(3, 3) << K(0, 0), K(0, 1), K(0, 2),
                K(1, 0), K(1, 1), K(1, 2),
                K(2, 0), K(2, 1), K(2, 2));
        dist_ = dist;
        return true;
    }

    void setDisplayMode(bool grid_on) {
        grid_on_ = grid_on;
    }

    cv::Mat project(Eigen::Matrix3d rotation)
    {  
        cv::Mat outImg, mapx, mapy;

        cv::Mat R = (cv::Mat_<double>(3, 3) << rotation(0, 0), rotation(1, 0), rotation(2, 0),
                     rotation(0, 1), rotation(1, 1), rotation(2, 1),
                     rotation(0, 2), rotation(1, 2), rotation(2, 2));

        cv::initUndistortRectifyMap(K_C_, dist_, R, K_C_, img_.size(), CV_32FC1, mapx, mapy);
        cv::remap(img_, outImg, mapx, mapy, cv::INTER_LINEAR);

        if (grid_on_){
            cv::Point Pt1(K_C_.at<double>(0, 2), 0);
            cv::Point Pt2(K_C_.at<double>(0, 2), outImg.rows - 1);
            cv::Point Pt3(0, K_C_.at<double>(1, 2));
            cv::Point Pt4(outImg.cols - 1, K_C_.at<double>(1, 2));
            cv::line(outImg, Pt1, Pt2, cv::Scalar(0, 0, 255));
            cv::line(outImg, Pt3, Pt4, cv::Scalar(0, 0, 255));
        }
        return outImg;
    }

private:
    int rows_, cols_;
    cv::Mat img_;
    cv::Mat K_C_;
    std::vector<double> dist_;
    bool grid_on_ = true;
};
