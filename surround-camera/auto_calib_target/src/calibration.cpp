#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>

using namespace std;
using namespace cv;

void InitializeK(Mat &K_F,Mat &K_L,Mat &K_B,Mat &K_R){
    K_F=(Mat_<double>(3,3)<<390.425287,   0, 750,
                            0,  390.425287,  750, 
                            0,0,1);
    K_L=(Mat_<double>(3,3)<<390.425287,   0, 750,
                            0,  390.425287,  750, 
                            0,0,1);
    K_B=(Mat_<double>(3,3)<<390.425287,   0, 750,
                            0,  390.425287,  750, 
                            0,0,1);
    K_R=(Mat_<double>(3,3)<<390.425287,   0, 750,
                            0,  390.425287,  750, 
                            0,0,1);
    return;
}

void InitializeD(Mat &D_F,Mat &D_L,Mat &D_B,Mat &D_R){
    D_F=(Mat_<double>(5,1)<<0.0,0.0,0.0,0.0,0.0);
    D_L=(Mat_<double>(5,1)<<0.0,0.0,0.0,0.0,0.0);
    D_B=(Mat_<double>(5,1)<<0.0,0.0,0.0,0.0,0.0);
    D_R=(Mat_<double>(5,1)<<0.0,0.0,0.0,0.0,0.0);
    return;
}

void Initialize3dPoints(vector<Point3f> &pts_3d_F,vector<Point3f> &pts_3d_L,vector<Point3f> &pts_3d_B,vector<Point3f> &pts_3d_R){
    
    pts_3d_F.push_back(Point3f(3.5, 0.12, 0.36)); 
    pts_3d_F.push_back(Point3f(3.5, -0.12, 0.36)); 
    pts_3d_F.push_back(Point3f(3.5, 0.36, 0.12)); 
    pts_3d_F.push_back(Point3f(3.5, 0.12, 0.12)); 
    pts_3d_F.push_back(Point3f(3.5, -0.12, 0.12)); 
    pts_3d_F.push_back(Point3f(3.5, -0.36, 0.12)); 
    pts_3d_F.push_back(Point3f(3.5, 0.36, -0.12));   
    pts_3d_F.push_back(Point3f(3.5, 0.12, -0.12));   
    pts_3d_F.push_back(Point3f(3.5, -0.12, -0.12));   
    pts_3d_F.push_back(Point3f(3.5, -0.36, -0.12));  
    pts_3d_F.push_back(Point3f(3.5, 0.12, -0.36)); 
    pts_3d_F.push_back(Point3f(3.5, -0.12, -0.36));

    

    pts_3d_L.push_back(Point3f(-0.12,2.0,0.36));
    pts_3d_L.push_back(Point3f(0.12,2.0,0.36));
    pts_3d_L.push_back(Point3f(-0.36,2.0,0.12));
    pts_3d_L.push_back(Point3f(-0.12,2.0,0.12));
    pts_3d_L.push_back(Point3f(0.12,2.0,0.12));
    pts_3d_L.push_back(Point3f(0.36,2.0,0.12));
    pts_3d_L.push_back(Point3f(-0.36,2.0,-0.12));
    pts_3d_L.push_back(Point3f(-0.12,2.0,-0.12));
    pts_3d_L.push_back(Point3f(0.12,2.0, -0.12));
    pts_3d_L.push_back(Point3f(0.36,2.0, -0.12));
    pts_3d_L.push_back(Point3f(-0.12,2.0,-0.36));
    pts_3d_L.push_back(Point3f(0.12,2.0,-0.36));

    
    pts_3d_B.push_back(Point3f(-3.0,-0.12,0.36)); 
    pts_3d_B.push_back(Point3f(-3.0,0.12,0.36)); 
    pts_3d_B.push_back(Point3f(-3.0,-0.36,0.12)); 
    pts_3d_B.push_back(Point3f(-3.0,-0.12,0.12)); 
    pts_3d_B.push_back(Point3f(-3.0,0.12,0.12)); 
    pts_3d_B.push_back(Point3f(-3.0,0.36,0.12)); 
    pts_3d_B.push_back(Point3f(-3.0,-0.36,-0.12)); 
    pts_3d_B.push_back(Point3f(-3.0,-0.12,-0.12)); 
    pts_3d_B.push_back(Point3f(-3.0,0.12, -0.12)); 
    pts_3d_B.push_back(Point3f(-3.0,0.36, -0.12));  
    pts_3d_B.push_back(Point3f(-3.0,-0.12,-0.36)); 
    pts_3d_B.push_back(Point3f(-3.0,0.12,-0.36)); 

    
    pts_3d_R.push_back(Point3f(0.12,  -2.0, 0.36)); 
    pts_3d_R.push_back(Point3f(-0.12, -2.0, 0.36)); 
    pts_3d_R.push_back(Point3f(0.36,  -2.0, 0.12)); 
    pts_3d_R.push_back(Point3f(0.12,  -2.0, 0.12)); 
    pts_3d_R.push_back(Point3f(-0.12, -2.0, 0.12)); 
    pts_3d_R.push_back(Point3f(-0.36, -2.0, 0.12)); 
    pts_3d_R.push_back(Point3f(0.36,  -2.0, -0.12)); 
    pts_3d_R.push_back(Point3f(0.12,  -2.0, -0.12)); 
    pts_3d_R.push_back(Point3f(-0.12, -2.0, -0.12)); 
    pts_3d_R.push_back(Point3f(-0.36, -2.0, -0.12)); 
    pts_3d_R.push_back(Point3f(0.12,  -2.0, -0.36)); 
    pts_3d_R.push_back(Point3f(-0.12, -2.0, -0.36));

    return;
}

void Initialize2dPoints(vector<Point2f> &pts_2d_F,vector<Point2f> &pts_2d_L,vector<Point2f> &pts_2d_B,vector<Point2f> &pts_2d_R){
    pts_2d_F.push_back(cv::Point2f(708,765));
    pts_2d_F.push_back(cv::Point2f(802,765));
    pts_2d_F.push_back(cv::Point2f(615,859));
    pts_2d_F.push_back(cv::Point2f(708,859));
    pts_2d_F.push_back(cv::Point2f(802,859));
    pts_2d_F.push_back(cv::Point2f(896,859));
    pts_2d_F.push_back(cv::Point2f(615,952));
    pts_2d_F.push_back(cv::Point2f(709,952));
    pts_2d_F.push_back(cv::Point2f(802,952));
    pts_2d_F.push_back(cv::Point2f(896,952));
    pts_2d_F.push_back(cv::Point2f(708,1046));
    pts_2d_F.push_back(cv::Point2f(802,1046));

     
    pts_2d_L.push_back(cv::Point2f(703,771)); 
    pts_2d_L.push_back(cv::Point2f(796,771)); 
    pts_2d_L.push_back(cv::Point2f(609,864)); 
    pts_2d_L.push_back(cv::Point2f(703,864)); 
    pts_2d_L.push_back(cv::Point2f(796,864)); 
    pts_2d_L.push_back(cv::Point2f(890,864));
    pts_2d_L.push_back(cv::Point2f(609,958)); 
    pts_2d_L.push_back(cv::Point2f(703,958)); 
    pts_2d_L.push_back(cv::Point2f(796,958)); 
    pts_2d_L.push_back(cv::Point2f(890,958)); 
    pts_2d_L.push_back(cv::Point2f(703,1051)); 
    pts_2d_L.push_back(cv::Point2f(796,1051)); 


    pts_2d_B.push_back(cv::Point2f(697,765));
    pts_2d_B.push_back(cv::Point2f(791,765));
    pts_2d_B.push_back(cv::Point2f(603,859)); 
    pts_2d_B.push_back(cv::Point2f(697,859)); 
    pts_2d_B.push_back(cv::Point2f(791,859)); 
    pts_2d_B.push_back(cv::Point2f(885,859)); 
    pts_2d_B.push_back(cv::Point2f(603,953)); 
    pts_2d_B.push_back(cv::Point2f(697,953)); 
    pts_2d_B.push_back(cv::Point2f(791,953)); 
    pts_2d_B.push_back(cv::Point2f(885,953)); 
    pts_2d_B.push_back(cv::Point2f(697,1047)); 
    pts_2d_B.push_back(cv::Point2f(791,1047)); 


    pts_2d_R.push_back(cv::Point2f(702,759)); 
    pts_2d_R.push_back(cv::Point2f(797,759)); 
    pts_2d_R.push_back(cv::Point2f(609,853)); 
    pts_2d_R.push_back(cv::Point2f(702,853)); 
    pts_2d_R.push_back(cv::Point2f(797,853)); 
    pts_2d_R.push_back(cv::Point2f(890,853)); 
    pts_2d_R.push_back(cv::Point2f(609,944)); 
    pts_2d_R.push_back(cv::Point2f(703,944)); 
    pts_2d_R.push_back(cv::Point2f(796,944)); 
    pts_2d_R.push_back(cv::Point2f(890,944)); 
    pts_2d_R.push_back(cv::Point2f(703,1041)); 
    pts_2d_R.push_back(cv::Point2f(796,1041)); 

    return;
}

int main(int argc,char **argv) {
    
    //camera intrinsic
    Mat K_F,K_L,K_B,K_R; 
    InitializeK(K_F,K_L,K_B,K_R);
    Mat D_F,D_L,D_B,D_R;
    InitializeD(D_F,D_L,D_B,D_R);

    //rotation and translation
    Mat rvecs_F,rvecs_L,rvecs_B,rvecs_R;
    Mat rotation_F,rotation_L,rotation_B,rotation_R;
    Mat translation_F,translation_L,translation_B,translation_R;
    
    Mat T_F,T_L,T_B,T_R;

    //3D coordinates of target points in vehicle coordinate system
    vector<Point3f> pts_3d_F,pts_3d_L,pts_3d_B,pts_3d_R; 
    //matched points in images 
    vector<Point2f> pts_2d_F,pts_2d_L,pts_2d_B,pts_2d_R;
    
    Initialize3dPoints(pts_3d_F,pts_3d_L,pts_3d_B,pts_3d_R);
    Initialize2dPoints(pts_2d_F,pts_2d_L,pts_2d_B,pts_2d_R);

    //calculate extrinsic
    solvePnP(pts_3d_F, pts_2d_F, K_F, D_F, rvecs_F, translation_F);
    Rodrigues(rvecs_F, rotation_F); 
    hconcat(rotation_F,translation_F,T_F);
    cout<<"T_F:"<<T_F<<endl;
    
    solvePnP(pts_3d_L, pts_2d_L, K_L, D_L, rvecs_L, translation_L);
    Rodrigues(rvecs_L, rotation_L); 
    hconcat(rotation_L,translation_L,T_L);
    cout<<"T_L:"<<T_L<<endl;

    solvePnP(pts_3d_B, pts_2d_B, K_B, D_B, rvecs_B, translation_B);
    Rodrigues(rvecs_B, rotation_B); 
    hconcat(rotation_B,translation_B,T_B);
    cout<<"T_B:"<<T_B<<endl;

    solvePnP(pts_3d_R, pts_2d_R, K_R, D_R, rvecs_R, translation_R);
    Rodrigues(rvecs_R, rotation_R); 
    hconcat(rotation_R,translation_R,T_R);
    cout<<"T_R:"<<T_R<<endl;

    return 0;
}
