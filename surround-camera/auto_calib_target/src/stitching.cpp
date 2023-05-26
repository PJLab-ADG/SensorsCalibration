/*
 * Copyright (C) 2022 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include <fstream>
#include <iostream>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cstdlib>

using namespace std;
using namespace cv;

void initializeK(Eigen::Matrix3d& K_F, Eigen::Matrix3d& K_L,
				 Eigen::Matrix3d& K_B, Eigen::Matrix3d& K_R)
{
	K_F<<390.425287,  0.00000000, 750,
		0.00000000, 390.425287,   750,
		0.00000000, 0.00000000, 1.00000000;

	K_L<<390.425287,  0.00000000, 750,
		0.00000000, 390.425287,   750,
		0.00000000, 0.00000000, 1.00000000;

	K_B<<390.425287,  0.00000000, 750,
		0.00000000, 390.425287,   750,
		0.00000000, 0.00000000, 1.00000000;

	K_R<<390.425287,  0.00000000, 750,
		0.00000000, 390.425287,   750,
		0.00000000, 0.00000000, 1.00000000;
	return;
}

void initializeD(Eigen::Vector4d& D_F, Eigen::Vector4d& D_L,
				 Eigen::Vector4d& D_B, Eigen::Vector4d& D_R)
{
	D_F<<0 ,     0,     0,     0;
	D_L<<0 ,     0,     0,     0;
	D_B<<0 ,     0,     0,     0;
	D_R<<0 ,     0,     0,     0;
	return;
}

void initializeExtrinsic(Eigen::Matrix4d& T_FG, Eigen::Matrix4d& T_LG,
				 Eigen::Matrix4d& T_BG, Eigen::Matrix4d& T_RG)
{

	Eigen::Matrix4d T_CG;
	//transformation matrix from bev to vehicle
	T_CG<< 0,  -1,  0,  0,
	       -1,  0,  0, 0,			 
		   0,  0, -1,  1.5,			//the height between car-center and bev is 1.5m 
		   0,  0,  0,  1;

	T_FG<<0.003549662439087387, -0.9999932306665611, -0.0009687711918229813, 0.0008429940074524061,
 		-0.002167111598488647, 0.0009610824607919066, -0.9999971899699635, 0.4061947515423991,
 		0.9999913517245478, 0.003551751899715572, -0.00216368541027584, -2.499436724456814,
		0,0,0,1;
	T_FG=T_FG*T_CG;

	T_LG<<0.9999999999917426, -3.755851846588426e-06, 1.551837621024431e-06, -0.001275392309651035,
 		1.547292026896347e-06, -0.001209967368988627, -0.999999267988018, 0.41564252503368,
 		3.75772677014328e-06, 0.9999992679821618, -0.001209967363167236, -0.9980876060012519,
		0,0,0,1;
	T_LG=T_LG*T_CG;

	T_BG<<1.110223024625157e-16, 1, 0, -0.01531914937196414,
 		-2.220446049250313e-16, 1.110223024625157e-16, -1, 0.3982978836710719,
 		-1, 0, 3.33066907387547e-16, -2.003169451642492,
		0,0,0,1;
	T_BG=T_BG*T_CG;		

	T_RG<<-0.9999999999263908, 1.205703480472718e-05, -1.358778890488407e-06, -0.001258799042720178,
 		1.290415426966694e-06, -0.00566819593876311, -0.9999839356475356, 0.370861686147207,
 		-1.206454294125915e-05, -0.9999839355756811, 0.005668195922787556, -0.9977066460653484,
 		0,0,0,1;
	T_RG=T_RG*T_CG;

	return;
}

void ProjectPoints(cv::Mat &P_GC1,cv::Mat &p_GC,Eigen::Matrix3d &K_C)
{
    for(int i=0;i<P_GC1.cols;i++){
		//cout<<"i"<<i<<endl;
 
		double x=P_GC1.at<cv::Vec2d>(0,i)[0];
		double y=P_GC1.at<cv::Vec2d>(0,i)[1];
		
		double u=x*K_C(0,0)+K_C(0,2);
		double v=y*K_C(1,1)+K_C(1,2);
		

		p_GC.at<cv::Vec2d>(0,i)[0]=u;
		p_GC.at<cv::Vec2d>(0,i)[1]=v;

	}

}

cv::Mat eigen2mat(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A)
{
	cv::Mat B;
	cv::eigen2cv(A,B);
	
	return B;
}

cv::Mat project_on_ground(cv::Mat img, Eigen::Matrix4d T_CG,
						  Eigen::Matrix3d K_C,Eigen::Vector4d D_C,
						  cv::Mat K_G,int rows, int cols)
{
	cv::Mat p_G = cv::Mat::ones(3,rows*cols,CV_64FC1);  
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			p_G.at<double>(0,cols*i+j) = j;
			p_G.at<double>(1,cols*i+j) = i;
		}
	}
	cv::Mat P_G = cv::Mat::ones(4,rows*cols,CV_64FC1);
	float bevHeight=2.1;  
	P_G(cv::Rect(0,0,rows*cols,3)) = K_G.inv()*p_G*bevHeight; 
	cv::Mat P_GC = cv::Mat::zeros(4,rows*cols,CV_64FC1);
	cv::Mat T_CG_(4,4,CV_64FC1);
	cv::eigen2cv(T_CG,T_CG_);
	P_GC =  T_CG_ * P_G;
	cv::Mat P_GC1 = cv::Mat::zeros(1,rows*cols,CV_64FC2);
	vector<cv::Mat> channels(2);
	cv::split(P_GC1, channels);
	channels[0] = P_GC(cv::Rect(0,0,rows*cols,1))/P_GC(cv::Rect(0,2,rows*cols,1));
	channels[1] = P_GC(cv::Rect(0,1,rows*cols,1))/P_GC(cv::Rect(0,2,rows*cols,1));
	cv::merge(channels, P_GC1);
	cv::Mat p_GC = cv::Mat::zeros(1,rows*cols,CV_64FC2);
	vector<double> D_C_{D_C(0,0),D_C(1,0),D_C(2,0),D_C(3,0)};
	ProjectPoints(P_GC1,p_GC,K_C);
	p_GC.reshape(rows,cols);
	cv::Mat p_GC_table = p_GC.reshape(0,rows);
	vector<cv::Mat> p_GC_table_channels(2);
	cv::split(p_GC_table, p_GC_table_channels);
	cv::Mat p_GC_table_32F;
	p_GC_table.convertTo(p_GC_table_32F,CV_32FC2);
	cv::Mat img_GC;
	cv::remap(img,img_GC,p_GC_table_32F,cv::Mat(),cv::INTER_LINEAR);
	return img_GC;
}

void generate_surround_view(cv::Mat surround_view,
		cv::Mat img_F, cv::Mat img_L, cv::Mat img_B, cv::Mat img_R, 
		Eigen::Vector4d D_F, Eigen::Vector4d D_L, Eigen::Vector4d D_B, Eigen::Vector4d D_R,
		Eigen::Matrix4d T_FG,Eigen::Matrix4d T_LG,Eigen::Matrix4d T_BG,Eigen::Matrix4d T_RG, 
		Eigen::Matrix3d K_F,Eigen::Matrix3d K_L,Eigen::Matrix3d K_B,Eigen::Matrix3d K_R){

	// project images on the ground
	int rows = 1500;		
	int cols = 1500;
	double dX = 0.005;		
	double dY = 0.005;
	cv::Mat K_G = cv::Mat::zeros(3,3,CV_64FC1);
	K_G.at<double>(0,0) = 1/dX;
	K_G.at<double>(1,1) = 1/dY;
	K_G.at<double>(0,2) = cols/2;
	K_G.at<double>(1,2) = rows/2;
	K_G.at<double>(2,2) =   1.0;

	cv::Mat img_GF = project_on_ground(img_F,T_FG,K_F,D_F,K_G,rows,cols);
	cv::Mat img_GL = project_on_ground(img_L,T_LG,K_L,D_L,K_G,rows,cols);
	cv::Mat img_GB = project_on_ground(img_B,T_BG,K_B,D_B,K_G,rows,cols);
	cv::Mat img_GR = project_on_ground(img_R,T_RG,K_R,D_R,K_G,rows,cols);
	cv::imwrite("img_GF.png",img_GF);
	cv::imwrite("img_GL.png",img_GL);
	cv::imwrite("img_GB.png",img_GB);
	cv::imwrite("img_GR.png",img_GR);

	cv::Mat dst1,dst2,dst3;

	for(int row=540;row<1500;row++)
        for(int col=0;col<1500;col++)
            img_GF.at<cv::Vec3b>(row,col) = cv::Vec3b(0,0,0);	

    for(int col=700;col<1500;col++)
        for(int row=0;row<1500;row++)
            img_GL.at<cv::Vec3b>(row,col) = cv::Vec3b(0,0,0);
		
    for(int row=0;row<900;row++)
        for(int col=0;col<1500;col++)
            img_GB.at<cv::Vec3b>(row,col) = cv::Vec3b(0,0,0);

    for(int row=0;row<1500;row++)
        for(int col=0;col<810;col++)
            img_GR.at<cv::Vec3b>(row,col) = cv::Vec3b(0,0,0);

	addWeighted(img_GF,0.5,img_GL,0.5,3,dst1);
	addWeighted(dst1,1.0,img_GB,0.5,3,dst2);
	addWeighted(dst2,1.0,img_GR,0.5,3,dst3);
	cv::imwrite("surround_view.png",dst3);
	imshow("surround_view", dst3);
	waitKey(0);

}

int main(int argc, char **argv) {
  
	//camera intrinsic
	Eigen::Matrix3d K_F,K_L,K_B,K_R;
	initializeK(K_F, K_L, K_B, K_R);
	Eigen::Vector4d D_F,D_L,D_B,D_R;
	initializeD(D_F, D_L, D_B, D_R);

	// camera extrinsic
	Eigen::Matrix4d T_FG,T_LG,T_BG,T_RG;
	initializeExtrinsic(T_FG, T_LG, T_BG, T_RG);
	
	// load images
	cv::Mat img_F = cv::imread("imgs/front_125.png");
	cv::Mat img_L = cv::imread("imgs/left_125.png");
	cv::Mat img_B = cv::imread("imgs/behind_125.png");
	cv::Mat img_R = cv::imread("imgs/right_125.png");
    
	//surrounding stitching
	cv::Mat surround_view;
	generate_surround_view(surround_view,
		img_F, img_L, img_B, img_R,
		D_F,D_L,D_B,D_R, 
		T_FG,T_LG,T_BG,T_RG, 
		K_F,K_L,K_B,K_R);

  	return 0;
}