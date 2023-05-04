#include"lidar.h"



void Lidar::setLines(double num_lines_in){
    num_lines=num_lines_in;
}


void Lidar::setVerticalAngle(double vertical_angle_in){
    vertical_angle = vertical_angle_in;
}


void Lidar::setVerticalResolution(double vertical_angle_resolution_in){
    vertical_angle_resolution = vertical_angle_resolution_in;
}


void Lidar::setScanPeriod(double scan_period_in){
    scan_period = scan_period_in;
}


void Lidar::setMaxDistance(double max_distance_in){
	max_distance = max_distance_in;
}

void Lidar::setMinDistance(double min_distance_in){
	min_distance = min_distance_in;
}