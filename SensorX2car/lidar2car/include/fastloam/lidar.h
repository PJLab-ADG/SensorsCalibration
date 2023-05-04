#ifndef _LIDAR_H_
#define _LIDAR_H_

//define lidar parameter


class Lidar
{
    public:
        Lidar(){};

        void setScanPeriod(double scan_period_in);
        void setLines(double num_lines_in);
        void setVerticalAngle(double vertical_angle_in);
        void setVerticalResolution(double vertical_angle_resolution_in);
        //by default is 100. pls do not change
        void setMaxDistance(double max_distance_in);
        void setMinDistance(double min_distance_in);

    	double max_distance;
        double min_distance;
        int num_lines;
        double scan_period;
        int points_per_line;
        double horizontal_angle_resolution;
        double horizontal_angle;
        double vertical_angle_resolution;
        double vertical_angle;
};




#endif // _LIDAR_H_
