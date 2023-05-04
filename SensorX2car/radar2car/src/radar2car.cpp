#include "radar2car.hpp"
#include "math.h"
#include <cmath>
#include <algorithm>

namespace autoCalib {
namespace calibration {

double RadarCalibrator::calib(const std::string &radar_file_dir,
                            const std::string &radar_type,
                            const std::string &novatel_enu,
                            const RadarCalibParam &param,
                            bool guessRoughYaw,
                            bool savefig)
{
    // set parameters
    angle_thresh_ = 1.5 * DEG2RAD;

    min_velocity_ = param.min_velocity;
    max_velocity_ = param.max_velocity;
    // load data from radar file dir and align timestamp with novatel speed
    RadarDataLoader data_loader;
    std::vector<RadarObject> radar_datas;
    // data_loader.setMinVelocity(min_velocity_*(1-ve_diff_range_));
    data_loader.getRadarCalibData(radar_file_dir, radar_type,
        novatel_enu, radar_datas, param.start_sec, param.max_sec);
    std::cout << "Read " << radar_datas.size() << " objects\n";

    int min_gap = 3;
    min_gap_ = min_gap;
    double all_yaw = 0;
    double conf = 0;

    for (size_t i = 0; i < radar_datas.size(); ++i)
    {
        double tmp_yaw = 0;
        double tmp_conf = 0;
        int n = 0;
        int object_size = radar_datas[i].timestamps.size();
        if (object_size < min_gap + 1) continue;
        if (whetherStatic(radar_datas[i])) {

            for (int ti = min_gap; ti < object_size; ti++)
            {
                double yaw_deg, confidence;
                if (getSingleYaw(radar_datas[i], 0, ti, yaw_deg, confidence))
                {
                    tmp_yaw += yaw_deg * confidence;
                    tmp_conf += confidence;
                    n++;
                }
            }

            if (n != 0) {
                printf("Object %d: %d pts, yaw(aver)=%.6f, confidence(aver)=%3f\n", i, object_size,
                        tmp_yaw / tmp_conf, tmp_conf / n);
                all_yaw += tmp_yaw;
                conf += tmp_conf;
            }
        }
    }

    printf("Final yaw estimation: %.6f degree\n", all_yaw / conf);
    return all_yaw / conf;
}

bool RadarCalibrator::whetherStatic(const RadarObject &object)
{
    int object_size = object.timestamps.size();
    double min_phi = 5 * DEG2RAD;

    // check car movement
    double dist = object.dist(0, object_size - 1);
    double timegap = object.timeGap(0, object_size - 1);
    double aver_velo = dist / timegap;
    if (aver_velo < min_velocity_) return false;
    // check triangle angle
    double a = object.track_range[0];
    double phi0 = object.track_angle[0];
    double a_square = a * a;

    if(fabs(phi0) < min_phi) return false;
    for (int i = min_gap_; i < object_size; i ++) {
        double b = object.track_range[i];
        double c = object.dist(0, i);
        double gama = acos((a_square + b * b - c * c) / (2 * a * b));
        if (std::isnan(gama))
            return false;
        double phi1 = object.track_angle[i];
        double angle_diff = std::min(fabs(phi0 + gama - phi1),
                                     fabs(phi0 - gama - phi1));
        if (angle_diff > angle_thresh_) return false;
        // if (gama < angle_tresh) return false;
    }

    return true;
}

bool RadarCalibrator::getSingleYaw(const RadarObject &object, 
                                   int i0,
                                   int i1, 
                                   double &yaw_deg,
                                   double &confidence)
{
    double phi0 = object.track_angle[i0];
    double phi1 = object.track_angle[i1];
    double a = object.track_range[i0];
    double b = object.track_range[i1];
    double c = object.dist(i0, i1);
    if(a > 50 || a / c > 20)
        return false;
    double beta = acos((c * c + b * b - a * a) / (2 * c * b));
    double alpha = acos((c * c + a * a - b * b) / (2 * c * a));
    double gama = acos((b * b + a * a - c * c) / (2 * a * b));


    double yaw_deg1, yaw_deg2;
    // right or left side
    if (fabs(phi0 + gama - phi1) <= fabs(phi0 - gama - phi1)) {
        // left
        yaw_deg1 = (M_PI - phi1 - beta) * RAD2DEG;
        yaw_deg2 = (- phi0 + alpha) * RAD2DEG;
        yaw_deg = (yaw_deg1 + yaw_deg2) / 2;
    }
    else {
        yaw_deg1 = (beta - M_PI - phi1) * RAD2DEG;
        yaw_deg2 = (- phi0 - alpha) * RAD2DEG;
        yaw_deg = (yaw_deg1 + yaw_deg2) / 2;
    }

    if (phi0 * phi1 > 0 && fabs(phi0) > fabs(phi1)) {
        confidence = 0;
        return false;
    }
    
    confidence = gama / angle_thresh_;
    if (gama < angle_thresh_) {
        // confidence = 0.1;
        confidence *= 0.2;
    }

    if(fabs(object.vi[i1]) > fabs(object.ve[i1]))
    {
        confidence *= 0.2;
    }

    // apply velocity to get confidence
    double estimated_ve0 = object.vi[i0] / cos(phi0 + yaw_deg * DEG2RAD);
    double estimated_ve1 = object.vi[i1] / cos(phi1 + yaw_deg * DEG2RAD);
    double v_diff0 = 1 - fabs((estimated_ve0- object.ve[i0]) / object.ve[i0]);
    double v_diff1 = 1 - fabs((estimated_ve1- object.ve[i1]) / object.ve[i1]);
    
    v_diff0 =  v_diff0 > 0.5? std::pow(v_diff0, 1) : 0.1;
    v_diff1 =  v_diff1 > 0.5? std::pow(v_diff1, 2) : 0.1;

    double v_conf = estimated_ve1 < min_velocity_ ? 0.2 : 1;
    // apply static difference to confidence
    confidence *= v_diff0 * v_diff1 * v_conf;

    return true;
}

} // namespace calibration
} // namepsace autoCalib