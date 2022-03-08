#include "radar2car.hpp"
#include "RadarDataLoader.hpp"
#include <cmath>
#include <algorithm>

namespace autoCalib {
namespace calibration {

void RadarCalibrator::calib(const std::string &radar_file_dir,
                            const std::string &radar_type,
                            const std::string &novatel_inspva,
                            const std::string &output_dir,
                            const RadarCalibParam &param,
                            bool guessRoughYaw,
                            bool savefig)
{
    // set parameters
    min_velocity_ = param.min_velocity;
    max_velocity_ = param.max_velocity;
    applyScale_ = param.applyVegoScale;
    ve_diff_range_ = 1 - cos(rough_deg_gap_ * 2 * M_PI / 180.0);
    // load data from radar file dir and align timestamp with novatel speed
    RadarDataLoader data_loader;
    std::vector<RadarFrame> radar_datas;
    data_loader.setMinVelocity(min_velocity_*(1-ve_diff_range_));
    data_loader.getRadarCalibData(radar_file_dir, radar_type, novatel_inspva,
        radar_datas, param.start_file_num, param.max_file_num);
    
    used_frm_ = 0;
    double yaw_all = 0;
    double cnt = 0;
    double start_yaw = 0;
    int iter = 1;
    int estimate_iter = 200;
    applyScale_ = false;

    if (guessRoughYaw)
        getRoughYaw(radar_datas, 20, start_yaw); 
    else 
        start_yaw = param.initial_yaw;

    if (radar_type == "conti") {
        start_yaw = 0;
        std::cout << "yaw start at :" << start_yaw * 180.0 / M_PI << std::endl;
        estimate_iter = 300;
        iter = 5;
        int data_num = radar_datas.size();
        bool applyWeight = true;
        for(int i = 0; i < data_num; ++i) {
            std::vector<int> stationary_idxs;
            auto data = radar_datas[i];
            for(size_t turn = 0; turn < iter; ++turn) {
                // get stationary based on max velocity and provided vehicle speed
                double new_yaw;
                if(!getStationary(start_yaw, data.ve, data.vi, data.track_angle, 
                                stationary_idxs)) break;
                // if (used_frm_ % 10 == 0 && used_frm_ > estimate_iter) {
                //     saveStationaryTxt(data.track_angle, data.track_dist, data.times,
                //                     stationary_idxs, 32*M_PI/180);
                // }
                if(!fittingCosine(start_yaw, data.ve, data.vi, data.track_angle,
                                stationary_idxs, new_yaw, applyWeight)) break;
                // if (used_frm_ < estimate_iter)
                start_yaw = new_yaw;
                // narrow range to pick out stationary object
                if (used_frm_ == estimate_iter / 2.0) {
                    // applyScale_ = true;
                    ve_diff_range_ = 1 - cos(param.v_diff_angle_deg * 2 * M_PI / 180.0);
                }
                if (used_frm_ == estimate_iter) {
                    ve_diff_range_ = 1 - cos(param.v_diff_angle_deg * M_PI / 180.0);
                    applyWeight = true;
                }
                if (used_frm_ > estimate_iter) {
                    cnt += 1;
                    yaw_all += new_yaw;
                }

                used_frm_ ++;
            }
        }
    }
    else {
        // start_yaw = 0;
        std::cout << "yaw start at :" << start_yaw * 180.0 / M_PI << std::endl;
        int data_num = radar_datas.size();
        bool applyWeight = true;
        for(int i = 0; i < data_num; ++i) {
            std::vector<int> stationary_idxs;
            auto data = radar_datas[i];
            for(size_t turn = 0; turn < iter; ++turn) {
                // get stationary based on max velocity and provided vehicle speed
                double new_yaw;
                if(!getStationary(start_yaw, data.ve, data.vi, data.track_angle, 
                                stationary_idxs)) break;
                // if (used_frm_ % 10 == 0 && used_frm_ > estimate_iter) {
                //     saveStationaryTxt(data.track_angle, data.track_dist, data.times,
                //                     stationary_idxs, 32*M_PI/180);
                // }
                if(!fittingCosine(start_yaw, data.ve, data.vi, data.track_angle,
                                stationary_idxs, new_yaw, applyWeight)) break;
                // if (used_frm_ < estimate_iter)
                start_yaw = new_yaw;
                // narrow range to pick out stationary object
                if (used_frm_ == estimate_iter / 2.0) {
                    // applyScale_ = true;
                    ve_diff_range_ = 1 - cos(param.v_diff_angle_deg * 2 * M_PI / 180.0);
                }
                if (used_frm_ == estimate_iter) {
                    ve_diff_range_ = 1 - cos(param.v_diff_angle_deg * M_PI / 180.0);
                    // applyWeight = true;
                    iter = 1;
                }
                if (used_frm_ > estimate_iter) {
                    double confidence = static_cast<double>(stationary_idxs.size());
                    if (confidence < 6) confidence = 0.02;
                    else confidence = 1.0;
                    cnt += confidence;
                    yaw_all += new_yaw;
                    // start_yaw = yaw_all/cnt;
                }

                used_frm_ ++;
            }
        }
    }


    if (used_frm_ < 1) {
        LOGE("failed to calibrate radar.");
        return;
    }

    std::cout << "start yaw: " << start_yaw * 180 / M_PI << std::endl;
    std::cout << "final yaw: " << yaw_all * 180 / M_PI / cnt << std::endl;
    std::cout << "use frame: " << used_frm_ << std::endl;
    std::cout << "ve_scale_: " << ve_scale_ << std::endl;

    // save data to file
}


bool RadarCalibrator::getRoughYaw(const std::vector<RadarFrame> &radar_datas,
                                  const int &frame_gap,
                                  double &rough_yaw)
{
    // x axis: [-90, 90], 5 degree interval
    int interval = 180 / rough_deg_gap_;
    double thresh = cos(rough_deg_gap_ * M_PI / 180.0);

    std::vector<double> probability(interval);
    int data_num = radar_datas.size();
    // use frame gap to avoid same scene
    if (data_num < frame_gap) {
        LOGE("there should be no less than %d frames to get rough yaw.", frame_gap);
        return false;
    }
    for (int i = 0; i < data_num; i += frame_gap) {
        int yaw_idx;
        double confidence;
        if(this->getRoughYawSingle(thresh, radar_datas[i].ve, radar_datas[i].vi, 
                             radar_datas[i].track_angle, yaw_idx, confidence))
            probability[yaw_idx] += confidence;
    }  
    std::vector<size_t> sort_idx(interval);
    std::iota(sort_idx.begin(), sort_idx.end(), 0);
    std::sort(sort_idx.begin(), sort_idx.end(), [&probability]
              (size_t i1, size_t i2){return probability[i1]>probability[i2];});
    int max_idx = sort_idx[0];
    if (probability[max_idx] == 0) {
        LOGE("failed to get rough yaw. No potential stationary object detected.");
        return false;
    }
    rough_yaw = (max_idx-18) * rough_deg_gap_ + 
                static_cast<double>(rough_deg_gap_)/2.0;
    // deg to rad
    rough_yaw *= - M_PI / 180.0;
    return true;
}


bool RadarCalibrator::getRoughYawSingle(const double &thresh,
                                        const std::vector<double> &ve,
                                        const std::vector<double> &vi,
                                        const std::vector<double> &track_angle,
                                        int &rough_yaw_idx,
                                        double &confidence)
{
    if (ve.size() != vi.size()) {
        LOGE("Vego should be the same size with Vi.");
        return false;
    }
    confidence = 0;
    double thresh_prob = fabs(asin(thresh));
    // x axis: [-90, 90], 5 degree interval
    int interval = 180 / rough_deg_gap_;
    std::vector<int> deg_box(interval);
    std::vector<double> probability(interval);    
    for (size_t i = 0; i < vi.size(); ++i) {
        if (ve_scale_ * ve[i] > max_velocity_ || ve_scale_ * ve[i] < min_velocity_)
            continue;
        double angle = track_angle[i] * 180 / M_PI;
        int x = (angle + 90) / rough_deg_gap_;
        double similar_c = 1.0 - fabs(ve_scale_ * ve[i] - vi[i]) 
                           / ve_scale_ * ve[i];
        similar_c = similar_c > thresh ? 0 : fabs(asin(similar_c));
        if (similar_c > 0) {
            deg_box[x] += 1;
            probability[x] += similar_c;
        }
    }

    for (size_t i = 0; i < probability.size(); ++i) {
        if (deg_box[i] != 0)
            probability[i] /= static_cast<double>(deg_box[i]);
    }
    std::vector<size_t> sort_idx(interval);
    std::iota(sort_idx.begin(), sort_idx.end(), 0);
    std::sort(sort_idx.begin(), sort_idx.end(), [&probability]
              (size_t i1, size_t i2){return probability[i1]>probability[i2];});
    // find most possible yaw
    rough_yaw_idx = sort_idx[0];
    confidence = probability[rough_yaw_idx] * deg_box[rough_yaw_idx];
    // confidence = probability[rough_yaw_idx];
    return true;
}


// timestamp of ve is the same with vi
bool RadarCalibrator::getStationary(const double &yaw,
                                    const std::vector<double> &ve,
                                    const std::vector<double> &vi,
                                    const std::vector<double> &track_angle,
                                    std::vector<int> &stationary_indexs)
{
    stationary_indexs.clear();
    if (ve.size() != vi.size()) {
        LOGE("Vego should be the same size with Vi.");
        return false;
    }
    // assume there's no big velocity difference in 500ms
    std::vector<double> estimated_ve;
    for (int i = 0; i < ve.size(); ++i) {
        estimated_ve.emplace_back(vi[i] / cos(track_angle[i] + yaw));
    }    
    for (int i = 0; i < ve.size(); ++i) {
        // whether speed is in the right range
        double speed = ve_scale_ * ve[i];
        if (fabs(speed) > max_velocity_ || fabs(speed) < min_velocity_)
            continue;
        // check whether stationary
        double estimated_ve = vi[i] / cos(track_angle[i] + yaw);
        // printf("%.2f,%.3f  ", estimated_ve, speed);
        double diff = fabs((estimated_ve - speed) / speed);
        if (diff < ve_diff_range_) 
        // if (diff < ve_diff_range_ || fabs(estimated_ve - speed) < 0.125) 
            stationary_indexs.emplace_back(i);
    }
    // std::cout << std::endl;
    // char ge[10];
    // std::cin >> ge;
    if (stationary_indexs.size() < 1) return false;
    return true; 
}


bool RadarCalibrator::fittingCosine(const double &yaw,
                                    const std::vector<double> &ve,
                                    const std::vector<double> &vi,
                                    const std::vector<double> &track_angle,
                                    const std::vector<int> &selected_indexs,
                                    double &new_yaw,
                                    bool applyWeight)
{
    if (selected_indexs.size() < 3) {
        // LOGW("Failed to fit cosine.Stationary objs less than 3.");
        return false;
    }
    int pt_num = selected_indexs.size();
    // vi = ve * A * cos(phi + yaw + dyaw)
    // because ve is imu data, so set A as coefficient
    // least square: y = A*cos(x + dyaw), x=track_angle+yaw
    // y = ksinx + bcosx
    // dJ/dk = sinx(ksinx + bcosx - y), dJ/db = cosx(ksinx + bcosx - y)
    double sin2x = 0, cos2x = 0;
    double sinxcosx = 0;
    double ysinx = 0, ycosx = 0;
    // double diff_square = ve_diff_range_ * ve_diff_range_;
    for (size_t i = 0; i < selected_indexs.size(); ++i) {
        size_t idx = selected_indexs[i];
        double sinx = sin(track_angle[idx] + yaw);
        double cosx = cos(track_angle[idx] + yaw);
        double y = vi[idx] / ve[idx] / ve_scale_;
        double weight = 1;
        if (applyWeight) {
            weight = fabs(1 - ve_scale_ * ve[idx] * cosx / vi[idx]);
            // weight = weight > ve_diff_range_ ? 0 : 2.0 - weight/ve_diff_range_;
            weight = weight > ve_diff_range_ ? 0 : 1.0 - weight/ve_diff_range_;
            weight *= weight;
        }

        sin2x += sinx * sinx * weight;
        cos2x += cosx * cosx * weight;
        sinxcosx += sinx * cosx * weight;
        ysinx += y * sinx * weight;
        ycosx += y * cosx * weight;
    }
    Eigen::Matrix2d Amat;
    Amat << sin2x, sinxcosx,
            sinxcosx, cos2x;
    Eigen::Vector2d y_vec(ysinx, ycosx);
    Eigen::Vector2d kb = Amat.inverse() * y_vec;
    double A = kb.norm();
    double dyaw = atan2(-kb(0), kb(1));
    // check whether is nan
    if (dyaw != dyaw) return false;
    // if (A > (1+ve_diff_range_) || A < (1-ve_diff_range_)) return false;

    new_yaw = yaw + dyaw;
    if (applyScale_)
        ve_scale_ *= A;

    if (used_frm_ % 2 == 0 ) {
        if (new_yaw > 0)
            printf("%dpts->%dpts: y = %.4fcos(x+%.3f)\n", ve.size(), pt_num, A, new_yaw*180/M_PI);
        else 
            printf("%dpts->%dpts: y = %.4fcos(x%.3f)\n", ve.size(), pt_num, A, new_yaw*180/M_PI);
    }

    // // save image
    double yaw_deg = new_yaw*180/M_PI;
    if (used_frm_ % 10 == 0) {
        std::string txtfile = "./fitline/" + std::to_string(pt_num) + "_" + 
                              std::to_string(yaw_deg) + ".txt";
        saveFitlineTxt(txtfile, ve, vi, track_angle, selected_indexs, A, yaw_deg);
    }

    return true;
}


void RadarCalibrator::saveStationaryTxt(const std::vector<double> &track_angle,
                                        const std::vector<double> &track_dist,
                                        const std::vector<double> &timestamps,
                                        const std::vector<int> &selected_indexs,
                                        const double &yaw)
{
    // int fidx = 0;
    // int sidx = 0;
    // for (size_t i = 0; i < track_angle.size(); ++i) {

    // }
    std::string output_file = "./stationary/" + std::to_string(used_frm_) + "_" + 
                               std::to_string(selected_indexs.size()) + ".txt";
    std::ofstream out_file(output_file);
    if (!out_file.is_open()) {
        LOGE("open output file %s failed.\n", output_file);
        return;
    }
    double prev_time = timestamps[selected_indexs[0]];
    int max_idx = 0;
    // std::cout << "save " << output_file << std::endl;
    for (size_t i = 0; i < selected_indexs.size(); ++i){
        size_t idx = selected_indexs[i];        
        if (timestamps[idx] > prev_time + 0.2) {
            max_idx = selected_indexs[i-1];
            break;
        }
        double x = sin(track_angle[idx] + yaw) * track_dist[idx];
        double y = cos(track_angle[idx] + yaw) * track_dist[idx];
        // prev_time = timestamps[idx];
        out_file << x << " " << y << std::endl;
    }
    for (size_t i = 0; i < timestamps.size(); ++i){
        double x = sin(track_angle[i] + yaw) * track_dist[i];
        double y = cos(track_angle[i] + yaw) * track_dist[i];
        out_file << 0 << " " << x << " " << y << std::endl;
    }    

    out_file.close();    
}

void RadarCalibrator::saveFitlineTxt(const std::string &output_file,
                                     const std::vector<double> &ve,
                                     const std::vector<double> &vi,
                                     const std::vector<double> &track_angle,
                                     const std::vector<int> &selected_indexs,
                                     const double &A,
                                     const double &yaw_deg)
{
    std::ofstream out_file(output_file);
    if (!out_file.is_open()) {
        LOGE("open output file %s failed.\n", output_file);
        return;
    }
    // std::cout << "save " << output_file << std::endl;
    out_file << A << " " << yaw_deg << std::endl;
    for (size_t i = 0; i < selected_indexs.size(); ++i){
        size_t idx = selected_indexs[i];
        double y = vi[idx] / ve[idx];
        double x = track_angle[idx] * 180 / M_PI;
        out_file << x << " " << y << std::endl;
    }

    for (size_t i = 0; i < ve.size(); ++i){
        double x = track_angle[i] * 180 / M_PI;
        double y = vi[i] / ve[i];
        out_file << 0 << " " << x << " " << y << std::endl;
    }   

    out_file.close();    
}

} // namespace calibration
} // namepsace autoCalib