#include "RadarDataLoader.hpp"

#define OBJECT_MIN_RANGE_DIST 30
#define FILTER_OBJECT_MAX_WIDTH 4
#define FILTER_OBJECT_MIN_WIDTH 1.0

namespace autoCalib {
namespace calibration {

void RadarDataLoader::getRadarCalibData(
    const std::string &radar_file_dir,
    const std::string &radar_type,
    const std::string &novatel_inspva,
    std::vector<RadarFrame> &radar_frames,
    int start_radar_file_num,
    int max_radar_file_num)
{
    start_file_num_ = start_radar_file_num;
    std::vector<std::string> radar_files;
    DIR *dir;
    struct dirent *ptr;
    int success_num = 0;
    if ((dir = opendir(radar_file_dir.c_str())) == NULL) {
        LOGE("Open dir error !");
        exit(1);
    }
    while ((ptr = readdir(dir)) != NULL) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            radar_files.emplace_back(ptr->d_name);
        ptr++;
    }
    if (radar_files.size() == 0) {
        LOGE("no file under parsed radar dir.");
        exit(1);
    }
    closedir(dir);
    std::sort(radar_files.begin(), radar_files.end());
    max_file_num_ = std::min(static_cast<int>(radar_files.size()) - start_file_num_ + 1, 
                                max_radar_file_num);
    if (max_file_num_ == 0) {
        LOGE("not enough radar file based on start and end parameters.");
        exit(1);            
    }
    // get start timestamp and end timestamp
    std::stringstream st(radar_files[start_file_num_ - 1]);
    std::stringstream et(radar_files[start_file_num_ - 1 + max_file_num_ - 1]);
    std::string sts, ets;
    std::getline(st, sts, '.');
    std::getline(et, ets, '.');
    timeString2timecount_2(sts, start_timestamp_); 
    timeString2timecount_2(ets, end_timestamp_);
    end_timestamp_ += 0.2;  // conclude the timegap in last radar file 

    // load inspva speed from start_time-1/novatel_freq to end_time+1/novatel_freq
    std::vector<double> inspva_times;
    std::vector<double> inspva_speed;
    if (!getNovatelSpeed(novatel_inspva, inspva_times, inspva_speed)) 
        exit(1);

    if (radar_type == "delphi") {
        if(!getDelphiRadarFrame(radar_file_dir, radar_files, inspva_times, 
                                inspva_speed, radar_frames)) exit(1);
    }
    else if (radar_type == "conti") {
        if(!getContiRadarFrame(radar_file_dir, radar_files, inspva_times, 
                               inspva_speed, radar_frames)) exit(1);        
    }
    else {
        LOGE("only support 'delphi' and 'conti' radar type.");
        exit(1);
    }
    return;        
}

bool RadarDataLoader::getNovatelSpeed(const std::string &novatel_inspva,
                                      std::vector<double> &timestamps,
                                      std::vector<double> &novatel_speed) 
{
    std::ifstream infile(novatel_inspva);
    std::string line;
    int cnt = 0;
    if (!infile) {
        LOGE("Open novatel inspva file failed.\n");
        return false;
    }
    double time_gap = 1 / static_cast<double>(NOVATEL_INSPVA_FREQ);
    while (getline(infile, line)) {
        std::stringstream ss(line);
        std::vector<std::string> elements;
        std::string elem;
        while (getline(ss, elem, ',')) {
            elements.emplace_back(elem);
        }
        if (elements.size() != 12) {
            LOGW("num of line elements error! skip this line.\n");
            continue;
        }
        // skip the first line
        if (elements[0] == "gps_time") {
            continue;
        }

        double timestamp;
        if (!timeString2timecount(elements[0], timestamp)) continue;
        if (timestamp < start_timestamp_ - time_gap) continue;
        if (timestamp > end_timestamp_ + time_gap) break;
        timestamps.emplace_back(timestamp);
        double vn = stod(elements[5]);
        double ve = stod(elements[6]);
        double vu = stod(elements[7]);
        novatel_speed.emplace_back(sqrt(vn*vn + ve*ve + vu*vu));
        cnt ++;
    }

    if (cnt == 0) {
        LOGE("read 0 inspva data.");
        return false;
    }
    LOGI("Read %d novatel inspva datas.", cnt);
    return true;
}


bool RadarDataLoader::getDelphiRadarFrame(const std::string &radar_dir,
                             const std::vector<std::string> &radar_files,
                             const std::vector<double> &inspva_times,
                             const std::vector<double> &inspva_speed,
                             std::vector<RadarFrame> &radar_frames)
{
    if (inspva_times.size() < 2) {
        LOGE("there must be more than 1 inspva data.\n");
        return false;
    }
    radar_frames.clear();
    // inspva data index to interpolate speed
    int inspva_idx = 1; 
    int cnt = 0;
    for (size_t i = start_file_num_-1 ; i < max_file_num_+start_file_num_-1; ++i) {
        std::string file_path = radar_dir + radar_files[i];
        std::ifstream infile(file_path);
        std::string line;
        if (!infile) {
            LOGW("Open radar file %s failed.", radar_files[i]);
        }
        RadarFrame data;
        while (getline(infile, line)) {
            std::stringstream ss(line);
            std::vector<std::string> elements;
            std::string elem;
            while (getline(ss, elem, ',')) {
                elements.emplace_back(elem);
            }
            if (elements.size() != 15) {
                LOGW("num of line elements error! skip this line.\n");
                continue;
            }
            // skip the first line
            if (elements[0] == "time_ns") continue;

            double timestamp = stod(elements[0]) * 1e-09;
            if (inspva_times[inspva_idx-1] > timestamp) {
                LOGI("Read %d radar frames.", cnt);
                return true;
            }
            while(inspva_times[inspva_idx] < timestamp) {
                inspva_idx ++;
                // reach the end of inspva data
                if (inspva_idx >= inspva_times.size()) {
                    LOGI("Read %d radar frames.", cnt);
                    return true;
                }
            }
            if (stoi(elements[4]) == 1) {
                std::cout << "track changed!\n";
            }
            double vi_speed = -stod(elements[14]);
            // double object_width = stod(elements[11]);
            if (fabs(vi_speed) > 81) continue;
            // if (object_width > FILTER_OBJECT_MIN_WIDTH &&
            //     object_width < FILTER_OBJECT_MAX_WIDTH) continue;
            if (inspva_speed[inspva_idx] < min_velocity_) continue;

            double speed = this->interpolateSpeed(timestamp, 
                inspva_times[inspva_idx-1], inspva_times[inspva_idx],
                inspva_speed[inspva_idx-1], inspva_speed[inspva_idx]);
            data.ve.emplace_back(speed);
            // vi is track_range_rate_m_per_s?
            data.vi.emplace_back(vi_speed);
            data.track_angle.emplace_back(stod(elements[7]));
            data.track_dist.emplace_back(stod(elements[8]));
            data.times.emplace_back(timestamp);

        }
        if (data.vi.size() > 0) {
            radar_frames.emplace_back(data);
            cnt ++;
        }
    }
    LOGI("Read %d radar frames.", cnt);
    return true;
}


bool RadarDataLoader::getContiRadarFrame(const std::string &radar_dir,
    const std::vector<std::string> &radar_files,
    const std::vector<double> &inspva_times,
    const std::vector<double> &inspva_speed,
    std::vector<RadarFrame> &radar_frames)
{
    if (inspva_times.size() < 2) {
        LOGE("there must be more than 1 inspva data.\n");
        return false;
    }
    radar_frames.clear();
    // inspva data index to interpolate speed
    int inspva_idx = 1;
    int cnt = 0;
    int file_combine = 50;
    for (size_t i = start_file_num_-1 ; i < max_file_num_+start_file_num_-1; i+=file_combine) {
        RadarFrame data;
        for (int di = 0; di < file_combine; di++) {
            std::string file_path = radar_dir + radar_files[i + di];
            std::ifstream infile(file_path);
            std::string line;
            if (!infile) {
                LOGW("Open radar file %s failed.", radar_files[i + di]);
            }
            while (getline(infile, line)) {
                std::stringstream ss(line);
                std::vector<std::string> elements;
                std::string elem;
                while (getline(ss, elem, ',')) {
                    elements.emplace_back(elem);
                }
                if (elements.size() != 26) {
                    LOGW("num of line elements error! skip this line. %d", elements.size());
                    continue;
                }
                // skip the first line
                if (elements[0] == "time_ns") continue;

                double timestamp = stod(elements[0]) * 1e-09;
                if (inspva_times[inspva_idx-1] > timestamp) {
                    LOGI("Read %d radar frames.", cnt);
                    return true;
                }
                while(inspva_times[inspva_idx] < timestamp) {
                    inspva_idx ++;
                    // reach the end of inspva data
                    if (inspva_idx >= inspva_times.size()) {
                        LOGI("Read %d radar frames.", cnt);
                        return true;
                    }
                }
                
                if (inspva_speed[inspva_idx] < min_velocity_) continue;

                double vx = stod(elements[2]);
                double vy = stod(elements[3]);
                double px = stod(elements[4]);
                double py = stod(elements[5]);
                // double vi = sqrt(vx*vx + vy*vy);
                double vi = -vx;
                // if (vx > 0) vi *= -1;
                // vi is vx+vy?
                data.vi.emplace_back(vi);
                double speed = this->interpolateSpeed(timestamp, 
                    inspva_times[inspva_idx-1], inspva_times[inspva_idx],
                    inspva_speed[inspva_idx-1], inspva_speed[inspva_idx]);
                data.ve.emplace_back(speed);
                data.track_angle.emplace_back(atan(py/px));
                data.track_dist.emplace_back(sqrt(px*px + py*py));
                data.times.emplace_back(timestamp);
            }
        }
        if (data.vi.size() > 0) {
            radar_frames.emplace_back(data);
            cnt ++;
        }
    }
    LOGI("Read %d radar frames.", cnt);
    return true;
}

} // namespace calibration
} // namespace autoCalib