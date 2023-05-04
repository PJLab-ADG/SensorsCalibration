#include "RadarDataLoader.hpp"

#define OBJECT_MIN_RANGE_DIST 30
#define FILTER_OBJECT_MAX_WIDTH 4
#define FILTER_OBJECT_MIN_WIDTH 1.0

namespace autoCalib {
namespace calibration {

void RadarDataLoader::preLoadImu(const std::string &novatel_enu,
                                 std::vector<ImuFrame> &imu_frames,
                                 double start_sec,
                                 double max_sec)
{
    int start_frm = start_sec * NOVATEL_INSPVA_FREQ;
    int end_frm = start_frm + max_sec * NOVATEL_INSPVA_FREQ;
    imu_frames.clear();

    std::ifstream enu_file(novatel_enu);
    std::string line;
    if (!enu_file) {
        LOGE("Open imu file failed.\n");
        return;
    }

    // skip first line
    getline(enu_file, line);

    int cnt = 0, real_cnt = 0;

    while (getline(enu_file, line)) {
        cnt ++;
        if (cnt < start_frm) continue;
        if (cnt > end_frm) break;

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
        
        
        ImuFrame imu_frm;
        if(!timeString2timecount(elements[0], imu_frm.timestamp)) continue;
        imu_frm.yaw = stod(elements[10]) * RAD2DEG;
        imu_frm.v = Eigen::Vector3d(stod(elements[6]), stod(elements[5]),
                                    stod(elements[7])).norm();
        imu_frm.x = stod(elements[2]);
        imu_frm.y = stod(elements[3]);
        imu_frm.z = stod(elements[4]);
        imu_frames.emplace_back(imu_frm);
        real_cnt ++;
    }

    LOGI("Read %d pose datas.", real_cnt);
}

bool RadarDataLoader::getStraightSegment(
    const std::vector<ImuFrame> &imu_frames,
    std::vector<std::vector<int>> &segment_idxes)
{
    segment_idxes.clear();
    int i = 0;
    int size = imu_frames.size();
    dist_thresh_ = 0.2;
    angle_thresh_ = 1.0;
    length_thresh_ = 5;
    // line parameters
    double m_a, m_b, m_c;
    while (i < size - 1) {
        int start_idx = i;
        int end_idx = i + 1;
        // get line param
        m_a = imu_frames[end_idx].y - imu_frames[start_idx].y;
        m_b = imu_frames[start_idx].x - imu_frames[end_idx].x;
        m_c = imu_frames[end_idx].x * imu_frames[start_idx].y - imu_frames[start_idx].x * imu_frames[end_idx].y;
        double sqrt_l_param = sqrt(m_a * m_a + m_b * m_b);
        double line_yaw = (imu_frames[i+1].yaw + imu_frames[i].yaw) / 2.0;
        end_idx ++;

        while (end_idx < size) {
            double c_x = imu_frames[end_idx].x;
            double c_y = imu_frames[end_idx].y;
            double c_yaw = imu_frames[end_idx].yaw;
            // check whether current point fit to the line
            double pt_line_dist = fabs(m_a * c_x + m_b * c_y + m_c);
            pt_line_dist /= sqrt_l_param;
            if (pt_line_dist > dist_thresh_) break;
            double yaw_diff = c_yaw - line_yaw;
            if (yaw_diff > 180.0) yaw_diff -= 360;
            if (yaw_diff < -180.0) yaw_diff += 360;
            if (yaw_diff > angle_thresh_) break;
            end_idx ++;
        }

        i = end_idx;
        // check whether line length is enough
        double dx = imu_frames[end_idx - 1].x - imu_frames[start_idx].x;
        double dy = imu_frames[end_idx - 1].y - imu_frames[start_idx].y;
        double line_len = sqrt(dx * dx + dy * dy);
        if (line_len > length_thresh_)
        {
            printf("Find line (%d, %d), length=%.3f!\n", start_idx, end_idx-1, line_len);
            segment_idxes.emplace_back(std::vector<int>({start_idx, end_idx-1}));
        }
        else {
            printf("    Fail line (%d, %d), length=%.3f.\n", start_idx, end_idx-1, line_len);
        }
    }
    if (segment_idxes.size() < 1) {
        LOGE("No straight line detected.");
        return false;
    }
    return true;
}

void RadarDataLoader::getRadarCalibData(
    const std::string &radar_file_dir,
    const std::string &radar_type,
    const std::string &novatel_enu,
    std::vector<RadarObject> &radar_frames,
    double start_sec,
    double max_sec)
{
    std::vector<ImuFrame> imu_frames;
    preLoadImu(novatel_enu, imu_frames, start_sec, max_sec);
    std::vector<std::vector<int>> segment_idxs;
    if (!getStraightSegment(imu_frames, segment_idxs)) {
        LOGE("unable to find straight line.");
        exit(1);
    }

    std::vector<std::string> radar_files;
    DIR *dir;
    struct dirent *ptr;
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

    if (radar_type == "delphi") {
        if(!getDelphiRadarFrame(radar_file_dir, radar_files, imu_frames, 
                                segment_idxs, radar_frames)) exit(1);
    }
    else if (radar_type == "conti") {
        if(!getContiRadarFrame(radar_file_dir, radar_files, imu_frames, 
                               segment_idxs, radar_frames)) exit(1);        
    }
    else {
        LOGE("only support 'delphi' and 'conti' radar type.");
        exit(1);
    }
    return;        
}

bool RadarDataLoader::getDelphiRadarFrame(const std::string &radar_dir,
                             const std::vector<std::string> &radar_files,
                             const std::vector<ImuFrame> &imu_frames,
                             const std::vector<std::vector<int>> &segment_idxs,
                             std::vector<RadarObject> &radar_objects)
{

    radar_objects.clear();
    std::vector<int> object_idxs(64, -1);
    // inspva data index to interpolate speed
    unsigned int seg_i = 0;
    double start_time = imu_frames[segment_idxs[seg_i][0]].timestamp;
    double end_time = imu_frames[segment_idxs[seg_i][1]].timestamp;
    int imu_idx = segment_idxs[seg_i][0];
    int imu_size = imu_frames.size();
    for (size_t i = 0; i < radar_files.size(); ++i) {
        std::stringstream st(radar_files[i]);
        std::string sts;
        std::getline(st, sts, '.');
        double file_time;
        timeString2timecount_2(sts, file_time);
        if (file_time + DELPHI_TIMEGAP < start_time) continue;

        std::string file_path = radar_dir + radar_files[i];
        std::ifstream infile(file_path);
        std::string line;
        if (!infile) {
            LOGW("Open radar file %s failed.", radar_files[i].c_str());
            continue;
        }
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
            // no target
            int track_status = stoi(elements[6]);
            double timestamp = stod(elements[0]) * 1e-09;
            double track_id = stoi(elements[1]);
            if (track_status == 0 || track_status == 6) { 
                object_idxs[track_id] = -1;
                continue;
            }
            if (timestamp >= start_time && timestamp < end_time) {
                // find neighbor imu data
                while (imu_frames[imu_idx].timestamp < timestamp) {
                    imu_idx ++;
                    if (imu_idx >= imu_size) return true;
                }
                // what is new coasted target?
                if (object_idxs[track_id] == -1 || track_status == 1) {
                    object_idxs[track_id] = radar_objects.size();
                    // RadarObject object;
                    radar_objects.push_back(RadarObject());
                }
                  
                int oidx = object_idxs[track_id];
                radar_objects[oidx].timestamps.emplace_back(timestamp);
                radar_objects[oidx].track_range.emplace_back(stod(elements[8]));
                radar_objects[oidx].track_angle.emplace_back(stod(elements[7]));
                radar_objects[oidx].vi.emplace_back(fabs(stod(elements[14])));

                // imu data
                double speed = this->interpolate(timestamp, 
                    imu_frames[imu_idx-1].timestamp, imu_frames[imu_idx].timestamp,
                    imu_frames[imu_idx-1].v, imu_frames[imu_idx].v);
                double x = this->interpolate(timestamp, 
                    imu_frames[imu_idx-1].timestamp, imu_frames[imu_idx].timestamp,
                    imu_frames[imu_idx-1].x, imu_frames[imu_idx].x);
                double y = this->interpolate(timestamp, 
                    imu_frames[imu_idx-1].timestamp, imu_frames[imu_idx].timestamp,
                    imu_frames[imu_idx-1].y, imu_frames[imu_idx].y);
                double z = this->interpolate(timestamp, 
                    imu_frames[imu_idx-1].timestamp, imu_frames[imu_idx].timestamp,
                    imu_frames[imu_idx-1].z, imu_frames[imu_idx].z);    
                radar_objects[oidx].ve.emplace_back(speed);
                radar_objects[oidx].x.emplace_back(x);
                radar_objects[oidx].y.emplace_back(y);
                radar_objects[oidx].z.emplace_back(z);
            }
            else if (timestamp >= end_time) {
                seg_i ++;
                if (seg_i >= segment_idxs.size()) return true;
                start_time = imu_frames[segment_idxs[seg_i][0]].timestamp;
                end_time = imu_frames[segment_idxs[seg_i][1]].timestamp;
                // reset object
                object_idxs = std::vector<int>(64, -1);
            }
        }

    }

    LOGI("Read %d radar objects.", static_cast<int>(radar_objects.size()));
    return true;
}


bool RadarDataLoader::getContiRadarFrame(const std::string &radar_dir,
                             const std::vector<std::string> &radar_files,
                             const std::vector<ImuFrame> &imu_frames,
                             const std::vector<std::vector<int>> &segment_idxs,
                             std::vector<RadarObject> &radar_objects)
{
    radar_objects.clear();
    std::vector<int> object_idxs(100, -1);
    std::vector<bool> object_track(100, false);
    // inspva data index to interpolate speed
    unsigned int seg_i = 0;
    double start_time = imu_frames[segment_idxs[seg_i][0]].timestamp;
    double end_time = imu_frames[segment_idxs[seg_i][1]].timestamp;
    int imu_idx = segment_idxs[seg_i][0];
    int imu_size = imu_frames.size();

    double prev_id = 200;
    for (size_t i = 0; i < radar_files.size(); ++i) {
        std::stringstream st(radar_files[i]);
        std::string sts;
        std::getline(st, sts, '.');
        double file_time;
        timeString2timecount_2(sts, file_time);
        if (file_time < start_time) continue;

        std::string file_path = radar_dir + radar_files[i];
        std::ifstream infile(file_path);
        std::string line;
        if (!infile) {
            LOGW("Open radar file %s failed.", radar_files[i].c_str());
            continue;
        }
        while (getline(infile, line)) {
            std::stringstream ss(line);
            std::vector<std::string> elements;
            std::string elem;
            while (getline(ss, elem, ',')) {
                elements.emplace_back(elem);
            }

            // skip the first line
            if (elements[0] == "time_ns") continue;
            // no target
            double timestamp = stod(elements[0]) * 1e-9;
            double track_id = stoi(elements[1]);
            // new frame, check whether track and clean on track box
            if (track_id < prev_id) {
                for (size_t idx = 0; idx < object_track.size(); idx++) {
                    if (!object_track[idx] && object_idxs[idx] != -1) {
                        // object is no longer on track
                        object_idxs[idx] = -1;
                    }
                    object_track[idx] = false;
                }
            }

            if (timestamp >= start_time && timestamp < end_time) {
                // find neighbor imu data
                while (imu_frames[imu_idx].timestamp < timestamp) {
                    imu_idx ++;
                    if (imu_idx >= imu_size) return true;
                }
                object_track[track_id] = true;
                // whether new traget
                if (object_idxs[track_id] == -1) {
                    object_idxs[track_id] = radar_objects.size();
                    // RadarObject object;
                    radar_objects.push_back(RadarObject());
                }
                  
                int oidx = object_idxs[track_id];
                double posx = stod(elements[4]);
                double posy = stod(elements[5]);
                double vx = stod(elements[2]);
                double vy = stod(elements[3]);
                radar_objects[oidx].timestamps.emplace_back(timestamp);
                radar_objects[oidx].track_range.emplace_back(sqrt(posx*posx + posy*posy));
                radar_objects[oidx].track_angle.emplace_back(atan(posy/posx));
                radar_objects[oidx].vi.emplace_back(sqrt(vx * vx + vy * vy));

                // imu data
                double speed = this->interpolate(timestamp, 
                    imu_frames[imu_idx-1].timestamp, imu_frames[imu_idx].timestamp,
                    imu_frames[imu_idx-1].v, imu_frames[imu_idx].v);
                double x = this->interpolate(timestamp, 
                    imu_frames[imu_idx-1].timestamp, imu_frames[imu_idx].timestamp,
                    imu_frames[imu_idx-1].x, imu_frames[imu_idx].x);
                double y = this->interpolate(timestamp, 
                    imu_frames[imu_idx-1].timestamp, imu_frames[imu_idx].timestamp,
                    imu_frames[imu_idx-1].y, imu_frames[imu_idx].y);
                double z = this->interpolate(timestamp, 
                    imu_frames[imu_idx-1].timestamp, imu_frames[imu_idx].timestamp,
                    imu_frames[imu_idx-1].z, imu_frames[imu_idx].z);    
                radar_objects[oidx].ve.emplace_back(speed);
                radar_objects[oidx].x.emplace_back(x);
                radar_objects[oidx].y.emplace_back(y);
                radar_objects[oidx].z.emplace_back(z);
                prev_id = track_id;
            }
            else if (timestamp >= end_time) {
                seg_i ++;
                if (seg_i >= segment_idxs.size()) return true;
                start_time = imu_frames[segment_idxs[seg_i][0]].timestamp;
                end_time = imu_frames[segment_idxs[seg_i][1]].timestamp;
                // reset object
                object_idxs = std::vector<int>(100, -1);
                object_track = std::vector<bool>(100, false);
            }
        }

    }

    LOGI("Read %d radar objects.", static_cast<int>(radar_objects.size()));
    return true;
}

} // namespace calibration
} // namespace autoCalib