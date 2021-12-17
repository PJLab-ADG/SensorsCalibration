/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include <unistd.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pangolin/pangolin.h>
#include <iostream>
#include <boost/filesystem.hpp>

using namespace std;

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#define APPLY_COLOR_TO_LIDAR_INTENSITY  // to set intensity colored or not
typedef std::vector<Eigen::Matrix4d> VecMatrixs4d;

std::vector<pangolin::GlBuffer *> vertexBuffers;
std::vector<pangolin::GlBuffer *> colorBuffers;
std::vector<unsigned int> sizeBuffers;

bool ReadLidarPoses(const std::string &lidar_pose_file,
                    VecMatrixs4d *poses,
                    std::vector<std::string> *pcd_file_names);

void ProcessSingleFrame(const std::string &pcdName,
                        const Eigen::Matrix4d &lidarPose);

#ifdef APPLY_COLOR_TO_LIDAR_INTENSITY
struct RGB {
    unsigned char r;
    unsigned char g;
    unsigned char b;
};
RGB GreyToColorMix(int val);
#endif

int main(int argc, char **argv) {
    size_t beginIndex = 0;
    if (argc < 3 ) {
        cout << "Usage: ./pcd_view_stitcher pcd_folder lidar_pose_file "
                "\nexample:\n\t"
                "./bin/pcd_view_stitcher data/top_center_lidar data/top_center_lidar-pose.txt"
             << endl;
        return 0;
    }

    // argument parse
    std::string pcd_dir(argv[1]);
    if (pcd_dir.rfind("/") != pcd_dir.size() - 1) {
        pcd_dir += "/";
    }
    std::string lidar_pose_file(argv[2]);

    VecMatrixs4d poses;
    std::vector<std::string> pcd_file_names;
    if (!ReadLidarPoses(lidar_pose_file, &poses, &pcd_file_names)) {
        std::cerr << "load lidar poses failed." << std::endl;
        return -1;
    }
    std::cout << "total read " << pcd_file_names.size()
              << " pcd files with poses\n";

    // convert to local frame to support global pose
    auto T0 = poses[beginIndex].inverse().eval();
    for (size_t i = beginIndex; i < poses.size(); ++i) {
        poses[i] = T0 * poses[i];
    }

    vertexBuffers.reserve(pcd_file_names.size());
    colorBuffers.reserve(pcd_file_names.size());
    sizeBuffers.reserve(pcd_file_names.size());
    // view
    const int width = 1920, height = 1280;
    pangolin::CreateWindowAndBind("intensity player", width, height);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));
    pangolin::View &d_cam =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1.0 * width / height)
            .SetHandler(new pangolin::Handler3D(s_cam));
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    pangolin::OpenGlMatrix Twc;  // camera to world
    Twc.SetIdentity();

    // frm idx of current frame
    int curFrm = beginIndex;
    int lastFrm = beginIndex - 1;
    int step = 1;

    // control panel
    pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
                                          pangolin::Attach::Pix(150));
    pangolin::Var<int> cpStep("cp.step", step, 0, 7);
    pangolin::Var<bool> menuNext("cp.Next Frame", false, false);
    pangolin::Var<bool> autoplay("cp.Autoplay", false, true);

    pangolin::Var<int> currIndex("cp.Index", pcd_file_names.size());
    pangolin::Var<int> gpuMem("cp.Memory Free", 0);

    while (!pangolin::ShouldQuit()) {
        // register key
        pangolin::RegisterKeyPressCallback('f',
                                           [&curFrm, step] { curFrm += step; });
        if (static_cast<size_t>(curFrm) >= poses.size()) {
            curFrm = poses.size() - 1;
        }

        auto &T = poses[curFrm];
        // update camera
        Twc.m[0] = T(0, 0);
        Twc.m[1] = T(1, 0);
        Twc.m[2] = T(2, 0);
        Twc.m[3] = 0.0;
        Twc.m[4] = T(0, 1);
        Twc.m[5] = T(1, 1);
        Twc.m[6] = T(2, 1);
        Twc.m[7] = 0.0;
        Twc.m[8] = T(0, 2);
        Twc.m[9] = T(1, 2);
        Twc.m[10] = T(2, 2);
        Twc.m[11] = 0.0;
        Twc.m[12] = T(0, 3);
        Twc.m[13] = T(1, 3);
        Twc.m[14] = T(2, 3);
        Twc.m[15] = 1.0;
        s_cam.Follow(Twc);
        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (cpStep.GuiChanged()) {
            step = cpStep.Get();
            std::cout << "step changed !! current value is: " << step
                      << std::endl;
        }
        // next frame
        if (autoplay || menuNext) {
            curFrm += step;
            if (static_cast<size_t>(curFrm) >= poses.size()) {
                curFrm = poses.size() - 1;
            }
            menuNext = false;
        }
        if (lastFrm < curFrm) {
            ProcessSingleFrame(pcd_dir + pcd_file_names[curFrm], poses[curFrm]);
            lastFrm = curFrm;
        }

        // draw points
        glDisable(GL_LIGHTING);
        glPointSize(2);
        for (size_t i = 0; i < vertexBuffers.size(); i++) {
            pangolin::GlBuffer *colorBufferCurr = colorBuffers[i];
            pangolin::GlBuffer *vertexBufferCurr = vertexBuffers[i];

            if (colorBufferCurr == nullptr || vertexBufferCurr == nullptr) {
                throw std::runtime_error("error in drawing buffers");
            }
            int numGLBufferGoodPoints = sizeBuffers[i];
            colorBufferCurr->Bind();
            glColorPointer(colorBufferCurr->count_per_element,
                           colorBufferCurr->datatype, 0, 0);
            glEnableClientState(GL_COLOR_ARRAY);
            vertexBufferCurr->Bind();
            glVertexPointer(vertexBufferCurr->count_per_element,
                            vertexBufferCurr->datatype, 0, 0);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_POINTS, 0, numGLBufferGoodPoints);
            glDisableClientState(GL_VERTEX_ARRAY);
            vertexBufferCurr->Unbind();
            glDisableClientState(GL_COLOR_ARRAY);
            colorBufferCurr->Unbind();
        }
        // draw cameras
        for (int i = 1; i < curFrm; i++) {
            glLineWidth(3.0);
            glColor3f(1.0f, 0.4f, 0.7f);
            glBegin(GL_LINES);
            glVertex3d(poses[i](0, 3), poses[i](1, 3), poses[i](2, 3));
            glVertex3d(poses[i - 1](0, 3), poses[i - 1](1, 3),
                       poses[i - 1](2, 3));
            glEnd();
        }
        currIndex = curFrm;
        GLint cur_avail_mem_kb = 0;
        glGetIntegerv(GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX,
                      &cur_avail_mem_kb);
        int memFree = cur_avail_mem_kb / 1024;
        gpuMem = memFree;
        pangolin::FinishFrame();
        usleep(2000);
        glFinish();
    }

    return 0;
}

bool ReadLidarPoses(const std::string &lidar_pose_file,
                    VecMatrixs4d *poses,
                    std::vector<std::string> *pcd_file_names) {
    std::ifstream fin(lidar_pose_file);
    if (!fin) {
        std::cerr << "bad lidar pose file!!!";
        return false;
    }
    std::string line;
    std::string ptline;
    std::string timestamp;
    Eigen::Matrix4d mat44 = Eigen::Matrix4d::Identity();
    while (getline(fin, line)) {
        timestamp = line.substr(0, line.find_first_of(" "));
        pcd_file_names->emplace_back(timestamp + ".pcd");
        ptline = line.substr(line.find_first_of(" ") + 1);
        std::stringstream ss(ptline);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                ss >> mat44(i, j);
            }
        }
        poses->emplace_back(mat44);
    }

    if (poses->empty()) {
        std::cerr << "empty in pose file!!!";
        return false;
    }
    return true;
}

void ProcessSingleFrame(const std::string &pcdName,
                        const Eigen::Matrix4d &lidarPose) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudLidar(
        new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile(pcdName, *cloudLidar) != 0) {
        return;
    }

    int pointsNum = cloudLidar->points.size();

    pangolin::GlBuffer *vertexbuffer = new pangolin::GlBuffer(
        pangolin::GlArrayBuffer, pointsNum, GL_FLOAT, 3, GL_DYNAMIC_DRAW);
    pangolin::GlBuffer *colorbuffer =
        new pangolin::GlBuffer(pangolin::GlArrayBuffer, pointsNum,
                               GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);

    float *dataUpdate = new float[pointsNum * 3];
    unsigned char *colorUpdate = new unsigned char[pointsNum * 3];
    for (int ipt = 0; ipt < pointsNum; ipt++) {
        Eigen::Vector4d pointPos(cloudLidar->points[ipt].x,
                                 cloudLidar->points[ipt].y,
                                 cloudLidar->points[ipt].z, 1.0);
        Eigen::Vector4d pointPosWorld = lidarPose * pointPos;
        dataUpdate[ipt * 3 + 0] = pointPosWorld.x();
        dataUpdate[ipt * 3 + 1] = pointPosWorld.y();
        dataUpdate[ipt * 3 + 2] = pointPosWorld.z();

#ifdef APPLY_COLOR_TO_LIDAR_INTENSITY
        RGB colorFake = GreyToColorMix(cloudLidar->points[ipt].intensity);
        colorUpdate[ipt * 3 + 0] = static_cast<unsigned char>(colorFake.r);
        colorUpdate[ipt * 3 + 1] = static_cast<unsigned char>(colorFake.g);
        colorUpdate[ipt * 3 + 2] = static_cast<unsigned char>(colorFake.b);
#else
        for (int k = 0; k < 3; k++) {
            colorUpdate[ipt * 3 + k] =
                static_cast<unsigned char>(cloudLidar->points[ipt].intensity);
        }
#endif
    }

    (vertexbuffer)->Upload(dataUpdate, sizeof(float) * 3 * pointsNum, 0);
    (colorbuffer)
        ->Upload(colorUpdate, sizeof(unsigned char) * 3 * pointsNum, 0);

    vertexBuffers.push_back(vertexbuffer);
    colorBuffers.push_back(colorbuffer);
    sizeBuffers.push_back(pointsNum);
}

#ifdef APPLY_COLOR_TO_LIDAR_INTENSITY
RGB GreyToColorMix(int val) {
    int r, g, b;
    if (val < 128) {
        r = 0;
    } else if (val < 192) {
        r = 255 / 64 * (val - 128);
    } else {
        r = 255;
    }
    if (val < 64) {
        g = 255 / 64 * val;
    } else if (val < 192) {
        g = 255;
    } else {
        g = -255 / 63 * (val - 192) + 255;
    }
    if (val < 64) {
        b = 255;
    } else if (val < 128) {
        b = -255 / 63 * (val - 192) + 255;
    } else {
        b = 0;
    }
    RGB rgb;
    rgb.b = b;
    rgb.g = g;
    rgb.r = r;
    return rgb;
}
#endif
