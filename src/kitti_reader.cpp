#include "kitti_reader.hpp"
#include <fstream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <stdexcept>
#include <cstring>

namespace fs = std::filesystem;

KittiReader::KittiReader(const std::string& drive_path)
    : drive_path_(drive_path) {} // Initialize driver path

std::vector<std::string> KittiReader::scanPaths() const {
    std::vector<std::string> paths;
    std::string velo_dir = drive_path_ + "/velodyne_points/data";
    for (auto& entry : fs::directory_iterator(velo_dir)){
        if (entry.path().extension() == ".bin")
            paths.push_back(entry.path().string()); // Store paths as strings and sort them afterwards
    }
    std::sort(paths.begin(), paths.end()); // Ensure paths are sorted in the correct order
    return paths; // Return sorted paths
}

// Format of timestamps.txt: "2011-09-26 14:43:57.469083595"
std::vector<double> KittiReader::scanTimestamps() const {
    std::vector<double> ts;
    std::ifstream f(drive_path_ + "/velodyne_points/timestamps.txt");
    std::string line;
    while (std::getline(f, line)){
        if (line.empty()) continue;
        int h, m; double s;
        sscanf(line.c_str() + 11, "%d:%d:%lf", &h, &m, &s); // Parse timestamp from the line
        ts.push_back(h * 3600.0 + m * 60.0 + s);
    }
    return ts; 
}

PointCloud KittiReader::loadBin(const std::string& path){
    PointCloud cloud;
    std::ifstream f(path, std::ios::binary); // Open the binary file for reading
    if (!f) throw std::runtime_error("Cannot open file: " + path);
    float buf[4]; // Buffer to hold x, y, z, intensity for each point
    while (f.read(reinterpret_cast<char*>(buf), sizeof(buf))){
        // char* meaning: read 4 floats (16 bytes) into the buffer
        PointXYZ pt;
        pt.x = buf[0]; pt.y = buf[1]; pt.z = buf[2]; pt.intensity = buf[3];
        cloud.push_back(pt);
    }
    return cloud;
}

Eigen::Matrix4d KittiReader::loadCalibImuToVelo() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(); // Initialize transformation matrix as identity which identity is a 4x4 matrix with 1s on the diagonal and 0s elsewhere
    std::ifstream f(drive_path_ + "/calib/imu_to_velo.txt");
    std::string line;
    while (std::getline(f, line)){
        if (line.substr(0, 1) == "R") {
            std::istringstream ss(line.substr(3));
            // Parse the rotation part of the transformation matrix
            for (int i=0; i<3; ++i)
                for (int j=0; j<3; ++j)
                    ss >> T(i, j); // Rotation is on the upper-left 3x3 part of the transformation matrix
        } else if (line.substr(0, 1) == "T") {
            std::istringstream ss(line.substr(3));
            for (int i=0; i<3; ++i)
                ss >> T(i, 3); // Translation is on the last column of the transformation matrix
        }
    }
    return T;
}

