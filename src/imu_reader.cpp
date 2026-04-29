#include "imu_reader.hpp"
#include <fstream>
#include <sstream>
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;

ImuReader::ImuReader(const std::string& drive_path)
    : drive_path_(drive_path) {}


std::vector<double> ImuReader::loadTimestamps() const {
    std::vector<double> ts;
    std::ifstream f(drive_path_ + "/oxts/timestamps.txt");
    std::string line;

    while(std::getline(f, line)) {
        if (line.empty()) continue; 
        int h, m; double s;
        sscanf(line.c_str() + 11, "%d:%d:%lf", &h, &m, &s);
        ts.push_back(h * 3600 + m * 60 + s);
    }
    return ts;
}

std::vector<ImuMeasurement> ImuReader::measurements() const {
    auto ts = loadTimestamps();
    std::vector<ImuMeasurement> data;
    std::string oxts_dir = drive_path_ + "/oxts/data";
    std::vector<std::string> files;
    for (auto& e :  fs::directory_iterator(oxts_dir))
        if (e.path().extension() == ".txt")
            files.push_back(e.path().string());
    std::sort(files.begin(), files.end());

    for (size_t i = 0; i < files.size(); ++i) {
        std::ifstream f(files[i]);
        std::vector<double> vals;
        double v; while (f >> v) vals.push_back(v);
        if (vals.size() < 30) continue;

        ImuMeasurement m;
        m.timestamp = ts[i];
        m.acc = {vals[11], vals[12], vals[13]};
        m.gyro = {vals[17], vals[18], vals[19]};
        data.push_back(m);
    }
    return data;
}