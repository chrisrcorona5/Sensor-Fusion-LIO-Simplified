#pragma once
#include "types.hpp"
#include <eigen3/Eigen/Dense>


class KittiReader{
    public:
        explicit KittiReader(const std::string& drive_path);

        std::vector<std::string> scanPaths() const;

        std::vector<double> scanTimestamps() const;

        static PointCloud loadBin(const std::string& path);

        Eigen::Matrix4d loadCalibImuToVelo() const;

    private:
        std::string drive_path_;
};