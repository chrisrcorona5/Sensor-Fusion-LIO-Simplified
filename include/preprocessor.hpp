#pragma once
#include "types.hpp"
#include <eigen3/Eigen/Dense>

class Preprocessor {
    public:
        struct Config {
            float voxel_size = 0.2f; // Voxel grid filter size in meters. (voxel_size, voxel_size, voxel_size) will be the size of each voxel.
            float min_range = 2.0f;
            float max_range = 50.0f;
        };
        Preprocessor();
        explicit Preprocessor(const Config& cfg);

        PointCloud process (const PointCloud& raw) const;

        PointCloud deskew (const PointCloud& cloud,
                           const Eigen::Vector3d& w_body,
                           double scan_duration = 0.1) const;

    private:
        Config cfg_;
        // Optional literal bounds computation if needed
        // Eigen::Vector3d ComputeMinBound(
        //     const std::vector<Eigen::Vector3d>& points) const;
        // Eigen::Vector3d ComputeMaxBound(
        //     const std::vector<Eigen::Vector3d>& points) const;
};