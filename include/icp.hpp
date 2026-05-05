#pragma once
#include "types.hpp"
#include <eigen3/Eigen/Dense>

struct ICPResult {
    Eigen::Matrix4d T;
    bool converged;
    double error;
};


class IcpPointToPlane {
    public:
        struct Config {
            int max_iterations = 20;
            float convergence_threshold = 1e-6f;
        };
        IcpPointToPlane();
        explicit IcpPointToPlane(const Config& cfg);

        ICPResult align(const PointCloud& source, const PointCloud& target, const Eigen::Matrix4d& T_init);

    private:
        Config cfg_;

        std::vector<Eigen::Vector3d> ComputeNormals(const PointCloud& cloud, int k = 10) const;

        std::vector<int> FindCorrespondences(const PointCloud& source, const PointCloud& target) const;


};