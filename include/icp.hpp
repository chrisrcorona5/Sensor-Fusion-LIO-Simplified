#pragma once
#include "types.hpp"
#include <eigen3/Eigen/Dense>
#include "thirdparty/nanoflann.hpp"

struct PointCloudAdaptor {
    const PointCloud& cloud;

    PointCloudAdaptor(const PointCloud& cloud) : cloud(cloud) {}

    inline size_t kdtree_get_point_count() const { return cloud.size(); }
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return cloud[idx].x;
        else if (dim == 1) return cloud[idx].y;
        else return cloud[idx].z;
    };
    template <class BBOX> bool kdtree_get_bbox(BBOX& bbox) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor, 3> kd_tree;

struct ICPResult {
    Eigen::Matrix4d T;
    bool converged = false;
    double error = 0.0;
};


class IcpPointToPlane {
    public:
        struct Config {
            int max_iterations = 20;
            double max_corr_dist = 1.2;
            float convergence_threshold = 1e-6f;
        };
        IcpPointToPlane();
        explicit IcpPointToPlane(const Config& cfg);

        ICPResult align(const PointCloud& source, const PointCloud& target, const Eigen::Matrix4d& T_init);

    private:
        Config cfg_;

        std::vector<Eigen::Vector3d> ComputeNormals(const PointCloud& cloud, const kd_tree& index) const;

        std::vector<int> FindCorrespondences(const PointCloud& source, const kd_tree& index, float max_dist_sq) const;
};