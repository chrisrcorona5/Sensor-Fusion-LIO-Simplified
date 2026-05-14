// build iterative closest point algorithm
// optimized with kd-tree for nearest neighbor search
// kd-tree from nanoflann library
// the algorithm goes as follows:
// 1. build kd-tree for target point cloud
// 2. for each point in source point cloud, find the nearest neighbor in target point
// 3. compute the transformation that minimizes the distance between the matched points
// 4. apply the transformation to the source point cloud
// 5. repeat until convergence
// What is the target point? its what we want to align to, maybe imu-based estimation? 
#include "types.hpp"
#include "icp.hpp"


// Detailed Description From nanoflann documentation:
// template<typename Distance, class DatasetAdaptor, int DIM = -1, typename IndexType = size_t>
// class nanoflann::KDTreeSingleIndexAdaptor< Distance, DatasetAdaptor, DIM, IndexType >

// kd-tree static index

// Contains the k-d trees and other information for indexing a set of points for nearest-neighbor matching.

// The class "DatasetAdaptor" must provide the following interface (can be non-virtual, inlined methods):
// Must return the number of data poins
// inline size_t kdtree_get_point_count() const { ... }
// Must return the dim'th component of the idx'th point in the class:
// inline T kdtree_get_pt(const size_t idx, int dim) const { ... }
// Optional bounding-box computation: return false to default to a standard bbox computation loop.
//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
// template <class BBOX>
// bool kdtree_get_bbox(BBOX &bb) const
// {
//    bb[0].low = ...; bb[0].high = ...;  // 0th dimension limits
//    bb[1].low = ...; bb[1].high = ...;  // 1st dimension limits
//    ...
//    return true;
// }

// Template Parameters
//     DatasetAdaptor	The user-provided adaptor (see comments above).
//     Distance	The distance metric to use: nanoflann::metric_L1, nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc.
//     DIM	Dimensionality of data points (e.g. 3 for 3D points)
//     IndexType	Will be typically size_t or int

IcpPointToPlane::IcpPointToPlane() : cfg_(Config()) {}
IcpPointToPlane::IcpPointToPlane(const Config& cfg) : cfg_(cfg) {}



std::vector<Eigen::Vector3d> IcpPointToPlane::ComputeNormals(const PointCloud& cloud, const kd_tree& index) const {
    int n = cloud.size();
    int k  = 10;
    std::vector<Eigen::Vector3d> normals(n);
    // knn search
    for (size_t i = 0; i < n; ++i) {
        float query_point[3] = {cloud[i].x, cloud[i].y, cloud[i].z};
        std::vector<kd_tree::IndexType> ret_indices(k);
        std::vector<float> out_dists_sq(k);

        index.knnSearch(&query_point[0], k, &ret_indices[0], &out_dists_sq[0]);
        
        Eigen::Matrix3Xd nbrs(3, k);
        for (int j = 0; j < k; ++j) {
            size_t idx = ret_indices[j];
            nbrs.col(j) = Eigen::Vector3d(cloud[idx].x, cloud[idx].y, cloud[idx].z);
        }


        Eigen::Vector3d mean = nbrs.rowwise().mean();
        Eigen::Matrix3d cov = (nbrs.colwise() - mean) * (nbrs.colwise() - mean).transpose();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        normals[i] = solver.eigenvectors().col(0).normalized();
    }
    return normals;
};

std::vector<int> IcpPointToPlane::FindCorrespondences(const PointCloud& source, const kd_tree& index, float max_dist_sq) const {
    std::vector<int> correspondence (source.size(), -1);
    
    for (size_t i = 0; i < source.size(); ++i) {
        float query_point[3] = {source[i].x, source[i].y, source[i].z};
        kd_tree::IndexType ret_index;
        float out_dist_sq;

        if (index.knnSearch(&query_point[0], 1, &ret_index, &out_dist_sq) > 0) {
            if (out_dist_sq < max_dist_sq) {
                correspondence[i] = static_cast<int>(ret_index);
            }
        }
    }
    return correspondence;

};


ICPResult IcpPointToPlane::align(const PointCloud& source, const PointCloud& target, const Eigen::Matrix4d& T_init) {
    ICPResult result;
    result.T = T_init;
    float max_dist_sq = static_cast<float>(cfg_.max_corr_dist * cfg_.max_corr_dist);

    PointCloudAdaptor adaptor(target);
    kd_tree index(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    index.buildIndex();

    auto targe_normals = ComputeNormals(target, index);

    for (int iter = 0; iter < cfg_.max_iterations; ++iter) {
        PointCloud source_transformed;
        for (const auto& pt : source) {
            // p_h stands for point homogeneous, p_t stands for point transformed
            Eigen::Vector4d p_h(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4d p_t = result.T * p_h;
            source_transformed.push_back({(float)p_t.x(), (float)p_t.y(), (float)p_t.z(), pt.intensity});
        }

        auto corrs = FindCorrespondences(source_transformed, index, max_dist_sq);

        Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Vector<double, 6> b = Eigen::Vector<double, 6>::Zero();
        int count = 0;

        for (size_t i = 0; i < source_transformed.size(); ++i) {
            if (corrs[i] == -1) continue;

            Eigen::Vector3d s(source_transformed[i].x, source_transformed[i].y, source_transformed[i].z);
            Eigen::Vector3d t(target[corrs[i]].x, target[corrs[i]].y, target[corrs[i]].z);
            Eigen::Vector3d n = targe_normals[corrs[i]];

            // point-to-plane error
            double d = (s - t).dot(n);
            Eigen::Vector3d cross = s.cross(n);
            Eigen::Matrix<double, 1, 6> J;
            // swap the order of cross and n to match the order of rotation and translation in the state vector\
            // still trying to figure out what caused orientation mismatch
            J << n.x(), n.y(), n.z(), cross.x(), cross.y(), cross.z();
            A += J.transpose() * J;
            b -= J.transpose() * d;
            count++;
        }
        if (count < 10) break;

        Eigen::Vector<double, 6> dx = A.ldlt().solve(b);

        Eigen::Matrix4d dT = Eigen::Matrix4d::Identity();
        Eigen::Vector3d trans_vec = dx.head<3>(); // Translation is head
        Eigen::Vector3d rot_vec = dx.tail<3>();   // Rotation is tail

        double angle = rot_vec.norm();
        if (angle > 1e-9) {
            dT.block<3,3>(0,0) = Eigen::AngleAxisd(angle, rot_vec.normalized()).toRotationMatrix();
        }
        dT.block<3,1>(0,3) = trans_vec;
        result.T = dT * result.T;

        if (dx.norm() < cfg_.convergence_threshold) {
            result.converged = true;
            break;
        }
    }
    return result;
}
