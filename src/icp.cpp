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

#include "thirdparty/nanoflann.hpp"
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



struct PointCloudAdaptor {
    const PointCloud& cloud;

    PointCloudAdaptor(const PointCloud& cloud) : cloud(cloud) {}

    inline size_t kdtree_get_point_count() const { return cloud.size(); }
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return cloud[idx].x;
        else if (dim == 1) return cloud[idx].y;
        else return cloud[idx].z;
    };
};

std::vector<Eigen::Vector3d> IcpPointToPlane::ComputeNormals(const PointCloud& cloud, int k) const {
    int n = cloud.size();
    std::vector<Eigen::Vector3d> normals(n);

    PointCloudAdaptor adaptor(cloud);
    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor, 3> kd_tree;
    
    kd_tree index(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(k));
    index.buildIndex();

    // knn search and PCA for normal estimation
}

