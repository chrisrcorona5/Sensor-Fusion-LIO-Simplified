#include "preprocessor.hpp"
#include <unordered_map>
#include <cmath>
#include <vector>

struct VoxelAggregator{
    Eigen::Vector3d sum_points = Eigen::Vector3d::Zero();
    int count = 0;

    void add(const PointXYZ& pt){
        sum_points += Eigen::Vector3d(pt.x, pt.y, pt.z); // x, y, z are the coordinates of the point
        count++;
    }

    PointXYZ get_average() const {
        PointXYZ avg;
        avg.x = static_cast<float>(sum_points.x() / count);
        avg.y = static_cast<float>(sum_points.y() / count);
        avg.z = static_cast<float>(sum_points.z() / count);
        return avg;
    }
};

struct VoxelKey{
    int x_idx, y_idx, z_idx;

    bool operator==(const VoxelKey& other) const {
        // returning true if the voxel indices are the same, meaning they belong to the same voxel in the grid
        return x_idx == other.x_idx && y_idx == other.y_idx && z_idx == other.z_idx;
    }
};

struct VoxelHash {
    size_t operator()(const VoxelKey& key) const{
        size_t hx = std::hash<int>()(key.x_idx);
        size_t hy = std::hash<int>()(key.y_idx);
        size_t hz = std::hash<int>()(key.z_idx);
        return hx ^ (hy << 1) ^ (hz << 2); // bit-shift combine hash values of the voxel indices for unique identity fo each voxel in map
    }
};
// problems with cfg calling in hpp file so moved to cpp file
Preprocessor::Preprocessor() : cfg_(Config()) {}
Preprocessor::Preprocessor(const Config& cfg) : cfg_(cfg) {}


PointCloud Preprocessor::process(const PointCloud& raw) const {
    std::unordered_map<VoxelKey, VoxelAggregator, VoxelHash> voxel_map;

    for (const auto& pt : raw){
        float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (dist_sq < cfg_.min_range * cfg_.min_range || 
            dist_sq > cfg_.max_range * cfg_.max_range) continue;
        
        VoxelKey key{
            static_cast<int>(std::floor(pt.x / cfg_.voxel_size)),
            static_cast<int>(std::floor(pt.y / cfg_.voxel_size)),
            static_cast<int>(std::floor(pt.z / cfg_.voxel_size))
        };
        // For each point, we compute distance squared from the origin and check if it falls within
        // the specified min and max range, if it does, calculate voxel key by
        // dividing the point's coordinates by the voxel size and flooring that result.
        // static_cast is used because runtime type of is int
        voxel_map[key].add(pt);
    }

    PointCloud output;
    output.reserve(voxel_map.size());
    // reserve space point cloud to voxel map size avoiding reallocation
    for (auto const& [key, aggregator] : voxel_map){
        output.push_back(aggregator.get_average());
        // for each voxel compute average point (centroid) push back xyz avg to point cloud
    }

    return output;
}


PointCloud Preprocessor::deskew(const PointCloud& cloud,
                                const Eigen::Vector3d& w_body,
                                double scan_duration) const {
    int N = cloud.size();
    PointCloud out;
    out.reserve(N);

    // w_body.norm() gives the angular velocity magnitude of the sensor 
    double w_norm = w_body.norm();
    if (w_norm < 1e-6) return cloud; // If rotation is fine then no need

    Eigen::Vector3d axis = w_body / w_norm; //Rotation axis
    
    // Kitti lidar fire sequentially, so we assume point time 
    // as proportional to its index in the scan
    for (int i=0; i<N; ++i){
        double t_frac = static_cast<double>(i) / N; // frac scan duration for each point
        double angle = w_norm * scan_duration * t_frac; // angle rotated sensor at point capture

        Eigen::AngleAxisd rotation(angle, axis);
        Eigen::Vector3d p_orig(cloud[i].x, cloud[i].y, cloud[i].z);
        Eigen::Vector3d p_corrected = rotation * p_orig; // Rotate point corrected for motion

        PointXYZ q = cloud[i];
        q.x = static_cast<float>(p_corrected.x());
        q.y = static_cast<float>(p_corrected.y());
        q.z = static_cast<float>(p_corrected.z());
        out.push_back(q);
    }

    return out;
}