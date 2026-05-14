#include "kitti_reader.hpp"
#include "imu_reader.hpp"
#include "preprocessor.hpp"
#include "icp.hpp"
#include "eskf.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>

PointCloud transformCloud(const PointCloud& cloud, const Eigen::Matrix4d& T) {
    // into a 4x4 matrix
    PointCloud out;
    out.reserve(cloud.size());
    for (const auto& p : cloud) {
        Eigen::Vector4d h(p.x, p.y, p.z, 1.0);
        Eigen::Vector4d th = T * h;
        PointXYZ q = p;
        q.x = th.x(); q.y = th.y(); q.z = th.z();
        out.push_back(q);
    }
    return out;
}

void savePose(std::ofstream& f, double ts, const State& s) {
    auto& q = s.q;
    // TUM format: timestamp tx ty tz qx qy qz qw
    f << std::fixed << std::setprecision(9) << ts << " "
      << s.p.x() << " " << s.p.y() << " " << s.p.z() << " "
      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: Kitti_lio <drive_path>\n";
        return 1;
    }
    std::string drive_path = argv[1];

    KittiReader lidar_reader(drive_path);
    ImuReader imu_reader(drive_path);
    Preprocessor prep;
    ESKF eskf;
    IcpPointToPlane icp;

    auto scan_paths = lidar_reader.scanPaths();
    auto scan_ts = lidar_reader.scanTimestamps();
    auto imu_data = imu_reader.measurements();

    Eigen::Matrix4d T_IL = lidar_reader.loadCalibImuToVelo();
    Eigen::Matrix4d T_LI = T_IL.inverse();

    std::ofstream traj_file("trajectory.txt");
    std::ofstream map_file("map.txt");

    PointCloud local_map;
    size_t imu_idx = 0;
    double prev_scan_ts = scan_ts[0];

    for (size_t k = 0; k < scan_paths.size(); ++k) {
        double cur_ts = scan_ts[k];
        std::cout << "Scan " << k << "/" << scan_paths.size() << " t=" << cur_ts << "\r" << std::flush;
        
        while (imu_idx < imu_data.size() && imu_data[imu_idx].timestamp <= cur_ts) {
            double dt = (imu_idx == 0) ? 0.01 : imu_data[imu_idx].timestamp - imu_data[imu_idx - 1].timestamp;
            if (dt > 0 && dt < 0.5)
                // let imu guess until the next lidar scan arrives, then correct with icp
                eskf.propagate(imu_data[imu_idx], dt);
            ++imu_idx;
        }
        
        auto raw = KittiReader::loadBin(scan_paths[k]);
        // w being angular velo needed to deskew by each point's timestamp
        Eigen::Vector3d w = eskf.state().q.toRotationMatrix()*(imu_data[std::min(imu_idx, imu_data.size() -1)].gyro
                                                                - eskf.state().bg);
        auto cloud = prep.deskew(prep.process(raw), w);

        if (k > 0 && !local_map.empty()) {
            Eigen::Matrix4d T_world_imu = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d R_world_imu = eskf.state().q.toRotationMatrix();
            // blocks (0,0) and (0,3) of T_world_imu are set to R_world_imu and eskf.state().p respectively, the rest is identity
            T_world_imu.block<3,3>(0,0) = R_world_imu;
            T_world_imu.block<3,1>(0,3) = eskf.state().p;
            
            Eigen::Matrix4d T_init = T_world_imu * T_IL;

            Preprocessor::Config map_cfg;
            map_cfg.voxel_size = 0.4f;
            PointCloud map_ds = Preprocessor(map_cfg).process(local_map);

            auto result = icp.align(cloud, map_ds, T_init);
            if (result.converged) {
                // Convert the ICP result to a measurement for the ESKF update
                Eigen::Matrix4d dT = result.T * T_init.inverse();
                Eigen::Vector3d dp_world = dT.block<3,1>(0,3);
                Eigen::AngleAxisd daa(dT.block<3,3>(0,0));
                Eigen::Vector3d dth_world = daa.axis() * daa.angle();
                
                Eigen::Vector3d dp_local = R_world_imu.transpose() * dp_world;
                Eigen::Vector3d dth_local = R_world_imu.transpose() * dth_world;
                Eigen::VectorXd z(6);

                z.head<3>() = dp_local;
                z.tail<3>() = dth_local;
                Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 15);
                H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
                H.block<3,3>(3,6) = Eigen::Matrix3d::Identity();
                Eigen::MatrixXd R_meas = Eigen::MatrixXd::Identity(6,6);
                R_meas.block<3,3>(0,0) *= 0.05;
                R_meas.block<3,3>(3,3) *= 0.01;

                eskf.update(H, z, R_meas);
            }

        }

        Eigen::Matrix4d T_world_imu = Eigen::Matrix4d::Identity();
        T_world_imu.block<3,3>(0,0) = eskf.state().q.toRotationMatrix();
        T_world_imu.block<3,1>(0,3) = eskf.state().p;
        Eigen::Matrix4d T_world_lidar = T_world_imu * T_IL;

        auto cloud_world = transformCloud(cloud, T_world_lidar);
        for (auto& p : cloud_world) {
            local_map.push_back(p);
            map_file << p.x << " " << p.y << " " << p.z << "\n";
        }
        if (local_map.size() > 150000)
            local_map.erase(local_map.begin(),
                            local_map.begin() + local_map.size() - 150000);
        savePose(traj_file, cur_ts, eskf.state());
        prev_scan_ts = cur_ts;
    }
    std::cout << "\nDone. Trajectory saved to trajectory.txt, map saved to map.txt\n";
    return 0;
}

// Problems: Orientation of the point cloud is not correct, maybe the T_IL is wrong? or the way we apply it?
// Trajectory is correct but we need to have a correctly updating point cloud
// so the map.txt is incorrect orientation but the trajectory seems to be correct