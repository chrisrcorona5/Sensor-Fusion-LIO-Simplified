#include "kitti_reader.hpp"
#include "imu_reader.hpp"
#include "preprocessor.hpp"
#include <iostream>
#include <iomanip>

int main() {
    std::string drive_path = "../data/2011_09_26/2011_09_26_drive_0070_sync";
    try {
        KittiReader kr(drive_path);
        ImuReader ir(drive_path);
        Preprocessor::Config cfg;
        cfg.voxel_size = 0.2f;
        cfg.min_range = 2.0f;
        cfg.max_range = 50.0f;
        Preprocessor processor(cfg);
        auto paths = kr.scanPaths();
        std::cout << "Velodyne Files: " << paths.size() << std::endl;

        auto raw_cloud = kr.loadBin(paths[0]);
        std::cout << "Preprocessor Test" << std::endl;
        std::cout << "Raw points:      " << raw_cloud.size() << std::endl;

        auto processed_cloud = processor.process(raw_cloud);
        std::cout << "New filtered points: " << processed_cloud.size() << std::endl;
        
        float reduction = (1.0f - (float)processed_cloud.size() / raw_cloud.size()) * 100.0f;
        std::cout << "Data Reduction:  " << std::fixed << std::setprecision(1) << reduction << "%" << std::endl;


        auto imu_data = ir.measurements();
        if (!imu_data.empty()) {
            // First IMU for now but later will be matching scan timestamps to IMU timestamps 
            Eigen::Vector3d w_body = imu_data[0].gyro; 
            
            std::cout << "\nDeskew Test" << std::endl;
            std::cout << "Angular Velocity: " << w_body.transpose() << " rad/s" << std::endl;

            auto deskewed_cloud = processor.deskew(raw_cloud, w_body, 0.1);

            size_t last_idx = raw_cloud.size() - 1;
            size_t target_idx = (last_idx > 157) ? (last_idx - 157) : 0; // random index from backend of cloud
            auto p_old = raw_cloud[target_idx];
            auto p_new = deskewed_cloud[target_idx];

            std::cout << "Point [" << target_idx << "] shifted from (" 
                      << p_old.x << ", " << p_old.y << ") to (" 
                      << p_new.x << ", " << p_new.y << ")" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}