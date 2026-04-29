#include "kitti_reader.hpp"
#include "imu_reader.hpp"
#include <iostream>

int main() {
    std::string drive_path = "data/2011_09_26_drive_0001_sync";
    try {
        KittiReader kr(drive_path);
        auto paths = kr.scanPaths();
        std::cout << "Velodyne Files: " << paths.size() << std::endl;
        if (!paths.empty()) {
            std::cout << "First file: " << paths[0] << std::endl;
            auto cloud  = kr.loadBin (paths[0]);
            std::cout << "Points in first cloud: " << cloud.size() << std::endl;
        }

        auto ts = kr.scanTimestamps();
        std::cout << "Velodyne Timestamps: " << ts.size() << std::endl;
        auto T = kr.loadCalibImuToVelo();
        std::cout << "IMU to Velodyne Transformation:\n" << T << std::endl;


        ImuReader ir(drive_path);
        auto imu_ts = ir.loadTimestamps();
        std::cout << "IMU Timestamps: " << imu_ts.size() << std::endl;
        auto imu_data = ir.measurements();
        std::cout << "IMU Measurements: " << imu_data.size() << std::endl;
        if (!imu_data.empty()) {
            std::cout << "First IMU accel: " << imu_data[0].acc.transpose() << std::endl;
            std::cout << "First IMU gyro: " << imu_data[0].gyro.transpose() << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}