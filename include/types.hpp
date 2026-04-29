#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>

struct ImuMeasurement
{
    double timestamp;
    Eigen::Vector3d acc; // m/s^2
    Eigen::Vector3d gyro; // rad/s
};

struct PointXYZ{
    float x, y, z, intensity;
};

using PointCloud = std::vector<PointXYZ>;

struct state{
    Eigen::Vector3d p;                  // Position
    Eigen::Vector3d v;                  // Velocity
    Eigen::Quaterniond q;               // Orientation
    Eigen::Vector3d bg;                 // Gyroscope bias
    Eigen::Vector3d ba;                 // Accelerometer bias
    Eigen::Matrix<double,15,15> P;      // Covariance matrix

    state(){
        p.setZero(); v.setZero();
        q = Eigen::Quaterniond::Identity();
        bg.setZero(); ba.setZero();
        P = Eigen::Matrix<double,15,15>::Identity() * 1e-4;
    }
};
