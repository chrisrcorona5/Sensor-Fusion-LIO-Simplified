#pragma once
#include "types.hpp"
#include <eigen3/Eigen/Dense>

struct NoiseParams {
    double gyro_noise     = 1.6968e-4;  // rad/s/√Hz Noise Density
    double accel_noise    = 2.0000e-3;  // m/s²/√Hz Noise Density
    double gyro_bias_noise  = 1.9393e-5;  // rad/s/√Hz Noise Density
    double accel_bias_noise = 3.0000e-3;  // m/s²/√Hz Noise Density
};

class ESKF {
    public:
        explicit ESKF(const NoiseParams& params = {});

        void propagate(const ImuMeasurement& imu, double dt);

        void update(const Eigen::MatrixXd& H,
                    const Eigen::VectorXd& z,
                    const Eigen::MatrixXd& R);
        
        void injectAndReset();

        const State& state() const { return state_; }
        State& state() { return state_; }
    
    private:
        State state_;
        NoiseParams params_;
        
        Eigen::Matrix<double, 15, 15> computeF(const Eigen::Vector3d& acc_meas,
                                                 double dt) const;
        Eigen::Matrix<double, 15, 15> computeQ(double dt) const;

        static Eigen::Matrix3d skew(const Eigen::Vector3d& v);
};