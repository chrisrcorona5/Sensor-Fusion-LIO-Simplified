#include "eskf.hpp"
#include <cmath>

ESKF::ESKF(const NoiseParams& params) : params_(params) {}

Eigen::Matrix3d ESKF::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return S;
}

// Nominal state vector: [p, v, q, bg, ba] which is in the types.hpp file
// [p, v, q, bg, ba] all are 3D vectors = 15 states in total
// Error state vector: [δp, δv, δθ, δbg, δba]
// we must compute Q and F matrices for the error state propagation

Eigen::Matrix<double, 15, 15> ESKF::computeF(const Eigen::Vector3d& acc_meas, double dt) const {
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();

    Eigen::Matrix3d R = state_.q.toRotationMatrix();
    Eigen::Vector3d a_corr = acc_meas - state_.ba;

    // F matrix is the Jacobian error state transition matrix 
    // which describes how the error state evolves over time
    // the structure of F is derived from the continuous-time error state dynamics

    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt; // ∂p += ∂v * dt
    F.block<3,3>(3,6) = -R * skew(a_corr) * dt; // ∂v += -R * skew(a_corr) * δθ * dt
    F.block<3,3>(3,12) = -R * dt; 
    Eigen::Vector3d w_corr = state_.bg; // fill in propagation
    F.block<3,3>(6,6) = Eigen::Matrix3d::Identity();
    F.block<3,3>(6,9) = -Eigen::Matrix3d::Identity() * dt;
    return F;
}

Eigen::Matrix<double, 15, 15> ESKF::computeQ(double dt) const {
    Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
    double gn2 = params_.gyro_noise * params_.gyro_noise;
    double an2 = params_.accel_noise * params_.accel_noise;
    double gbn2 = params_.gyro_bias_noise * params_.gyro_bias_noise;
    double abn2 = params_.accel_bias_noise * params_.accel_bias_noise;
    Q.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * an2 * dt; // Acceleration noise
    Q.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * gn2  * dt; // Gyro noise
    Q.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * gbn2 * dt; // Gyro bias noise
    Q.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * abn2 * dt; // Accel bias noise
    return Q;
}

void ESKF::propagate(const ImuMeasurement& imu, double dt) {
    Eigen::Matrix3d R = state_.q.toRotationMatrix();
    Eigen::Vector3d a = imu.acc - state_.ba;
    Eigen::Vector3d w = imu.gyro - state_.bg;

    state_.p += state_.v * dt + 0.5 * (R * a) * dt * dt;
    state_.v += (R * a) * dt;
    double angle = w.norm() * dt;
    Eigen::Vector3d axis = (angle > 1e-8) ? w.normalized() : Eigen::Vector3d::UnitX();
    Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
    state_.q = (state_.q * dq).normalized();

    Eigen::Matrix<double, 15, 15> F = computeF(imu.acc, dt);
    // calculate block (6,6) jacobian orientation part with actual angular velocity
    F.block<3,3>(6,6) = (Eigen::Matrix3d::Identity() - skew(w) * dt);
    Eigen::Matrix<double, 15, 15> Q = computeQ(dt);
    // Error state covariance propagation: P = F * P * F^T + Q
    state_.P = F * state_.P * F.transpose() + Q;
}

void ESKF::update(const Eigen::MatrixXd& H,
                  const Eigen::VectorXd& z,
                  const Eigen::MatrixXd& R) {
    Eigen::MatrixXd S = H  * state_.P * H.transpose() + R;
    Eigen::MatrixXd K = state_.P * H.transpose() * S.inverse();

    Eigen::VectorXd dx = K * z;
    state_.p += dx.segment<3>(0);
    state_.v += dx.segment<3>(3);
    Eigen::Vector3d dth = dx.segment<3>(6);
    double ang = dth.norm();
    if (ang > 1e-10) {
        Eigen::Quaterniond dq(Eigen::AngleAxisd(ang, dth / ang));
        state_.q = (state_.q * dq).normalized();
    }
    state_.bg += dx.segment<3>(9);
    state_.ba += dx.segment<3>(12);
    int n = 15;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    state_.P = (I - K * H) * state_.P;
}

void ESKF::injectAndReset() {}