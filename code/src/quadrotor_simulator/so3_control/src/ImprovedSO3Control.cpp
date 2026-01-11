#include <so3_control/ImprovedSO3Control.h>
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

ImprovedSO3Control::ImprovedSO3Control()
    : mass_(0.5)
    , g_(9.81)
    , arm_length_(0.17)
    , kf_(1e-6)
    , km_(1.2e-8)
    , integral_limit_(5.0)
    , thrust_(0.0)
    , use_jerk_feedforward_(true)
{
    J_ = Eigen::Matrix3d::Identity() * 0.01;
    
    // 默认增益
    kp_ << 25.0, 25.0, 25.0;
    kv_ << 12.0, 12.0, 12.0;
    ki_pos_ << 0.5, 0.5, 0.5;  // 小的积分增益
    
    kR_ << 1.5, 1.5, 1.0;
    kOm_ << 0.13, 0.13, 0.1;
    ki_att_ << 0.01, 0.01, 0.01;
    
    // 初始化状态
    pos_.setZero();
    vel_.setZero();
    acc_.setZero();
    orientation_ = Eigen::Quaterniond::Identity();
    omega_.setZero();
    
    // 初始化积分器
    pos_integral_.setZero();
    att_integral_.setZero();
    
    force_.setZero();
    des_orientation_ = Eigen::Quaterniond::Identity();
    torque_.setZero();
}

void ImprovedSO3Control::setMass(double mass) {
    mass_ = mass;
}

void ImprovedSO3Control::setGravity(double g) {
    g_ = g;
}

void ImprovedSO3Control::setInertia(const Eigen::Matrix3d& J) {
    J_ = J;
}

void ImprovedSO3Control::setArmLength(double d) {
    arm_length_ = d;
}

void ImprovedSO3Control::setPropellerCoeffs(double kf, double km) {
    kf_ = kf;
    km_ = km;
}

void ImprovedSO3Control::setPosition(const Eigen::Vector3d& position) {
    pos_ = position;
}

void ImprovedSO3Control::setVelocity(const Eigen::Vector3d& velocity) {
    vel_ = velocity;
}

void ImprovedSO3Control::setAcceleration(const Eigen::Vector3d& acc) {
    acc_ = acc;
}

void ImprovedSO3Control::setOrientation(const Eigen::Quaterniond& orientation) {
    orientation_ = orientation;
}

void ImprovedSO3Control::setAngularVelocity(const Eigen::Vector3d& omega) {
    omega_ = omega;
}

void ImprovedSO3Control::setPositionGains(const Eigen::Vector3d& kp, 
                                           const Eigen::Vector3d& kv,
                                           const Eigen::Vector3d& ki) {
    kp_ = kp;
    kv_ = kv;
    ki_pos_ = ki;
}

void ImprovedSO3Control::setAttitudeGains(const Eigen::Vector3d& kR, 
                                           const Eigen::Vector3d& kOm,
                                           const Eigen::Vector3d& kI) {
    kR_ = kR;
    kOm_ = kOm;
    ki_att_ = kI;
}

void ImprovedSO3Control::calculateControl(const Eigen::Vector3d& des_pos,
                                           const Eigen::Vector3d& des_vel,
                                           const Eigen::Vector3d& des_acc,
                                           const Eigen::Vector3d& des_jerk,
                                           double des_yaw,
                                           double des_yaw_dot) {
    // ============ 位置控制环 (外环) ============
    
    // 位置和速度误差
    Eigen::Vector3d pos_error = des_pos - pos_;
    Eigen::Vector3d vel_error = des_vel - vel_;
    
    // 更新积分器 (带抗饱和)
    pos_integral_ += pos_error * 0.01;  // 假设10ms控制周期
    saturate(pos_integral_, integral_limit_);
    
    // 自适应加速度误差增益
    Eigen::Vector3d totalError = pos_error + vel_error + (des_acc - acc_);
    Eigen::Vector3d ka(
        std::abs(totalError[0]) > 3 ? 0 : (std::abs(totalError[0]) * 0.2),
        std::abs(totalError[1]) > 3 ? 0 : (std::abs(totalError[1]) * 0.2),
        std::abs(totalError[2]) > 3 ? 0 : (std::abs(totalError[2]) * 0.2)
    );
    
    // 计算期望力 (包含前馈)
    force_ = kp_.asDiagonal() * pos_error +
             kv_.asDiagonal() * vel_error +
             ki_pos_.asDiagonal() * pos_integral_ +
             mass_ * des_acc +
             mass_ * ka.asDiagonal() * (des_acc - acc_) +
             mass_ * g_ * Eigen::Vector3d(0, 0, 1);
    
    // Jerk前馈 (可选)
    if (use_jerk_feedforward_ && des_jerk.norm() < 10.0) {
        // 简化的jerk前馈，改善动态响应
        force_ += mass_ * 0.02 * des_jerk;  // 小增益的jerk前馈
    }
    
    // ============ 姿态角限制 (安全限制) ============
    double max_tilt = M_PI / 4;  // 45度最大倾斜
    double c = cos(max_tilt);
    
    Eigen::Vector3d f_horizontal;
    f_horizontal << force_(0), force_(1), 0;
    
    if (Eigen::Vector3d(0, 0, 1).dot(force_.normalized()) < c) {
        // 需要限制倾斜角
        Eigen::Vector3d f_cmd = force_ - mass_ * g_ * Eigen::Vector3d(0, 0, 1);
        double nf = f_cmd.norm();
        double A = c * c * nf * nf - f_cmd(2) * f_cmd(2);
        double B = 2 * (c * c - 1) * f_cmd(2) * mass_ * g_;
        double C = (c * c - 1) * mass_ * mass_ * g_ * g_;
        
        double discriminant = B * B - 4 * A * C;
        if (discriminant >= 0 && std::abs(A) > 1e-6) {
            double s = (-B + sqrt(discriminant)) / (2 * A);
            force_ = s * f_cmd + mass_ * g_ * Eigen::Vector3d(0, 0, 1);
        }
    }
    
    // ============ 计算期望姿态 ============
    Eigen::Vector3d b1c, b2c, b3c;
    Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);
    
    if (force_.norm() > 1e-6) {
        b3c = force_.normalized();
    } else {
        b3c = Eigen::Vector3d(0, 0, 1);
    }
    
    b2c = b3c.cross(b1d).normalized();
    b1c = b2c.cross(b3c).normalized();
    
    Eigen::Matrix3d R_des;
    R_des << b1c, b2c, b3c;
    
    des_orientation_ = Eigen::Quaterniond(R_des);
    
    // 计算推力标量
    thrust_ = force_.dot(orientation_.toRotationMatrix().col(2));
    thrust_ = std::max(0.0, thrust_);
    
    // ============ 姿态控制环 (内环) ============
    calculateAttitudeControl(des_orientation_, Eigen::Vector3d(0, 0, des_yaw_dot));
    
    // 调试记录
    ros::Time currentTime = ros::Time::now();
    if (!dataFile.is_open()) {
        dataFile.open("/home/stuwork/MRPC-2025-homework/code/src/quadrotor_simulator/so3_control/src/control_data.txt", 
                      std::ios::out | std::ios::trunc);
    }
    
    dataFile << currentTime << " ";
    dataFile << des_pos(0) << " " << des_pos(1) << " " << des_pos(2) << " ";
    dataFile << pos_(0) << " " << pos_(1) << " " << pos_(2) << "\n";
    
    if ((pos_ - Eigen::Vector3d(10.0, -4.0, 0.0)).norm() <= 2.0) {
        if (!dataFile_time.is_open()) {
            dataFile_time.open("/home/stuwork/MRPC-2025-homework/code/src/quadrotor_simulator/so3_control/src/control_timedata.txt",
                               std::ios::out | std::ios::trunc);
            dataFile_time << currentTime << " ";
        }
    }
}

void ImprovedSO3Control::calculateControl(const Eigen::Vector3d& des_pos,
                                           const Eigen::Vector3d& des_vel,
                                           const Eigen::Vector3d& des_acc,
                                           double des_yaw,
                                           double des_yaw_dot,
                                           const Eigen::Vector3d& kp,
                                           const Eigen::Vector3d& kv) {
    kp_ = kp;
    kv_ = kv;
    calculateControl(des_pos, des_vel, des_acc, Eigen::Vector3d::Zero(), des_yaw, des_yaw_dot);
}

void ImprovedSO3Control::calculateAttitudeControl(const Eigen::Quaterniond& des_orientation,
                                                    const Eigen::Vector3d& des_omega) {
    // 从simulator移植的姿态控制逻辑
    Eigen::Matrix3d R = orientation_.toRotationMatrix();
    Eigen::Matrix3d Rd = des_orientation.toRotationMatrix();
    
    // 计算姿态误差 (SO3上的误差)
    Eigen::Vector3d eR = computeOrientationError(R, Rd);
    
    // 角速度误差
    Eigen::Vector3d eOm = omega_ - des_omega;
    
    // 更新姿态积分器
    att_integral_ += eR * 0.01;
    saturate(att_integral_, 1.0);
    
    // 计算力矩 (基于simulator中的getControl)
    // M = -kR * eR - kOm * eOm + omega x (J * omega)
    Eigen::Vector3d gyroscopic = omega_.cross(J_ * omega_);
    
    torque_ = -kR_.asDiagonal() * eR 
              - kOm_.asDiagonal() * eOm 
              - ki_att_.asDiagonal() * att_integral_
              + gyroscopic;
}

void ImprovedSO3Control::calculateRateControl(const Eigen::Vector3d& des_omega) {
    // 简单的角速度P控制
    Eigen::Vector3d omega_error = des_omega - omega_;
    
    // 使用姿态增益的一部分
    torque_ = kOm_.asDiagonal() * omega_error + omega_.cross(J_ * omega_);
}

Eigen::Vector3d ImprovedSO3Control::computeOrientationError(const Eigen::Matrix3d& R,
                                                              const Eigen::Matrix3d& Rd) {
    // Vee map of (Rd^T * R - R^T * Rd) / 2
    Eigen::Matrix3d error_matrix = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
    
    Eigen::Vector3d eR;
    eR << error_matrix(2, 1), error_matrix(0, 2), error_matrix(1, 0);
    
    return eR;
}

const Eigen::Vector3d& ImprovedSO3Control::getComputedForce() const {
    return force_;
}

const Eigen::Quaterniond& ImprovedSO3Control::getComputedOrientation() const {
    return des_orientation_;
}

const Eigen::Vector3d& ImprovedSO3Control::getComputedTorque() const {
    return torque_;
}

double ImprovedSO3Control::getComputedThrust() const {
    return thrust_;
}

Eigen::Vector4d ImprovedSO3Control::getMotorRPM() const {
    // 从推力和力矩计算电机转速
    // 这是从simulator中移植的混控逻辑
    
    double d = arm_length_;
    
    // 混控矩阵求解 (参考quadrotor_simulator_so3.cpp)
    Eigen::Vector4d w_sq;
    w_sq(0) = thrust_ / (4 * kf_) - torque_(1) / (2 * d * kf_) + torque_(2) / (4 * km_);
    w_sq(1) = thrust_ / (4 * kf_) + torque_(1) / (2 * d * kf_) + torque_(2) / (4 * km_);
    w_sq(2) = thrust_ / (4 * kf_) + torque_(0) / (2 * d * kf_) - torque_(2) / (4 * km_);
    w_sq(3) = thrust_ / (4 * kf_) - torque_(0) / (2 * d * kf_) - torque_(2) / (4 * km_);
    
    Eigen::Vector4d rpm;
    for (int i = 0; i < 4; ++i) {
        if (w_sq(i) < 0) w_sq(i) = 0;
        rpm(i) = sqrt(w_sq(i));
    }
    
    return rpm;
}

void ImprovedSO3Control::resetIntegrators() {
    pos_integral_.setZero();
    att_integral_.setZero();
}

void ImprovedSO3Control::limitAngle(double& angle, double limit) {
    angle = std::max(-limit, std::min(limit, angle));
}

void ImprovedSO3Control::saturate(Eigen::Vector3d& vec, double limit) {
    for (int i = 0; i < 3; ++i) {
        vec(i) = std::max(-limit, std::min(limit, vec(i)));
    }
}
