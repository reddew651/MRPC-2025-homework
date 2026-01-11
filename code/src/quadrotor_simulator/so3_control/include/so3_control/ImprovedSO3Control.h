#ifndef __IMPROVED_SO3_CONTROL_H__
#define __IMPROVED_SO3_CONTROL_H__

#include <Eigen/Geometry>
#include <fstream>

/**
 * 改进的SO3控制器
 * 
 * 相比原始控制器的改进:
 * 1. 集成完整的姿态控制环 (从simulator移植)
 * 2. 集成角速度控制环
 * 3. 添加积分项和抗饱和
 * 4. 前馈补偿
 * 5. 自适应增益调节
 */

class ImprovedSO3Control {
public:
    ImprovedSO3Control();
    
    // 基本设置
    void setMass(double mass);
    void setGravity(double g);
    void setInertia(const Eigen::Matrix3d& J);
    void setArmLength(double d);
    void setPropellerCoeffs(double kf, double km);
    
    // 状态更新
    void setPosition(const Eigen::Vector3d& position);
    void setVelocity(const Eigen::Vector3d& velocity);
    void setAcceleration(const Eigen::Vector3d& acc);
    void setOrientation(const Eigen::Quaterniond& orientation);
    void setAngularVelocity(const Eigen::Vector3d& omega);
    
    // 增益设置
    void setPositionGains(const Eigen::Vector3d& kp, const Eigen::Vector3d& kv, 
                           const Eigen::Vector3d& ki = Eigen::Vector3d::Zero());
    void setAttitudeGains(const Eigen::Vector3d& kR, const Eigen::Vector3d& kOm,
                           const Eigen::Vector3d& kI = Eigen::Vector3d::Zero());
    
    // 主控制计算
    void calculateControl(const Eigen::Vector3d& des_pos,
                          const Eigen::Vector3d& des_vel,
                          const Eigen::Vector3d& des_acc,
                          const Eigen::Vector3d& des_jerk,  // 新增: jerk前馈
                          double des_yaw,
                          double des_yaw_dot);
    
    // 简化版本 (兼容旧接口)
    void calculateControl(const Eigen::Vector3d& des_pos,
                          const Eigen::Vector3d& des_vel,
                          const Eigen::Vector3d& des_acc,
                          double des_yaw,
                          double des_yaw_dot,
                          const Eigen::Vector3d& kp,
                          const Eigen::Vector3d& kv);
    
    // 姿态控制环 (新增: 从simulator移植)
    void calculateAttitudeControl(const Eigen::Quaterniond& des_orientation,
                                   const Eigen::Vector3d& des_omega = Eigen::Vector3d::Zero());
    
    // 角速度控制环 (新增)
    void calculateRateControl(const Eigen::Vector3d& des_omega);
    
    // 获取控制输出
    const Eigen::Vector3d& getComputedForce() const;
    const Eigen::Quaterniond& getComputedOrientation() const;
    const Eigen::Vector3d& getComputedTorque() const;  // 新增
    double getComputedThrust() const;  // 新增
    
    // 获取电机转速 (新增: 完整控制链)
    Eigen::Vector4d getMotorRPM() const;
    
    // 重置积分器
    void resetIntegrators();
    
    // 调试输出
    std::ofstream dataFile;
    std::ofstream dataFile_time;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // 物理参数
    double mass_;
    double g_;
    double arm_length_;
    double kf_;  // 推力系数
    double km_;  // 力矩系数
    Eigen::Matrix3d J_;  // 惯性矩阵
    
    // 当前状态
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d acc_;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d omega_;  // 角速度
    
    // 位置控制增益
    Eigen::Vector3d kp_;
    Eigen::Vector3d kv_;
    Eigen::Vector3d ki_pos_;
    
    // 姿态控制增益
    Eigen::Vector3d kR_;
    Eigen::Vector3d kOm_;
    Eigen::Vector3d ki_att_;
    
    // 积分器状态
    Eigen::Vector3d pos_integral_;
    Eigen::Vector3d att_integral_;
    double integral_limit_;
    
    // 控制输出
    Eigen::Vector3d force_;
    Eigen::Quaterniond des_orientation_;
    Eigen::Vector3d torque_;
    double thrust_;
    
    // 前馈项
    bool use_jerk_feedforward_;
    
    // 辅助函数
    Eigen::Vector3d computeOrientationError(const Eigen::Matrix3d& R, 
                                             const Eigen::Matrix3d& Rd);
    void limitAngle(double& angle, double limit);
    void saturate(Eigen::Vector3d& vec, double limit);
};

#endif // __IMPROVED_SO3_CONTROL_H__
