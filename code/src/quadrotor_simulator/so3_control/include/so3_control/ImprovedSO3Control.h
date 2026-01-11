#ifndef __IMPROVED_SO3_CONTROL_H__
#define __IMPROVED_SO3_CONTROL_H__

#include <Eigen/Geometry>
#include <fstream>

/**
 * Improved SO3 Controller
 * 
 * Improvements over the original controller:
 * 1. Integrated complete attitude control loop (ported from simulator)
 * 2. Integrated angular velocity control loop
 * 3. Added integral terms with anti-windup
 * 4. Feedforward compensation
 * 5. Adaptive gain adjustment
 */

class ImprovedSO3Control {
public:
    ImprovedSO3Control();
    
    // Basic settings
    void setMass(double mass);
    void setGravity(double g);
    void setInertia(const Eigen::Matrix3d& J);
    void setArmLength(double d);
    void setPropellerCoeffs(double kf, double km);
    
    // State update
    void setPosition(const Eigen::Vector3d& position);
    void setVelocity(const Eigen::Vector3d& velocity);
    void setAcceleration(const Eigen::Vector3d& acc);
    void setOrientation(const Eigen::Quaterniond& orientation);
    void setAngularVelocity(const Eigen::Vector3d& omega);
    
    // Gain settings
    void setPositionGains(const Eigen::Vector3d& kp, const Eigen::Vector3d& kv, 
                           const Eigen::Vector3d& ki = Eigen::Vector3d::Zero());
    void setAttitudeGains(const Eigen::Vector3d& kR, const Eigen::Vector3d& kOm,
                           const Eigen::Vector3d& kI = Eigen::Vector3d::Zero());
    
    // Main control calculation
    void calculateControl(const Eigen::Vector3d& des_pos,
                          const Eigen::Vector3d& des_vel,
                          const Eigen::Vector3d& des_acc,
                          const Eigen::Vector3d& des_jerk,  // Added: jerk feedforward
                          double des_yaw,
                          double des_yaw_dot);
    
    // Simplified version (compatible with old interface)
    void calculateControl(const Eigen::Vector3d& des_pos,
                          const Eigen::Vector3d& des_vel,
                          const Eigen::Vector3d& des_acc,
                          double des_yaw,
                          double des_yaw_dot,
                          const Eigen::Vector3d& kp,
                          const Eigen::Vector3d& kv);
    
    // Attitude control loop (Added: ported from simulator)
    void calculateAttitudeControl(const Eigen::Quaterniond& des_orientation,
                                   const Eigen::Vector3d& des_omega = Eigen::Vector3d::Zero());
    
    // Angular velocity control loop (Added)
    void calculateRateControl(const Eigen::Vector3d& des_omega);
    
    // Get control outputs
    const Eigen::Vector3d& getComputedForce() const;
    const Eigen::Quaterniond& getComputedOrientation() const;
    const Eigen::Vector3d& getComputedTorque() const;  // Added
    double getComputedThrust() const;  // Added
    
    // Get motor RPM (Added: complete control chain)
    Eigen::Vector4d getMotorRPM() const;
    
    // Reset integrators
    void resetIntegrators();
    
    // Debug output
    std::ofstream dataFile;
    std::ofstream dataFile_time;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // Physical parameters
    double mass_;
    double g_;
    double arm_length_;
    double kf_;  // Thrust coefficient
    double km_;  // Torque coefficient
    Eigen::Matrix3d J_;  // Inertia matrix
    
    // Current state
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d acc_;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d omega_;  // Angular velocity
    
    // Position control gains
    Eigen::Vector3d kp_;
    Eigen::Vector3d kv_;
    Eigen::Vector3d ki_pos_;
    
    // Attitude control gains
    Eigen::Vector3d kR_;
    Eigen::Vector3d kOm_;
    Eigen::Vector3d ki_att_;
    
    // Integrator states
    Eigen::Vector3d pos_integral_;
    Eigen::Vector3d att_integral_;
    double integral_limit_;
    
    // Control outputs
    Eigen::Vector3d force_;
    Eigen::Quaterniond des_orientation_;
    Eigen::Vector3d torque_;
    double thrust_;
    
    // Feedforward terms
    bool use_jerk_feedforward_;
    
    // Helper functions
    Eigen::Vector3d computeOrientationError(const Eigen::Matrix3d& R, 
                                             const Eigen::Matrix3d& Rd);
    void limitAngle(double& angle, double limit);
    void saturate(Eigen::Vector3d& vec, double limit);
};

#endif // __IMPROVED_SO3_CONTROL_H__
