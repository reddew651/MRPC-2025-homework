#ifndef _MINIMUM_SNAP_TRAJECTORY_H_
#define _MINIMUM_SNAP_TRAJECTORY_H_

#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <vector>

/**
 * Improved Trajectory Generator - Minimum Snap Optimization
 * 
 * Improvements over the original QP method:
 * 1. Uses Minimum Snap (minimizing squared integral of jerk/snap) instead of simple polynomial fitting
 * 2. Supports intermediate point constraint relaxation (through corridor constraints)
 * 3. Better time allocation algorithm (considering physical constraints)
 * 4. Supports gradient descent optimization for time allocation
 */

class MinimumSnapTrajectory {
public:
    struct Constraints {
        double max_vel;       // Maximum velocity
        double max_acc;       // Maximum acceleration
        double max_jerk;      // Maximum jerk
        Constraints() : max_vel(3.0), max_acc(3.0), max_jerk(5.0) {}
    };
    
    struct OptimizeResult {
        Eigen::MatrixXd polyCoeff;
        Eigen::VectorXd timeAlloc;
        double cost;
        bool success;
    };

private:
    int order_;              // Polynomial order (usually 5 or 7)
    int derivative_order_;   // Order of derivative to optimize (4 for minimum snap)
    Constraints constraints_;
    
    // Cost matrix computation
    Eigen::MatrixXd computeQ(int n_seg, int order, double t, int deriv_order);
    Eigen::MatrixXd computeM(int n_seg, int order, const Eigen::VectorXd& time);
    
    // Helper functions
    double factorial(int n);
    Eigen::VectorXd timeDerivative(double t, int order, int deriv_order);
    
    // Time optimization
    Eigen::VectorXd optimizeTimeAllocation(
        const Eigen::MatrixXd& path,
        const Eigen::VectorXd& initial_time,
        const Eigen::MatrixXd& vel_bc,
        const Eigen::MatrixXd& acc_bc);

public:
    MinimumSnapTrajectory();
    ~MinimumSnapTrajectory();
    
    // Set constraints
    void setConstraints(const Constraints& constraints);
    void setConstraints(double max_vel, double max_acc, double max_jerk);
    
    // Set polynomial order
    void setOrder(int order);
    
    /**
     * Generate Minimum Snap trajectory
     * @param path Waypoints (N+1, 3)
     * @param vel_bc Velocity boundary conditions (2, 3) [start, end]
     * @param acc_bc Acceleration boundary conditions (2, 3) [start, end]
     * @param time Time allocation (N,)
     * @return Polynomial coefficient matrix (N, 3*(order+1))
     */
    Eigen::MatrixXd generateTrajectory(
        const Eigen::MatrixXd& path,
        const Eigen::MatrixXd& vel_bc,
        const Eigen::MatrixXd& acc_bc,
        const Eigen::VectorXd& time);
    
    /**
     * Trajectory generation with time optimization
     */
    OptimizeResult generateOptimizedTrajectory(
        const Eigen::MatrixXd& path,
        const Eigen::MatrixXd& vel_bc,
        const Eigen::MatrixXd& acc_bc);
    
    /**
     * Improved time allocation (considering physical constraints)
     */
    Eigen::VectorXd allocateTime(
        const Eigen::MatrixXd& path,
        double max_vel,
        double max_acc);
    
    /**
     * Trapezoidal velocity profile time allocation
     */
    Eigen::VectorXd trapezoidalTimeAllocation(
        const Eigen::MatrixXd& path,
        double max_vel,
        double max_acc);
    
    // Trajectory evaluation
    Eigen::Vector3d getPosition(const Eigen::MatrixXd& polyCoeff, int seg, double t);
    Eigen::Vector3d getVelocity(const Eigen::MatrixXd& polyCoeff, int seg, double t);
    Eigen::Vector3d getAcceleration(const Eigen::MatrixXd& polyCoeff, int seg, double t);
    Eigen::Vector3d getJerk(const Eigen::MatrixXd& polyCoeff, int seg, double t);
    
    // Compute trajectory cost
    double computeTrajectoryCost(const Eigen::MatrixXd& polyCoeff, 
                                  const Eigen::VectorXd& time);
    
    // Check physical feasibility
    bool checkFeasibility(const Eigen::MatrixXd& polyCoeff,
                          const Eigen::VectorXd& time,
                          double dt = 0.01);
    
    // Get maximum velocity/acceleration
    double getMaxVelocity(const Eigen::MatrixXd& polyCoeff,
                          const Eigen::VectorXd& time,
                          double dt = 0.01);
    double getMaxAcceleration(const Eigen::MatrixXd& polyCoeff,
                              const Eigen::VectorXd& time,
                              double dt = 0.01);
};

#endif // _MINIMUM_SNAP_TRAJECTORY_H_
