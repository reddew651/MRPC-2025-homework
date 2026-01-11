#ifndef _MINIMUM_SNAP_TRAJECTORY_H_
#define _MINIMUM_SNAP_TRAJECTORY_H_

#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <vector>

/**
 * 改进的轨迹生成器 - Minimum Snap优化
 * 
 * 相比原始QP方法的改进:
 * 1. 使用Minimum Snap (最小化jerk/snap的平方积分)代替简单的多项式拟合
 * 2. 支持中间点约束松弛 (通过corridor约束)
 * 3. 更优的时间分配算法 (考虑物理约束)
 * 4. 支持梯度下降优化时间分配
 */

class MinimumSnapTrajectory {
public:
    struct Constraints {
        double max_vel;       // 最大速度
        double max_acc;       // 最大加速度
        double max_jerk;      // 最大jerk
        Constraints() : max_vel(3.0), max_acc(3.0), max_jerk(5.0) {}
    };
    
    struct OptimizeResult {
        Eigen::MatrixXd polyCoeff;
        Eigen::VectorXd timeAlloc;
        double cost;
        bool success;
    };

private:
    int order_;              // 多项式阶数 (通常是5或7)
    int derivative_order_;   // 优化的导数阶数 (4 for minimum snap)
    Constraints constraints_;
    
    // 成本矩阵计算
    Eigen::MatrixXd computeQ(int n_seg, int order, double t, int deriv_order);
    Eigen::MatrixXd computeM(int n_seg, int order, const Eigen::VectorXd& time);
    
    // 辅助函数
    double factorial(int n);
    Eigen::VectorXd timeDerivative(double t, int order, int deriv_order);
    
    // 时间优化
    Eigen::VectorXd optimizeTimeAllocation(
        const Eigen::MatrixXd& path,
        const Eigen::VectorXd& initial_time,
        const Eigen::MatrixXd& vel_bc,
        const Eigen::MatrixXd& acc_bc);

public:
    MinimumSnapTrajectory();
    ~MinimumSnapTrajectory();
    
    // 设置约束
    void setConstraints(const Constraints& constraints);
    void setConstraints(double max_vel, double max_acc, double max_jerk);
    
    // 设置多项式阶数
    void setOrder(int order);
    
    /**
     * 生成Minimum Snap轨迹
     * @param path 路径点 (N+1, 3)
     * @param vel_bc 速度边界条件 (2, 3) [起始, 终止]
     * @param acc_bc 加速度边界条件 (2, 3) [起始, 终止]
     * @param time 时间分配 (N,)
     * @return 多项式系数矩阵 (N, 3*(order+1))
     */
    Eigen::MatrixXd generateTrajectory(
        const Eigen::MatrixXd& path,
        const Eigen::MatrixXd& vel_bc,
        const Eigen::MatrixXd& acc_bc,
        const Eigen::VectorXd& time);
    
    /**
     * 带时间优化的轨迹生成
     */
    OptimizeResult generateOptimizedTrajectory(
        const Eigen::MatrixXd& path,
        const Eigen::MatrixXd& vel_bc,
        const Eigen::MatrixXd& acc_bc);
    
    /**
     * 改进的时间分配 (考虑物理约束)
     */
    Eigen::VectorXd allocateTime(
        const Eigen::MatrixXd& path,
        double max_vel,
        double max_acc);
    
    /**
     * 梯形速度规划的时间分配
     */
    Eigen::VectorXd trapezoidalTimeAllocation(
        const Eigen::MatrixXd& path,
        double max_vel,
        double max_acc);
    
    // 轨迹评估
    Eigen::Vector3d getPosition(const Eigen::MatrixXd& polyCoeff, int seg, double t);
    Eigen::Vector3d getVelocity(const Eigen::MatrixXd& polyCoeff, int seg, double t);
    Eigen::Vector3d getAcceleration(const Eigen::MatrixXd& polyCoeff, int seg, double t);
    Eigen::Vector3d getJerk(const Eigen::MatrixXd& polyCoeff, int seg, double t);
    
    // 计算轨迹代价
    double computeTrajectoryCost(const Eigen::MatrixXd& polyCoeff, 
                                  const Eigen::VectorXd& time);
    
    // 检查物理可行性
    bool checkFeasibility(const Eigen::MatrixXd& polyCoeff,
                          const Eigen::VectorXd& time,
                          double dt = 0.01);
    
    // 获取最大速度/加速度
    double getMaxVelocity(const Eigen::MatrixXd& polyCoeff,
                          const Eigen::VectorXd& time,
                          double dt = 0.01);
    double getMaxAcceleration(const Eigen::MatrixXd& polyCoeff,
                              const Eigen::VectorXd& time,
                              double dt = 0.01);
};

#endif // _MINIMUM_SNAP_TRAJECTORY_H_
