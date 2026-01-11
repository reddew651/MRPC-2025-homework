#include "minimum_snap_trajectory.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

using namespace Eigen;
using namespace std;

MinimumSnapTrajectory::MinimumSnapTrajectory() 
    : order_(7)  // 7th order polynomial (8 coefficients)
    , derivative_order_(4)  // Minimize snap (4th derivative)
{
}

MinimumSnapTrajectory::~MinimumSnapTrajectory() {
}

void MinimumSnapTrajectory::setConstraints(const Constraints& constraints) {
    constraints_ = constraints;
}

void MinimumSnapTrajectory::setConstraints(double max_vel, double max_acc, double max_jerk) {
    constraints_.max_vel = max_vel;
    constraints_.max_acc = max_acc;
    constraints_.max_jerk = max_jerk;
}

void MinimumSnapTrajectory::setOrder(int order) {
    order_ = order;
}

double MinimumSnapTrajectory::factorial(int n) {
    double result = 1.0;
    for (int i = 2; i <= n; ++i) {
        result *= i;
    }
    return result;
}

VectorXd MinimumSnapTrajectory::timeDerivative(double t, int order, int deriv_order) {
    VectorXd coeff = VectorXd::Zero(order + 1);
    for (int i = deriv_order; i <= order; ++i) {
        coeff(i) = factorial(i) / factorial(i - deriv_order) * pow(t, i - deriv_order);
    }
    return coeff;
}

MatrixXd MinimumSnapTrajectory::computeQ(int n_seg, int order, double t, int deriv_order) {
    // Q matrix for computing cost function (integral of snap)
    int n = order + 1;
    MatrixXd Q = MatrixXd::Zero(n, n);
    
    for (int i = deriv_order; i < n; ++i) {
        for (int j = deriv_order; j < n; ++j) {
            double ci = factorial(i) / factorial(i - deriv_order);
            double cj = factorial(j) / factorial(j - deriv_order);
            Q(i, j) = ci * cj * pow(t, i + j - 2 * deriv_order + 1) / 
                      (i + j - 2 * deriv_order + 1);
        }
    }
    
    return Q;
}

MatrixXd MinimumSnapTrajectory::generateTrajectory(
    const MatrixXd& path,
    const MatrixXd& vel_bc,
    const MatrixXd& acc_bc,
    const VectorXd& time) {
    
    int n_seg = time.size();        // Number of segments
    int n_coeff = order_ + 1;       // Number of coefficients per segment
    int deriv = derivative_order_;  // Order of derivative to optimize
    
    // Each segment has n_coeff unknowns, total n_seg * n_coeff
    int dim = n_seg * n_coeff;
    
    MatrixXd polyCoeff = MatrixXd::Zero(n_seg, 3 * n_coeff);
    
    // Solve for each dimension separately
    for (int d = 0; d < 3; ++d) {
        // Build constraint matrix and cost matrix
        MatrixXd A = MatrixXd::Zero(dim, dim);
        VectorXd b = VectorXd::Zero(dim);
        MatrixXd Q = MatrixXd::Zero(dim, dim);
        
        int row = 0;
        
        // 1. Start point constraints (position, velocity, acceleration, jerk)
        // Position
        A(row, 0) = 1.0;
        b(row) = path(0, d);
        row++;
        
        // Velocity
        A(row, 1) = 1.0;
        b(row) = vel_bc(0, d);
        row++;
        
        // Acceleration
        A(row, 2) = 2.0;
        b(row) = acc_bc(0, d);
        row++;
        
        // Jerk (usually set to 0)
        A(row, 3) = 6.0;
        b(row) = 0.0;
        row++;
        
        // 2. Endpoint constraints
        double t_last = time(n_seg - 1);
        int last_offset = (n_seg - 1) * n_coeff;
        
        // Position
        for (int i = 0; i < n_coeff; ++i) {
            A(row, last_offset + i) = pow(t_last, i);
        }
        b(row) = path(n_seg, d);
        row++;
        
        // Velocity
        for (int i = 1; i < n_coeff; ++i) {
            A(row, last_offset + i) = i * pow(t_last, i - 1);
        }
        b(row) = vel_bc(1, d);
        row++;
        
        // Acceleration
        for (int i = 2; i < n_coeff; ++i) {
            A(row, last_offset + i) = i * (i - 1) * pow(t_last, i - 2);
        }
        b(row) = acc_bc(1, d);
        row++;
        
        // Jerk
        for (int i = 3; i < n_coeff; ++i) {
            A(row, last_offset + i) = i * (i - 1) * (i - 2) * pow(t_last, i - 3);
        }
        b(row) = 0.0;
        row++;
        
        // 3. Intermediate point position constraints and continuity constraints
        for (int seg = 0; seg < n_seg - 1; ++seg) {
            double t = time(seg);
            int offset = seg * n_coeff;
            int next_offset = (seg + 1) * n_coeff;
            
            // Position constraint (through intermediate waypoints)
            for (int i = 0; i < n_coeff; ++i) {
                A(row, offset + i) = pow(t, i);
            }
            b(row) = path(seg + 1, d);
            row++;
            
            // Continuity constraint: position continuity
            for (int i = 0; i < n_coeff; ++i) {
                A(row, offset + i) = pow(t, i);
            }
            A(row, next_offset) = -1.0;
            b(row) = 0.0;
            row++;
            
            // Velocity continuity
            for (int i = 1; i < n_coeff; ++i) {
                A(row, offset + i) = i * pow(t, i - 1);
            }
            A(row, next_offset + 1) = -1.0;
            b(row) = 0.0;
            row++;
            
            // Acceleration continuity
            for (int i = 2; i < n_coeff; ++i) {
                A(row, offset + i) = i * (i - 1) * pow(t, i - 2);
            }
            A(row, next_offset + 2) = -2.0;
            b(row) = 0.0;
            row++;
            
            // Jerk continuity
            for (int i = 3; i < n_coeff; ++i) {
                A(row, offset + i) = i * (i - 1) * (i - 2) * pow(t, i - 3);
            }
            A(row, next_offset + 3) = -6.0;
            b(row) = 0.0;
            row++;
            
            // Snap continuity (for 7th order polynomial)
            if (order_ >= 4) {
                for (int i = 4; i < n_coeff; ++i) {
                    A(row, offset + i) = i * (i - 1) * (i - 2) * (i - 3) * pow(t, i - 4);
                }
                A(row, next_offset + 4) = -24.0;
                b(row) = 0.0;
                row++;
            }
            
            // Crackle continuity (for 7th order polynomial)
            if (order_ >= 5) {
                for (int i = 5; i < n_coeff; ++i) {
                    A(row, offset + i) = i * (i - 1) * (i - 2) * (i - 3) * (i - 4) * pow(t, i - 5);
                }
                A(row, next_offset + 5) = -120.0;
                b(row) = 0.0;
                row++;
            }
        }
        
        // Construct Q matrix (cost matrix)
        for (int seg = 0; seg < n_seg; ++seg) {
            MatrixXd Qseg = computeQ(seg, order_, time(seg), deriv);
            Q.block(seg * n_coeff, seg * n_coeff, n_coeff, n_coeff) = Qseg;
        }
        
        // Solve using pseudo-inverse (if number of constraint equations equals unknowns)
        VectorXd coeffs = A.colPivHouseholderQr().solve(b);
        
        // Fill coefficient matrix
        for (int seg = 0; seg < n_seg; ++seg) {
            for (int i = 0; i < n_coeff; ++i) {
                polyCoeff(seg, d * n_coeff + i) = coeffs(seg * n_coeff + i);
            }
        }
    }
    
    return polyCoeff;
}

VectorXd MinimumSnapTrajectory::allocateTime(
    const MatrixXd& path,
    double max_vel,
    double max_acc) {
    
    int n_seg = path.rows() - 1;
    VectorXd time = VectorXd::Zero(n_seg);
    
    for (int i = 0; i < n_seg; ++i) {
        double dist = (path.row(i + 1) - path.row(i)).norm();
        
        // Trapezoidal velocity curve considering acceleration and deceleration phases
        double t_acc = max_vel / max_acc;  // Acceleration time
        double d_acc = 0.5 * max_acc * t_acc * t_acc;  // Acceleration distance
        
        if (dist < 2 * d_acc) {
            // Short distance: triangular velocity curve
            time(i) = 2.0 * sqrt(dist / max_acc);
        } else {
            // Long distance: trapezoidal velocity curve
            double d_const = dist - 2 * d_acc;
            double t_const = d_const / max_vel;
            time(i) = 2 * t_acc + t_const;
        }
        
        // Add safety margin
        time(i) *= 1.2;
        
        // Ensure minimum time
        time(i) = max(time(i), 0.3);
    }
    
    return time;
}

VectorXd MinimumSnapTrajectory::trapezoidalTimeAllocation(
    const MatrixXd& path,
    double max_vel,
    double max_acc) {
    
    int n_seg = path.rows() - 1;
    VectorXd time = VectorXd::Zero(n_seg);
    
    // Calculate direction change angle for each segment
    vector<double> angles(n_seg);
    for (int i = 0; i < n_seg; ++i) {
        if (i == 0 || i == n_seg - 1) {
            angles[i] = 0;
        } else {
            Vector3d v1 = (path.row(i) - path.row(i - 1)).transpose();
            Vector3d v2 = (path.row(i + 1) - path.row(i)).transpose();
            double cos_angle = v1.dot(v2) / (v1.norm() * v2.norm() + 1e-6);
            cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
            angles[i] = acos(cos_angle);
        }
    }
    
    for (int i = 0; i < n_seg; ++i) {
        double dist = (path.row(i + 1) - path.row(i)).norm();
        
        // Adjust velocity limit based on turning angle
        double angle_factor = 1.0;
        if (i > 0) {
            angle_factor = 1.0 - 0.5 * angles[i - 1] / M_PI;
        }
        if (i < n_seg - 1) {
            angle_factor = min(angle_factor, 1.0 - 0.5 * angles[i] / M_PI);
        }
        
        double adjusted_vel = max_vel * angle_factor;
        adjusted_vel = max(adjusted_vel, max_vel * 0.3);  // Minimum 30% speed
        
        // Trapezoidal planning
        double t_acc = adjusted_vel / max_acc;
        double d_acc = 0.5 * max_acc * t_acc * t_acc;
        
        if (dist < 2 * d_acc) {
            time(i) = 2.0 * sqrt(dist / max_acc);
        } else {
            double d_const = dist - 2 * d_acc;
            double t_const = d_const / adjusted_vel;
            time(i) = 2 * t_acc + t_const;
        }
        
        time(i) = max(time(i), 0.3);
    }
    
    return time;
}

MinimumSnapTrajectory::OptimizeResult MinimumSnapTrajectory::generateOptimizedTrajectory(
    const MatrixXd& path,
    const MatrixXd& vel_bc,
    const MatrixXd& acc_bc) {
    
    OptimizeResult result;
    result.success = false;
    
    // Initial time allocation
    VectorXd time = trapezoidalTimeAllocation(path, constraints_.max_vel, constraints_.max_acc);
    
    // Generate trajectory
    result.polyCoeff = generateTrajectory(path, vel_bc, acc_bc, time);
    result.timeAlloc = time;
    
    // Iterative time allocation optimization
    int max_iter = 10;
    double scale_factor = 1.1;
    
    for (int iter = 0; iter < max_iter; ++iter) {
        // Check feasibility
        double max_v = getMaxVelocity(result.polyCoeff, result.timeAlloc);
        double max_a = getMaxAcceleration(result.polyCoeff, result.timeAlloc);
        
        if (max_v <= constraints_.max_vel && max_a <= constraints_.max_acc) {
            // Try to shorten time
            VectorXd new_time = result.timeAlloc * 0.95;
            MatrixXd new_coeff = generateTrajectory(path, vel_bc, acc_bc, new_time);
            
            double new_max_v = getMaxVelocity(new_coeff, new_time);
            double new_max_a = getMaxAcceleration(new_coeff, new_time);
            
            if (new_max_v <= constraints_.max_vel && new_max_a <= constraints_.max_acc) {
                result.polyCoeff = new_coeff;
                result.timeAlloc = new_time;
            } else {
                break;  // Already optimal
            }
        } else {
            // Increase time to make it feasible
            result.timeAlloc *= scale_factor;
            result.polyCoeff = generateTrajectory(path, vel_bc, acc_bc, result.timeAlloc);
        }
    }
    
    result.cost = computeTrajectoryCost(result.polyCoeff, result.timeAlloc);
    result.success = checkFeasibility(result.polyCoeff, result.timeAlloc);
    
    return result;
}

Vector3d MinimumSnapTrajectory::getPosition(const MatrixXd& polyCoeff, int seg, double t) {
    Vector3d pos;
    int n_coeff = order_ + 1;
    
    for (int d = 0; d < 3; ++d) {
        double val = 0;
        for (int i = 0; i < n_coeff; ++i) {
            val += polyCoeff(seg, d * n_coeff + i) * pow(t, i);
        }
        pos(d) = val;
    }
    
    return pos;
}

Vector3d MinimumSnapTrajectory::getVelocity(const MatrixXd& polyCoeff, int seg, double t) {
    Vector3d vel;
    int n_coeff = order_ + 1;
    
    for (int d = 0; d < 3; ++d) {
        double val = 0;
        for (int i = 1; i < n_coeff; ++i) {
            val += i * polyCoeff(seg, d * n_coeff + i) * pow(t, i - 1);
        }
        vel(d) = val;
    }
    
    return vel;
}

Vector3d MinimumSnapTrajectory::getAcceleration(const MatrixXd& polyCoeff, int seg, double t) {
    Vector3d acc;
    int n_coeff = order_ + 1;
    
    for (int d = 0; d < 3; ++d) {
        double val = 0;
        for (int i = 2; i < n_coeff; ++i) {
            val += i * (i - 1) * polyCoeff(seg, d * n_coeff + i) * pow(t, i - 2);
        }
        acc(d) = val;
    }
    
    return acc;
}

Vector3d MinimumSnapTrajectory::getJerk(const MatrixXd& polyCoeff, int seg, double t) {
    Vector3d jerk;
    int n_coeff = order_ + 1;
    
    for (int d = 0; d < 3; ++d) {
        double val = 0;
        for (int i = 3; i < n_coeff; ++i) {
            val += i * (i - 1) * (i - 2) * polyCoeff(seg, d * n_coeff + i) * pow(t, i - 3);
        }
        jerk(d) = val;
    }
    
    return jerk;
}

double MinimumSnapTrajectory::computeTrajectoryCost(const MatrixXd& polyCoeff, 
                                                     const VectorXd& time) {
    double cost = 0;
    int n_seg = time.size();
    int n_coeff = order_ + 1;
    
    // Accumulate snap integral for each segment
    for (int seg = 0; seg < n_seg; ++seg) {
        MatrixXd Q = computeQ(seg, order_, time(seg), derivative_order_);
        
        for (int d = 0; d < 3; ++d) {
            VectorXd coeff(n_coeff);
            for (int i = 0; i < n_coeff; ++i) {
                coeff(i) = polyCoeff(seg, d * n_coeff + i);
            }
            cost += coeff.transpose() * Q * coeff;
        }
    }
    
    return cost;
}

bool MinimumSnapTrajectory::checkFeasibility(const MatrixXd& polyCoeff,
                                              const VectorXd& time,
                                              double dt) {
    double max_v = getMaxVelocity(polyCoeff, time, dt);
    double max_a = getMaxAcceleration(polyCoeff, time, dt);
    
    return (max_v <= constraints_.max_vel * 1.1) && (max_a <= constraints_.max_acc * 1.1);
}

double MinimumSnapTrajectory::getMaxVelocity(const MatrixXd& polyCoeff,
                                              const VectorXd& time,
                                              double dt) {
    double max_vel = 0;
    int n_seg = time.size();
    
    for (int seg = 0; seg < n_seg; ++seg) {
        for (double t = 0; t <= time(seg); t += dt) {
            double vel = getVelocity(polyCoeff, seg, t).norm();
            max_vel = max(max_vel, vel);
        }
    }
    
    return max_vel;
}

double MinimumSnapTrajectory::getMaxAcceleration(const MatrixXd& polyCoeff,
                                                  const VectorXd& time,
                                                  double dt) {
    double max_acc = 0;
    int n_seg = time.size();
    
    for (int seg = 0; seg < n_seg; ++seg) {
        for (double t = 0; t <= time(seg); t += dt) {
            double acc = getAcceleration(polyCoeff, seg, t).norm();
            max_acc = max(max_acc, acc);
        }
    }
    
    return max_acc;
}
