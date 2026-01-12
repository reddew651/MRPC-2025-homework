#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iomanip>

using namespace std;
using namespace Eigen;

// Function to calculate derivatives of x and y
// x = ux/v, y = uy/v
// x' = (ux'v - ux v') / v^2
// x'' = ((ux''v - ux v'')v - 2v'(ux'v - ux v')) / v^3
void get_state(double t, Vector3d& pos, Vector3d& vel, Vector3d& acc) {
    double sin_t = sin(t);
    double cos_t = cos(t);
    double sin_2t = sin(2*t);
    double cos_2t = cos(2*t);
    
    // u_x = 10 cos t
    double ux = 10.0 * cos_t;
    double ux_d = -10.0 * sin_t;
    double ux_dd = -10.0 * cos_t;
    
    // u_y = 10 sin t cos t = 5 sin 2t
    double uy = 5.0 * sin_2t;
    double uy_d = 10.0 * cos_2t;
    double uy_dd = -20.0 * sin_2t;
    
    // v = 1 + sin^2 t
    double v = 1.0 + sin_t * sin_t;
    double v_d = sin_2t; // deriv of 1+sin^2(t) is 2sin(t)cos(t) = sin(2t)
    double v_dd = 2.0 * cos_2t;
    
    // x calc
    double Qx = ux_d * v - ux * v_d;
    double x = ux / v;
    double vx = Qx / (v * v);
    double ax = ((ux_dd * v - ux * v_dd) * v - 2.0 * v_d * Qx) / (v * v * v);
    
    // y calc
    double Qy = uy_d * v - uy * v_d;
    double y = uy / v;
    double vy = Qy / (v * v);
    double ay = ((uy_dd * v - uy * v_dd) * v - 2.0 * v_d * Qy) / (v * v * v);
    
    // z calc
    double z = 10.0;
    double vz = 0.0;
    double az = 0.0;
    
    pos << x, y, z;
    vel << vx, vy, vz;
    acc << ax, ay, az;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "df_node");
    ros::NodeHandle nh;
    
    // 在虚拟机中运行时，通常当前工作目录是 ~/MRPC-2025-homework/code
    // solutions 目录与 code 同级，所以使用相对路径 ../solutions/df_quaternion.csv
    string output_path = "../solutions/df_quaternion.csv";
    
    ofstream file(output_path);
    if (!file.is_open()) {
        ROS_ERROR("Could not open file %s", output_path.c_str());
        return -1;
    }
    
    // Header
    file << "时间,qx,qy,qz,qw" << endl;
    
    double t_start = 0.0;
    double t_end = 2.0 * M_PI;
    double dt = 0.02;
    double g = 9.8; // Gravity
    
    // Iterate
    for (double t = t_start; t < t_end - 1e-4; t += dt) {
        Vector3d pos, vel, acc;
        get_state(t, pos, vel, acc);
        
        // 1. Calculate Yaw (psi)
        double psi = atan2(vel(1), vel(0));
        
        // 2. Calculate Body Z axis (zb)
        // Thrust T = a + g*zw
        // zb = T / ||T||
        Vector3d T = acc + Vector3d(0, 0, g);
        Vector3d zb = T.normalized();
        
        // 3. Calculate Body Y axis (yb)
        // xc = [cos(psi), sin(psi), 0]
        Vector3d xc(cos(psi), sin(psi), 0.0);
        Vector3d yb = (zb.cross(xc)).normalized();
        
        // 4. Calculate Body X axis (xb)
        Vector3d xb = yb.cross(zb);
        
        // 5. Rotation Matrix
        Matrix3d R;
        R.col(0) = xb;
        R.col(1) = yb;
        R.col(2) = zb;
        
        // 6. Quaternion
        Quaterniond q(R);
        q.normalize();
        
        // Ensure qw >= 0
        if (q.w() < 0) {
            q.w() = -q.w();
            q.x() = -q.x();
            q.y() = -q.y();
            q.z() = -q.z();
        }
        
        // Output format check
        // | 时间 | qx | qy | qz | qw |
        // 0.00 | 0.049...
        
        file << fixed << setprecision(2) << t << ","
             << setprecision(7) << q.x() << ","
             << q.y() << ","
             << q.z() << ","
             << q.w() << endl;
    }
    
    file.close();
    ROS_INFO("Successfully generated %s", output_path.c_str());
    
    return 0;
}
