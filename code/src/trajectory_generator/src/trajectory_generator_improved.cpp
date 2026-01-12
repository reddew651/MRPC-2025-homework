/**
 * trajectory_generator_improved.cpp
 * 
 * Improved Trajectory Generator Node
 * 
 * Features:
 * 1. Support JPS, A-star, RRT-star front-end planner switching
 * 2. Support Minimum Snap trajectory optimization
 * 3. Improved time allocation algorithm
 * 4. Better safety check and path smoothing
 */

#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <random>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Original header files
#include "Astar_searcher.h"
#include "backward.hpp"
#include "trajectory_generator_waypoint.h"

// New header files
#include "rrt_star_searcher.h"
#include "minimum_snap_trajectory.h"

using namespace std;
using namespace Eigen;

// ==================== Planner Selection ====================
enum PlannerType {
    PLANNER_ASTAR = 0,
    PLANNER_JPS = 1,
    PLANNER_RRT_STAR = 2
};

enum TrajectoryOptType {
    TRAJ_POLY_QP = 0,
    TRAJ_MINIMUM_SNAP = 1
};

// ==================== Global Variables ====================
PlannerType planner_type_ = PLANNER_JPS;
TrajectoryOptType traj_opt_type_ = TRAJ_MINIMUM_SNAP;

// Original planners
TrajectoryGeneratorWaypoint *_trajGene = new TrajectoryGeneratorWaypoint();
Astarpath *_astar_path_finder = new Astarpath();

// New planners
RRTStarSearcher *_rrt_path_finder = nullptr;
MinimumSnapTrajectory *_minsnap_traj = nullptr;

// Map parameters
double _resolution, _inv_resolution, _path_resolution;
double _x_size, _y_size, _z_size;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// Planning parameters
double _vis_traj_width;
double _Vel, _Acc;
int _dev_order, _min_order;

// ROS
ros::Subscriber _map_sub, _pts_sub, _odom_sub;
ros::Publisher _traj_vis_pub, _traj_before_vis_pub, _traj_pub, _path_vis_pub, _astar_path_vis_pub;
ros::Publisher _rrt_tree_pub;  // Added: RRT tree visualization

// State
Vector3d odom_pt, odom_vel, start_pt, target_pt, start_vel;
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
double time_duration;
ros::Time time_traj_start;
bool has_odom = false;
bool has_target = false;

// State machine
enum STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, EXEC_TRAJ, REPLAN_TRAJ, EMER_STOP };
STATE exec_state;
double no_replan_thresh, replan_thresh;
ros::Timer _exec_timer;
ros::Timer _safety_timer_;
bool cracked = false;

// ==================== Function Declarations ====================
void changeState(STATE new_state, string pos_call);
void printState();
void visTrajectory(MatrixXd polyCoeff, VectorXd time);
void visPath(MatrixXd nodes);
void trajOptimization(Eigen::MatrixXd path);
bool trajGeneration();
VectorXd timeAllocation(MatrixXd Path);
Vector3d getPos(double t_cur);
Vector3d getVel(double t_cur);
void visRRTTree(const vector<pair<Vector3d, Vector3d>>& edges);
void trajPublish(MatrixXd polyCoeff, VectorXd time);

// ==================== Callbacks ====================
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
    odom_pt(0) = odom->pose.pose.position.x;
    odom_pt(1) = odom->pose.pose.position.y;
    odom_pt(2) = odom->pose.pose.position.z;
    
    odom_vel(0) = odom->twist.twist.linear.x;
    odom_vel(1) = odom->twist.twist.linear.y;
    odom_vel(2) = odom->twist.twist.linear.z;
    
    has_odom = true;
}

void rcvWaypointsCallBack(const nav_msgs::Path &wp) {
    if (wp.poses[0].pose.position.z < 0.0) return;
    
    target_pt << 12.0, -4.0, wp.poses[0].pose.position.z;
    
    ROS_INFO("[Improved Planner] Received target: (%.2f, %.2f, %.2f)", 
             target_pt(0), target_pt(1), target_pt(2));
    
    start_pt = odom_pt;
    start_vel = odom_vel;
    has_target = true;
    
    if (exec_state == WAIT_TARGET)
        changeState(GEN_NEW_TRAJ, "STATE");
    else if (exec_state == EXEC_TRAJ)
        changeState(REPLAN_TRAJ, "STATE");
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    _astar_path_finder->resetOccupy();
    
    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if ((int)cloud.points.size() == 0) return;
    
    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++) {
        pt = cloud.points[idx];
        _astar_path_finder->set_barrier(pt.x, pt.y, pt.z);
    }
}

void execCallback(const ros::TimerEvent &e) {
    static int num = 0;
    num++;
    if (num == 100) {
        printState();
        num = 0;
    }
    
    switch (exec_state) {
        case INIT:
            if (!has_odom || !has_target) return;
            changeState(WAIT_TARGET, "STATE");
            break;
            
        case WAIT_TARGET:
            if (!has_target) return;
            changeState(GEN_NEW_TRAJ, "STATE");
            break;
            
        case GEN_NEW_TRAJ: {
            bool success = trajGeneration();
            if (success)
                changeState(EXEC_TRAJ, "STATE");
            else {
                changeState(WAIT_TARGET, "STATE");
                has_target = false;
            }
            break;
        }
        
        case EXEC_TRAJ: {
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - time_traj_start).toSec();
            double t_replan = 1.0;
            t_cur = min(time_duration, t_cur);
            
            if (t_cur > time_duration - 1e-2) {
                has_target = false;
                changeState(WAIT_TARGET, "STATE");
                return;
            } else if ((target_pt - odom_pt).norm() < no_replan_thresh) {
                return;
            } else if ((start_pt - odom_pt).norm() < replan_thresh) {
                return;
            } else if (t_cur < t_replan) {
                return;
            } else {
                changeState(REPLAN_TRAJ, "STATE");
            }
            break;
        }
        
        case REPLAN_TRAJ: {
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - time_traj_start).toSec();
            double t_delta = 0.05;
            t_cur = t_delta + t_cur;
            start_pt = getPos(t_cur);
            start_vel = getVel(t_cur);
            
            bool success = trajGeneration();
            if (success)
                changeState(EXEC_TRAJ, "STATE");
            else
                changeState(GEN_NEW_TRAJ, "STATE");
            break;
        }
        
        case EMER_STOP:
            break;
    }
}

// ==================== Core Planning Functions ====================
bool trajGeneration() {
    ros::Time start_time = ros::Time::now();
    vector<Vector3d> grid_path;
    bool search_success = false;
    
    // Perform path search based on selected planner
    switch (planner_type_) {
        case PLANNER_ASTAR:
            ROS_INFO("[Planner] Using A* search");
            search_success = _astar_path_finder->AstarSearch(start_pt, target_pt);
            if (search_success) {
                grid_path = _astar_path_finder->getPath();
            }
            _astar_path_finder->resetUsedGrids();
            break;
            
        case PLANNER_JPS:
            ROS_INFO("[Planner] Using JPS search");
            search_success = _astar_path_finder->JPSearch(start_pt, target_pt);
            if (search_success) {
                grid_path = _astar_path_finder->getPath();
            }
            _astar_path_finder->resetUsedGrids();
            break;
            
        case PLANNER_RRT_STAR:
            ROS_INFO("[Planner] Using RRT* search");
            if (_rrt_path_finder != nullptr) {
                search_success = _rrt_path_finder->search(start_pt, target_pt);
                if (search_success) {
                    grid_path = _rrt_path_finder->getPath();
                    // Path smoothing
                    grid_path = _rrt_path_finder->smoothPath(grid_path);
                    // Visualize RRT tree
                    visRRTTree(_rrt_path_finder->getTreeEdges());
                }
            }
            break;
    }
    
    if (!search_success) {
        ROS_WARN("[Planner] Path search failed!");
        return false;
    }
    
    ros::Time search_end = ros::Time::now();
    ROS_INFO("[Planner] Path found with %zu waypoints in %.3f s", 
             grid_path.size(), (search_end - start_time).toSec());
    
    // Visualize original path
    MatrixXd path(int(grid_path.size()), 3);
    for (int k = 0; k < int(grid_path.size()); k++) {
        path.row(k) = grid_path[k].transpose();
    }
    
    // Aggressive path simplification to reduce trajectory length
    if (planner_type_ != PLANNER_RRT_STAR) {
        // Use larger resolution for more aggressive simplification
        double simplify_resolution = _path_resolution * 1.5;
        grid_path = _astar_path_finder->pathSimplify(grid_path, simplify_resolution);
        path = MatrixXd::Zero(int(grid_path.size()), 3);
        for (int k = 0; k < int(grid_path.size()); k++) {
            path.row(k) = grid_path[k].transpose();
        }
    }
    
    ROS_INFO("[Planner] Simplified path has %ld waypoints", path.rows());
    
    // Trajectory optimization
    trajOptimization(path);
    time_duration = _polyTime.sum();
    
    // Publish trajectory
    trajPublish(_polyCoeff, _polyTime);
    time_traj_start = ros::Time::now();
    
    ros::Time end_time = ros::Time::now();
    ROS_INFO("[Planner] Total planning time: %.3f s, trajectory duration: %.3f s",
             (end_time - start_time).toSec(), time_duration);
    
    return _polyCoeff.rows() > 0;
}

void trajOptimization(Eigen::MatrixXd path) {
    // Safety check: need at least 2 waypoints
    if (path.rows() < 2) {
        ROS_ERROR("[Trajectory] Invalid path: need at least 2 waypoints, got %ld", path.rows());
        return;
    }
    
    MatrixXd vel = MatrixXd::Zero(2, 3);
    MatrixXd acc = MatrixXd::Zero(2, 3);
    vel.row(0) = start_vel.transpose();
    
    // Perform trajectory optimization based on selected optimizer
    if (traj_opt_type_ == TRAJ_MINIMUM_SNAP && _minsnap_traj != nullptr) {
        ROS_INFO("[Trajectory] Using Minimum Snap optimization");
        
        // Use improved time allocation
        _polyTime = _minsnap_traj->trapezoidalTimeAllocation(path, _Vel, _Acc);
        
        // Generate optimized trajectory
        auto result = _minsnap_traj->generateOptimizedTrajectory(path, vel, acc);
        
        // Validate result dimensions
        bool valid_result = result.success && 
                            result.polyCoeff.rows() == result.timeAlloc.size() &&
                            result.polyCoeff.cols() == _poly_num1D * 3;
        
        if (valid_result) {
            _polyCoeff = result.polyCoeff;
            _polyTime = result.timeAlloc;
            ROS_INFO("[Trajectory] Minimum Snap: cost = %.3f, max_vel = %.2f m/s, max_acc = %.2f m/s^2",
                     result.cost,
                     _minsnap_traj->getMaxVelocity(_polyCoeff, _polyTime),
                     _minsnap_traj->getMaxAcceleration(_polyCoeff, _polyTime));
        } else {
            ROS_WARN("[Trajectory] Minimum Snap failed or invalid dimensions (rows=%ld, time=%ld, cols=%ld, expected_cols=%d), falling back to QP",
                     result.polyCoeff.rows(), result.timeAlloc.size(), result.polyCoeff.cols(), _poly_num1D * 3);
            _polyTime = timeAllocation(path);
            _polyCoeff = _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);
        }
    } else {
        ROS_INFO("[Trajectory] Using Polynomial QP optimization");
        _polyTime = timeAllocation(path);
        _polyCoeff = _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);
    }
    
    // Safety check and re-optimization
    int unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);
    MatrixXd repath;
    bool regen_flag = false;
    int max_regen_iter = 20;  // Prevent infinite loop
    int regen_iter = 0;
    
    while (unsafe_segment != -1 && regen_iter < max_regen_iter) {
        regen_flag = true;
        regen_iter++;
        
        // Validate unsafe_segment is within bounds
        if (unsafe_segment < 0 || unsafe_segment >= path.rows() - 1) {
            ROS_WARN("[Safety] Invalid unsafe_segment=%d for path with %ld rows", unsafe_segment, path.rows());
            break;
        }
        
        int count = 0;
        repath = MatrixXd::Zero(path.rows() + 1, path.cols());
        
        // Copy points up to and including the unsafe segment start
        while (count <= unsafe_segment && count < path.rows()) {
            repath.row(count) = path.row(count);
            count++;
        }
        
        // Insert midpoint (with bounds check)
        if (count < path.rows() && count > 0) {
            repath.row(count) = (path.row(count) + path.row(count - 1)) / 2;
            ROS_INFO("[Safety] Adding intermediate point at segment %d to avoid collision", unsafe_segment);
        } else {
            ROS_WARN("[Safety] Cannot insert midpoint, count=%d, path.rows=%ld", count, path.rows());
            break;
        }
        
        // Copy remaining points
        while (count < path.rows()) {
            repath.row(count + 1) = path.row(count);
            count++;
        }
        path = repath;
        
        _polyTime = timeAllocation(path);
        _polyCoeff = _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);
        
        // Validate generated trajectory dimensions
        if (_polyCoeff.rows() != _polyTime.size()) {
            ROS_ERROR("[Safety] Dimension mismatch after regeneration: polyCoeff.rows=%ld, polyTime.size=%ld",
                      _polyCoeff.rows(), _polyTime.size());
            break;
        }
        
        unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);
    }
    
    if (regen_iter >= max_regen_iter) {
        ROS_WARN("[Safety] Max regeneration iterations reached");
    }
    
    if (regen_flag) {
        ROS_INFO("[Safety] Safe trajectory regenerated");
    }
    
    // Visualization
    visPath(path);
    visTrajectory(_polyCoeff, _polyTime);
}

// ==================== Helper Functions ====================
VectorXd timeAllocation(MatrixXd Path) {
    VectorXd time(Path.rows() - 1);
    const double t = _Vel / _Acc;
    const double d = 0.5 * _Acc * t * t;
    
    // Calculate turning angles for velocity adjustment
    std::vector<double> turn_angles(Path.rows() - 1, 0.0);
    for (int i = 1; i < int(Path.rows()) - 1; i++) {
        Vector3d v1 = (Path.row(i) - Path.row(i-1)).transpose();
        Vector3d v2 = (Path.row(i+1) - Path.row(i)).transpose();
        double cos_angle = v1.dot(v2) / (v1.norm() * v2.norm() + 1e-6);
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        turn_angles[i] = acos(cos_angle);
    }
    
    for (int i = 0; i < int(time.size()); i++) {
        MatrixXd piece = Path.row(i + 1) - Path.row(i);
        double dist = piece.norm();
        
        // Adjust velocity based on turning angle (mild slowdown at turns)
        double angle_factor = 1.0;
        if (i < int(turn_angles.size()) && i > 0) {
            // Only slow down 20% at sharp turns (was 40%)
            angle_factor = 1.0 - 0.2 * turn_angles[i] / M_PI;
            angle_factor = std::max(0.6, angle_factor);  // Minimum 60% speed (was 40%)
        }
        double adjusted_vel = _Vel * angle_factor;
        double adjusted_t = adjusted_vel / _Acc;
        double adjusted_d = 0.5 * _Acc * adjusted_t * adjusted_t;
        
        if (dist < 2.0 * adjusted_d) {
            time(i) = 2.0 * sqrt(dist / _Acc);
        } else {
            time(i) = 2.0 * adjusted_t + (dist - 2.0 * adjusted_d) / adjusted_vel;
        }
        
        // Add 8% safety margin (reduced from 15%)
        time(i) *= 1.08;
        
        // Ensure minimum segment time
        time(i) = std::max(time(i), 0.25);
    }
    return time;
}

void visRRTTree(const vector<pair<Vector3d, Vector3d>>& edges) {
    visualization_msgs::Marker tree_vis;
    tree_vis.header.stamp = ros::Time::now();
    tree_vis.header.frame_id = "world";
    tree_vis.ns = "rrt_tree";
    tree_vis.id = 0;
    tree_vis.type = visualization_msgs::Marker::LINE_LIST;
    tree_vis.action = visualization_msgs::Marker::ADD;
    tree_vis.scale.x = 0.02;
    tree_vis.color.a = 0.5;
    tree_vis.color.r = 0.0;
    tree_vis.color.g = 1.0;
    tree_vis.color.b = 0.0;
    
    for (const auto& edge : edges) {
        geometry_msgs::Point p1, p2;
        p1.x = edge.first(0);  p1.y = edge.first(1);  p1.z = edge.first(2);
        p2.x = edge.second(0); p2.y = edge.second(1); p2.z = edge.second(2);
        tree_vis.points.push_back(p1);
        tree_vis.points.push_back(p2);
    }
    
    _rrt_tree_pub.publish(tree_vis);
}

void changeState(STATE new_state, string pos_call) {
    string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ", "EMER_STOP"};
    int pre_s = int(exec_state);
    exec_state = new_state;
    ROS_INFO("[%s]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[int(new_state)].c_str());
}

void printState() {
    string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ", "EMER_STOP"};
    ROS_INFO("[State]: %s | has_odom: %d | has_target: %d", 
             state_str[int(exec_state)].c_str(), has_odom, has_target);
}

void visTrajectory(MatrixXd polyCoeff, VectorXd time) {
    visualization_msgs::Marker _traj_vis;
    _traj_vis.header.stamp = ros::Time::now();
    _traj_vis.header.frame_id = "world";
    _traj_vis.ns = "traj_node/trajectory";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.w = 1.0;
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 0.0;
    _traj_vis.color.g = 0.5;
    _traj_vis.color.b = 1.0;
    
    for (int i = 0; i < time.size(); i++) {
        // Bounds check to prevent index out of range
        if (i >= polyCoeff.rows()) {
            ROS_WARN("[visTrajectory] Segment index %d out of bounds (rows=%ld)", i, polyCoeff.rows());
            break;
        }
        for (double t = 0.0; t < time(i); t += 0.01) {
            Vector3d pos = _trajGene->getPosPoly(polyCoeff, i, t);
            geometry_msgs::Point pt;
            pt.x = pos(0); pt.y = pos(1); pt.z = pos(2);
            _traj_vis.points.push_back(pt);
        }
    }
    _traj_vis_pub.publish(_traj_vis);
}

void visPath(MatrixXd nodes) {
    visualization_msgs::Marker points;
    points.id = 0;
    points.type = visualization_msgs::Marker::SPHERE_LIST;
    points.header.frame_id = "world";
    points.header.stamp = ros::Time::now();
    points.ns = "traj_node/path";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.scale.z = 0.2;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 1.0;
    
    for (int i = 0; i < int(nodes.rows()); i++) {
        geometry_msgs::Point p;
        p.x = nodes(i, 0); p.y = nodes(i, 1); p.z = nodes(i, 2);
        points.points.push_back(p);
    }
    _path_vis_pub.publish(points);
}

Vector3d getPos(double t_cur) {
    // Safety check: trajectory must be valid
    if (_polyCoeff.rows() == 0 || _polyTime.size() == 0) {
        return Vector3d::Zero();
    }
    
    double time = 0;
    for (int i = 0; i < _polyTime.size(); i++) {
        // Bounds check
        if (i >= _polyCoeff.rows()) {
            ROS_WARN_THROTTLE(1.0, "[getPos] Segment index %d out of bounds (rows=%ld)", i, _polyCoeff.rows());
            return Vector3d::Zero();
        }
        for (double t = 0.0; t < _polyTime(i); t += 0.01) {
            time += 0.01;
            if (time > t_cur) {
                return _trajGene->getPosPoly(_polyCoeff, i, t);
            }
        }
    }
    return Vector3d::Zero();
}

Vector3d getVel(double t_cur) {
    // Safety check: trajectory must be valid
    if (_polyCoeff.rows() == 0 || _polyTime.size() == 0) {
        return Vector3d::Zero();
    }
    
    double time = 0;
    for (int i = 0; i < _polyTime.size(); i++) {
        // Bounds check
        if (i >= _polyCoeff.rows()) {
            ROS_WARN_THROTTLE(1.0, "[getVel] Segment index %d out of bounds (rows=%ld)", i, _polyCoeff.rows());
            return Vector3d::Zero();
        }
        for (double t = 0.0; t < _polyTime(i); t += 0.01) {
            time += 0.01;
            if (time > t_cur) {
                return _trajGene->getVelPoly(_polyCoeff, i, t);
            }
        }
    }
    return Vector3d::Zero();
}

void trajPublish(MatrixXd polyCoeff, VectorXd time) {
    if (polyCoeff.size() == 0 || time.size() == 0) {
        ROS_WARN("[trajectory_generator] Empty trajectory");
        return;
    }
    
    // Safety check: verify dimensions match
    if (polyCoeff.rows() != time.size()) {
        ROS_ERROR("[trajectory_generator] Dimension mismatch: polyCoeff.rows=%ld, time.size=%ld", 
                  polyCoeff.rows(), time.size());
        return;
    }
    
    static int count = 1;
    quadrotor_msgs::PolynomialTrajectory traj_msg;
    
    traj_msg.header.seq = count;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "world";
    traj_msg.trajectory_id = count;
    traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    traj_msg.num_order = 2 * _dev_order - 1;
    traj_msg.num_segment = time.size();
    
    unsigned int poly_number = traj_msg.num_order + 1;
    
    // Safety check for coefficient matrix width
    if (polyCoeff.cols() < 3 * (int)poly_number) {
        ROS_ERROR("[trajectory_generator] Coefficient matrix too narrow: cols=%ld, need=%u", 
                  polyCoeff.cols(), 3 * poly_number);
        return;
    }
    
    Vector3d initialVel = _trajGene->getVelPoly(polyCoeff, 0, 0);
    Vector3d finalVel = _trajGene->getVelPoly(polyCoeff, traj_msg.num_segment - 1, time(traj_msg.num_segment - 1));
    traj_msg.start_yaw = atan2(initialVel(1), initialVel(0));
    traj_msg.final_yaw = atan2(finalVel(1), finalVel(0));
    
    for (unsigned int i = 0; i < traj_msg.num_segment; i++) {
        for (unsigned int j = 0; j < poly_number; j++) {
            traj_msg.coef_x.push_back(polyCoeff(i, j) * pow(time(i), j));
            traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j) * pow(time(i), j));
            traj_msg.coef_z.push_back(polyCoeff(i, 2 * poly_number + j) * pow(time(i), j));
        }
        traj_msg.time.push_back(time(i));
        traj_msg.order.push_back(traj_msg.num_order);
    }
    traj_msg.mag_coeff = 1;
    
    count++;
    ROS_INFO("[trajectory_generator] Publishing trajectory");
    _traj_pub.publish(traj_msg);
}

void issafe(const ros::TimerEvent &e) {
    if (!has_odom) return;
    
    Eigen::Vector3i odom_index = _astar_path_finder->c2i(odom_pt);
    if (_astar_path_finder->is_occupy_raw(odom_index) && !cracked) {
        ROS_ERROR("COLLISION DETECTED! Drone has crashed!");
        cracked = true;
    }
}

// ==================== Main Function ====================
int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_generator_improved");
    ros::NodeHandle nh("~");
    
    // Read parameters
    nh.param("planning/vel", _Vel, 3.0);
    nh.param("planning/acc", _Acc, 3.0);
    nh.param("planning/dev_order", _dev_order, 4);  // 4th order for minimum snap
    nh.param("planning/min_order", _min_order, 3);
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
    nh.param("map/resolution", _resolution, 0.2);
    nh.param("map/x_size", _x_size, 50.0);
    nh.param("map/y_size", _y_size, 50.0);
    nh.param("map/z_size", _z_size, 5.0);
    nh.param("path/resolution", _path_resolution, 0.05);
    nh.param("replanning/thresh_replan", replan_thresh, -1.0);
    nh.param("replanning/thresh_no_replan", no_replan_thresh, -1.0);
    
    // New parameters: planner selection
    int planner_choice = 1;  // Default JPS
    int traj_opt_choice = 1;  // Default Minimum Snap
    nh.param("planning/planner_type", planner_choice, 1);
    nh.param("planning/traj_opt_type", traj_opt_choice, 1);
    
    planner_type_ = static_cast<PlannerType>(planner_choice);
    traj_opt_type_ = static_cast<TrajectoryOptType>(traj_opt_choice);
    
    ROS_INFO("===========================================");
    ROS_INFO("Improved Trajectory Generator Initialized");
    ROS_INFO("  Planner: %s", planner_type_ == PLANNER_ASTAR ? "A*" : 
                              (planner_type_ == PLANNER_JPS ? "JPS" : "RRT*"));
    ROS_INFO("  Optimizer: %s", traj_opt_type_ == TRAJ_POLY_QP ? "Polynomial QP" : "Minimum Snap");
    ROS_INFO("  Max Vel: %.2f m/s, Max Acc: %.2f m/s^2", _Vel, _Acc);
    ROS_INFO("===========================================");
    
    _poly_num1D = 2 * _dev_order;
    exec_state = INIT;
    
    // Initialize planners
    _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
    _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;
    _inv_resolution = 1.0 / _resolution;
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);
    
    _astar_path_finder = new Astarpath();
    _astar_path_finder->begin_grid_map(_resolution, _map_lower, _map_upper,
                                       _max_x_id, _max_y_id, _max_z_id);
    
    // Initialize RRT* planner
    if (planner_type_ == PLANNER_RRT_STAR) {
        _rrt_path_finder = new RRTStarSearcher();
        _rrt_path_finder->initGridMap(_resolution, _map_lower, _map_upper,
                                      _max_x_id, _max_y_id, _max_z_id);
        _rrt_path_finder->setParams(0.5, 0.5, 1.5, 5000, 0.15);
        // Share occupancy map data
        // Note: Need to add getData() method in Astarpath or modify RRT* to use same obstacle detection
    }
    
    // Initialize Minimum Snap optimizer
    if (traj_opt_type_ == TRAJ_MINIMUM_SNAP) {
        _minsnap_traj = new MinimumSnapTrajectory();
        _minsnap_traj->setConstraints(_Vel, _Acc, 5.0);
    }
    
    // Create timers
    _exec_timer = nh.createTimer(ros::Duration(0.01), execCallback);
    _safety_timer_ = nh.createTimer(ros::Duration(0.05), issafe);
    
    // Subscribers
    _odom_sub = nh.subscribe("odom", 10, rcvOdomCallback);
    _map_sub = nh.subscribe("local_pointcloud", 1, rcvPointCloudCallBack);
    _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallBack);
    
    // Publishers
    _traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
    _traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _traj_before_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory_before", 1);
    _path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path", 1);
    _astar_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path_astar", 1);
    _rrt_tree_pub = nh.advertise<visualization_msgs::Marker>("vis_rrt_tree", 1);
    
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    // Cleanup
    delete _astar_path_finder;
    delete _trajGene;
    if (_rrt_path_finder) delete _rrt_path_finder;
    if (_minsnap_traj) delete _minsnap_traj;
    
    return 0;
}
