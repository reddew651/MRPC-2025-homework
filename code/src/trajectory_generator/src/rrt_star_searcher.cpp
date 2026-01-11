#include "rrt_star_searcher.h"
#include <cmath>
#include <limits>

using namespace std;
using namespace Eigen;

RRTStarSearcher::RRTStarSearcher() 
    : data_(nullptr)
    , resolution_(0.1)
    , step_size_(0.5)
    , goal_threshold_(0.5)
    , rewire_radius_(1.0)
    , max_iterations_(5000)
    , goal_bias_(0.15)
    , start_node_(nullptr)
    , goal_node_(nullptr)
{
    // 初始化随机数生成器
    std::random_device rd;
    gen_ = std::mt19937(rd());
    dis_ = std::uniform_real_distribution<double>(0.0, 1.0);
}

RRTStarSearcher::~RRTStarSearcher() {
    reset();
}

void RRTStarSearcher::initGridMap(double resolution, 
                                   Vector3d global_xyz_l, 
                                   Vector3d global_xyz_u,
                                   int max_x_id, int max_y_id, int max_z_id) {
    resolution_ = resolution;
    inv_resolution_ = 1.0 / resolution;
    
    gl_xl_ = global_xyz_l(0);
    gl_yl_ = global_xyz_l(1);
    gl_zl_ = global_xyz_l(2);
    
    gl_xu_ = global_xyz_u(0);
    gl_yu_ = global_xyz_u(1);
    gl_zu_ = global_xyz_u(2);
    
    GRID_X_SIZE_ = max_x_id;
    GRID_Y_SIZE_ = max_y_id;
    GRID_Z_SIZE_ = max_z_id;
    GLYZ_SIZE_ = GRID_Y_SIZE_ * GRID_Z_SIZE_;
    GLXYZ_SIZE_ = GRID_X_SIZE_ * GLYZ_SIZE_;
}

void RRTStarSearcher::setOccupancyData(uint8_t* data) {
    data_ = data;
}

void RRTStarSearcher::setParams(double step_size, double goal_threshold,
                                 double rewire_radius, int max_iterations,
                                 double goal_bias) {
    step_size_ = step_size;
    goal_threshold_ = goal_threshold;
    rewire_radius_ = rewire_radius;
    max_iterations_ = max_iterations;
    goal_bias_ = goal_bias;
}

Vector3i RRTStarSearcher::coord2gridIndex(const Vector3d& pt) {
    Vector3i idx;
    idx << std::min(std::max(int((pt(0) - gl_xl_) * inv_resolution_), 0), GRID_X_SIZE_ - 1),
           std::min(std::max(int((pt(1) - gl_yl_) * inv_resolution_), 0), GRID_Y_SIZE_ - 1),
           std::min(std::max(int((pt(2) - gl_zl_) * inv_resolution_), 0), GRID_Z_SIZE_ - 1);
    return idx;
}

bool RRTStarSearcher::isOccupied(const Vector3d& pos) {
    if (pos(0) < gl_xl_ || pos(0) >= gl_xu_ ||
        pos(1) < gl_yl_ || pos(1) >= gl_yu_ ||
        pos(2) < gl_zl_ || pos(2) >= gl_zu_) {
        return true;  // 超出边界视为障碍
    }
    
    Vector3i idx = coord2gridIndex(pos);
    return data_[idx(0) * GLYZ_SIZE_ + idx(1) * GRID_Z_SIZE_ + idx(2)] == 1;
}

Vector3d RRTStarSearcher::sampleRandomPoint() {
    // 使用目标偏置采样
    if (dis_(gen_) < goal_bias_) {
        return goal_pos_;
    }
    
    // 随机采样
    Vector3d random_point;
    random_point(0) = gl_xl_ + dis_(gen_) * (gl_xu_ - gl_xl_);
    random_point(1) = gl_yl_ + dis_(gen_) * (gl_yu_ - gl_yl_);
    random_point(2) = gl_zl_ + dis_(gen_) * (gl_zu_ - gl_zl_);
    
    return random_point;
}

RRTNode* RRTStarSearcher::findNearestNode(const Vector3d& point) {
    RRTNode* nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    
    for (auto& node : tree_) {
        double dist = (node->position - point).norm();
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    
    return nearest;
}

Vector3d RRTStarSearcher::steer(const Vector3d& from, const Vector3d& to) {
    Vector3d direction = to - from;
    double distance = direction.norm();
    
    if (distance <= step_size_) {
        return to;
    }
    
    return from + direction.normalized() * step_size_;
}

bool RRTStarSearcher::isCollisionFree(const Vector3d& from, const Vector3d& to) {
    Vector3d direction = to - from;
    double distance = direction.norm();
    
    if (distance < 1e-6) return !isOccupied(from);
    
    // 沿路径检查碰撞
    double check_resolution = resolution_ * 0.5;  // 检查分辨率
    int num_checks = static_cast<int>(ceil(distance / check_resolution));
    
    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        Vector3d check_point = from + t * direction;
        if (isOccupied(check_point)) {
            return false;
        }
    }
    
    return true;
}

vector<RRTNode*> RRTStarSearcher::findNearbyNodes(const Vector3d& point, double radius) {
    vector<RRTNode*> nearby;
    
    for (auto& node : tree_) {
        if ((node->position - point).norm() <= radius) {
            nearby.push_back(node);
        }
    }
    
    return nearby;
}

void RRTStarSearcher::rewire(RRTNode* new_node, vector<RRTNode*>& nearby_nodes) {
    for (auto& near_node : nearby_nodes) {
        if (near_node == new_node->parent) continue;
        
        double new_cost = new_node->cost + (new_node->position - near_node->position).norm();
        
        if (new_cost < near_node->cost && isCollisionFree(new_node->position, near_node->position)) {
            // 从旧父节点移除
            if (near_node->parent) {
                auto& children = near_node->parent->children;
                children.erase(std::remove(children.begin(), children.end(), near_node), children.end());
            }
            
            // 设置新父节点
            near_node->parent = new_node;
            near_node->cost = new_cost;
            new_node->children.push_back(near_node);
        }
    }
}

bool RRTStarSearcher::search(const Vector3d& start_pt, const Vector3d& end_pt) {
    ros::Time start_time = ros::Time::now();
    
    // 重置
    reset();
    goal_pos_ = end_pt;
    
    // 检查起点和终点有效性
    if (isOccupied(start_pt) || isOccupied(end_pt)) {
        ROS_WARN("[RRT*] Start or goal is in obstacle!");
        return false;
    }
    
    // 创建起始节点
    start_node_ = new RRTNode(start_pt);
    tree_.push_back(start_node_);
    
    RRTNode* best_goal_node = nullptr;
    double best_goal_cost = std::numeric_limits<double>::max();
    
    for (int i = 0; i < max_iterations_; ++i) {
        // 采样随机点
        Vector3d random_point = sampleRandomPoint();
        
        // 找到最近节点
        RRTNode* nearest = findNearestNode(random_point);
        
        // 转向
        Vector3d new_point = steer(nearest->position, random_point);
        
        // 碰撞检测
        if (!isCollisionFree(nearest->position, new_point)) {
            continue;
        }
        
        // 创建新节点
        RRTNode* new_node = new RRTNode(new_point);
        
        // RRT* 改进: 选择最优父节点
        vector<RRTNode*> nearby = findNearbyNodes(new_point, rewire_radius_);
        
        RRTNode* best_parent = nearest;
        double best_cost = nearest->cost + (nearest->position - new_point).norm();
        
        for (auto& near_node : nearby) {
            double tentative_cost = near_node->cost + (near_node->position - new_point).norm();
            if (tentative_cost < best_cost && isCollisionFree(near_node->position, new_point)) {
                best_parent = near_node;
                best_cost = tentative_cost;
            }
        }
        
        // 连接到最优父节点
        new_node->parent = best_parent;
        new_node->cost = best_cost;
        best_parent->children.push_back(new_node);
        tree_.push_back(new_node);
        
        // 重连接
        rewire(new_node, nearby);
        
        // 检查是否到达目标
        double dist_to_goal = (new_point - goal_pos_).norm();
        if (dist_to_goal < goal_threshold_ && isCollisionFree(new_point, goal_pos_)) {
            double total_cost = new_node->cost + dist_to_goal;
            if (total_cost < best_goal_cost) {
                best_goal_cost = total_cost;
                best_goal_node = new_node;
                ROS_INFO("[RRT*] Found path at iteration %d, cost: %.3f", i, best_goal_cost);
            }
        }
    }
    
    if (best_goal_node != nullptr) {
        // 添加目标节点
        goal_node_ = new RRTNode(goal_pos_);
        goal_node_->parent = best_goal_node;
        goal_node_->cost = best_goal_cost;
        best_goal_node->children.push_back(goal_node_);
        tree_.push_back(goal_node_);
        
        ros::Time end_time = ros::Time::now();
        ROS_INFO("[RRT*] Path found! Time: %.3f s, Nodes: %zu", 
                 (end_time - start_time).toSec(), tree_.size());
        return true;
    }
    
    ROS_WARN("[RRT*] Failed to find path after %d iterations", max_iterations_);
    return false;
}

vector<Vector3d> RRTStarSearcher::extractPath(RRTNode* goal) {
    vector<Vector3d> path;
    RRTNode* current = goal;
    
    while (current != nullptr) {
        path.push_back(current->position);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

vector<Vector3d> RRTStarSearcher::getPath() {
    if (goal_node_ == nullptr) {
        return vector<Vector3d>();
    }
    return extractPath(goal_node_);
}

vector<pair<Vector3d, Vector3d>> RRTStarSearcher::getTreeEdges() {
    vector<pair<Vector3d, Vector3d>> edges;
    
    for (auto& node : tree_) {
        if (node->parent != nullptr) {
            edges.push_back({node->parent->position, node->position});
        }
    }
    
    return edges;
}

void RRTStarSearcher::reset() {
    for (auto& node : tree_) {
        delete node;
    }
    tree_.clear();
    start_node_ = nullptr;
    goal_node_ = nullptr;
}

vector<Vector3d> RRTStarSearcher::smoothPath(const vector<Vector3d>& path) {
    if (path.size() < 3) return path;
    
    vector<Vector3d> smoothed;
    smoothed.push_back(path.front());
    
    size_t current = 0;
    while (current < path.size() - 1) {
        size_t farthest = current + 1;
        
        // 找到最远可直达的点
        for (size_t j = current + 2; j < path.size(); ++j) {
            if (isCollisionFree(path[current], path[j])) {
                farthest = j;
            }
        }
        
        smoothed.push_back(path[farthest]);
        current = farthest;
    }
    
    return smoothed;
}
