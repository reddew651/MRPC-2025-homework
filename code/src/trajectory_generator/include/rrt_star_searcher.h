#ifndef _RRT_STAR_SEARCHER_H
#define _RRT_STAR_SEARCHER_H

#include <iostream>
#include <vector>
#include <random>
#include <memory>
#include <algorithm>
#include <ros/ros.h>
#include <Eigen/Eigen>

/**
 * RRT* (Rapidly-exploring Random Tree Star) 3D路径规划器
 * 相比传统A*，RRT*具有概率完备性和渐进最优性
 * 特别适合高维空间和复杂障碍物环境
 */

struct RRTNode {
    Eigen::Vector3d position;
    RRTNode* parent;
    std::vector<RRTNode*> children;
    double cost;  // 从起点到此节点的路径代价
    
    RRTNode(const Eigen::Vector3d& pos) 
        : position(pos), parent(nullptr), cost(0.0) {}
};

class RRTStarSearcher {
private:
    // 地图参数
    uint8_t* data_;
    double resolution_, inv_resolution_;
    double gl_xl_, gl_yl_, gl_zl_;
    double gl_xu_, gl_yu_, gl_zu_;
    int GRID_X_SIZE_, GRID_Y_SIZE_, GRID_Z_SIZE_;
    int GLYZ_SIZE_, GLXYZ_SIZE_;
    
    // RRT* 参数
    double step_size_;           // 扩展步长
    double goal_threshold_;      // 到达目标阈值
    double rewire_radius_;       // 重连接半径
    int max_iterations_;         // 最大迭代次数
    double goal_bias_;           // 目标偏置概率 (0-1)
    
    // 随机数生成器
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dis_;
    
    // 节点存储
    std::vector<RRTNode*> tree_;
    RRTNode* start_node_;
    RRTNode* goal_node_;
    Eigen::Vector3d goal_pos_;
    
    // 私有方法
    Eigen::Vector3d sampleRandomPoint();
    RRTNode* findNearestNode(const Eigen::Vector3d& point);
    Eigen::Vector3d steer(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    bool isCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    bool isOccupied(const Eigen::Vector3d& pos);
    std::vector<RRTNode*> findNearbyNodes(const Eigen::Vector3d& point, double radius);
    void rewire(RRTNode* new_node, std::vector<RRTNode*>& nearby_nodes);
    std::vector<Eigen::Vector3d> extractPath(RRTNode* goal);
    
    // 坐标转换
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt);
    
public:
    RRTStarSearcher();
    ~RRTStarSearcher();
    
    // 初始化地图
    void initGridMap(double resolution, 
                     Eigen::Vector3d global_xyz_l, 
                     Eigen::Vector3d global_xyz_u,
                     int max_x_id, int max_y_id, int max_z_id);
    
    // 设置占据数据指针 (与A*共享)
    void setOccupancyData(uint8_t* data);
    
    // 设置RRT*参数
    void setParams(double step_size, double goal_threshold, 
                   double rewire_radius, int max_iterations,
                   double goal_bias = 0.1);
    
    // 搜索路径
    bool search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);
    
    // 获取路径
    std::vector<Eigen::Vector3d> getPath();
    
    // 获取树节点 (用于可视化)
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getTreeEdges();
    
    // 重置搜索器
    void reset();
    
    // 路径平滑
    std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& path);
};

#endif // _RRT_STAR_SEARCHER_H
