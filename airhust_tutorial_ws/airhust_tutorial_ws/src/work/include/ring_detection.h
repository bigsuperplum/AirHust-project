// simple_ring_detection.h - 简单环检测
#ifndef SIMPLE_RING_DETECTION_H
#define SIMPLE_RING_DETECTION_H

#include <vector>
#include <cmath>
#include <iostream>

// 假设有以下全局变量已定义（在你的主代码中）
// extern sensor_msgs::LaserScan Laser;
// extern geometry_msgs::PoseStamped pos_drone;

struct RingCenter {
    float x;
    float y;
    float radius;
    bool found;
};

// 前置声明
bool detectRingPattern(const std::vector<float>& ranges);
float calculateRingRadius(const std::vector<float>& ranges, int center_idx);

// 主函数：找圆环圆心
RingCenter findRingCenter() {
    RingCenter result;
    result.found = false;
    result.x = 0;
    result.y = 0;
    result.radius = 0;
    
    std::cout << "寻找圆环圆心..." << std::endl;
    
    // 1. 获取激光雷达数据
    std::vector<float> ranges;
    // 假设 Laser.ranges 有数据
    // ranges = Laser.ranges;
    
    // 为了测试，创建模拟数据
    ranges.resize(360);
    for (int i = 0; i < 360; i++) {
        ranges[i] = 2.0 + 0.1 * sin(i * M_PI / 180.0); // 模拟一个圆形
    }
    // 在正前方设置一个更近的点模拟环
    ranges[0] = 1.5;
    ranges[180] = 1.5;
    
    if (ranges.empty()) {
        std::cout << "无激光雷达数据" << std::endl;
        return result;
    }
    
    // 2. 寻找最近点（可能是环的边缘）
    float min_dist = 100.0;
    int min_idx = 0;
    
    for (size_t i = 0; i < ranges.size(); i++) {
        if (ranges[i] < min_dist) {
            min_dist = ranges[i];
            min_idx = i;
        }
    }
    
    std::cout << "最近点: 角度=" << min_idx << "°, 距离=" << min_dist << "m" << std::endl;
    
    // 3. 检查是否是环
    if (detectRingPattern(ranges)) {
        // 4. 计算环中心
        int opposite_idx = (min_idx + 180) % 360;
        
        float dist1 = ranges[min_idx];
        float dist2 = ranges[opposite_idx];
        
        float angle1 = min_idx * M_PI / 180.0;
        float angle2 = opposite_idx * M_PI / 180.0;
        
        // 计算中心点（相对于无人机）
        float center_x = (dist1 * cos(angle1) + dist2 * cos(angle2)) / 2.0;
        float center_y = (dist1 * sin(angle1) + dist2 * sin(angle2)) / 2.0;
        
        // 转换为世界坐标（假设 pos_drone 是无人机位置）
        // result.x = center_x + pos_drone.pose.position.x;
        // result.y = center_y + pos_drone.pose.position.y;
        result.x = center_x; // 测试用
        result.y = center_y;
        
        result.radius = calculateRingRadius(ranges, min_idx);
        result.found = true;
        
        std::cout << "找到圆环中心: (" << result.x << ", " << result.y 
                  << "), 半径: " << result.radius << "m" << std::endl;
    } else {
        std::cout << "未检测到圆环" << std::endl;
    }
    
    return result;
}

// 检测环形模式
bool detectRingPattern(const std::vector<float>& ranges) {
    if (ranges.size() < 360) return false;
    
    // 简单检测：检查对称性
    int center_idx = 0; // 假设环在正前方
    
    // 检查左右对称
    float left_dist = ranges[90];
    float right_dist = ranges[270];
    
    if (fabs(left_dist - right_dist) > 0.5) {
        return false; // 不对称
    }
    
    // 检查前后距离
    float front_dist = ranges[0];
    float back_dist = ranges[180];
    
    if (fabs(front_dist - back_dist) > 0.5) {
        return false;
    }
    
    // 简单条件：前方距离在合理范围内
    if (front_dist > 0.5 && front_dist < 3.0) {
        return true;
    }
    
    return false;
}

// 计算环半径
float calculateRingRadius(const std::vector<float>& ranges, int center_idx) {
    // 简单估算：使用前方距离作为半径
    float front_dist = ranges[0];
    
    // 或者使用多个方向平均
    float sum = 0;
    int count = 0;
    
    for (int angle : {0, 45, 90, 135, 180, 225, 270, 315}) {
        int idx = (center_idx + angle) % 360;
        if (idx < (int)ranges.size()) {
            sum += ranges[idx];
            count++;
        }
    }
    
    if (count > 0) {
        return sum / count;
    }
    
    return front_dist; // 默认使用前方距离
}

#endif // SIMPLE_RING_DETECTION_H