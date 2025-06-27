/****************************************************************************/
/*  项目名称：中邮快递车自动驾驶控制系统 - 优化预瞄距离计算模块                  */
/*  文件名称：optimal_lookahead_calculator.h                               */
/*  创建时间：2025-06-05                                                     */
/*  开发人员：RXL                                                          */
/*  项目描述：基于纯跟踪算法的动态预瞄距离优化计算器头文件                    */
/****************************************************************************/

#ifndef OPTIMAL_LOOKAHEAD_CALCULATOR_H
#define OPTIMAL_LOOKAHEAD_CALCULATOR_H

#include <vector>
#include <utility>

// 优化预瞄距离计算器类声明
class OptimalLookaheadCalculator {
private:
    double wheelbase_;
    double max_steering_angle_;
    double min_turning_radius_;
    double base_lookahead_gain_;
    double min_lookahead_;
    double max_lookahead_;
    double tracking_weight_;
    double stability_weight_;
    double smoothness_weight_;
    std::vector<double> prev_lookaheads_;
    std::vector<double> prev_errors_;
    int history_size_;
public:
    OptimalLookaheadCalculator(double wheelbase = 2.0, double max_steering_angle = 33.0);
    double calculateDistance(double lat1, double lon1, double lat2, double lon2);
    int calculateDynamicIndexFactor(const std::vector<std::pair<double, double>>& path_points, int current_index);
    double calculatePathCurvature(const std::vector<std::pair<double, double>>& path_points, int current_index);
    double calculateLateralError(double current_lat, double current_lon, double target_lat, double target_lon, double heading_angle);
    double quadraticOptimization(double base_lookahead, double curvature, double lateral_error, double vehicle_speed);
    double calculateOptimalLookahead(double vehicle_speed, double current_lat, double current_lon, double heading_angle, const std::vector<std::pair<double, double>>& path_points, int current_path_index);
    void updateHistory(double lookahead, double error);
    void setParameters(double base_gain, double min_ld, double max_ld, double track_weight, double stab_weight, double smooth_weight);
    void getPerformanceStats(double& avg_error, double& error_variance);
};

// 全局优化器实例声明
extern OptimalLookaheadCalculator g_lookahead_optimizer;

// 优化版本的calc_Lf函数
/**
 * @brief 优化版本的calc_Lf函数
 * @details 替换原有的简单线性计算，使用二次规划优化方法
 * @param vehicle_speed 车辆速度 (km/h)
 * @param current_lat 当前纬度
 * @param current_lon 当前经度
 * @param heading_angle 当前航向角 (度)
 * @param path_points 路径点集合 (纬度, 经度)
 * @param current_path_index 当前路径点索引
 * @return 优化后的预瞄距离 (m)
 */
double calc_Lf_optimized(double vehicle_speed, double current_lat, double current_lon, double heading_angle, const std::vector<std::pair<double, double>>& path_points, int current_path_index);

/**
 * @brief 简化版本的优化calc_Lf函数（兼容原有接口）
 * @param vehicle_speed 车辆速度 (km/h)
 * @return 优化后的预瞄距离 (m)
 */
double calc_Lf_optimized_simple(double vehicle_speed);

/**
 * @brief 设置优化器参数
 * @param wheelbase 轴距 (m)
 * @param max_steering_angle 最大转向角 (度)
 * @param base_gain 基础预瞄增益系数
 * @param min_ld 最小预瞄距离 (m)
 * @param max_ld 最大预瞄距离 (m)
 * @param tracking_weight 跟踪精度权重 (可选)
 * @param stability_weight 稳定性权重 (可选)
 * @param smoothness_weight 平滑性权重 (可选)
 */
void setLookaheadOptimizerParams(double wheelbase, double max_steering_angle, double base_gain, double min_ld, double max_ld, double tracking_weight = 3.5, double stability_weight = 0.8, double smoothness_weight = 0.7);

#endif // OPTIMAL_LOOKAHEAD_CALCULATOR_H