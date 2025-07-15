/****************************************************************************/
/*  项目名称：中邮快递车自动驾驶控制系统 - 优化预瞄距离计算模块                  */
/*  文件名称：optimal_lookahead_calculator.cpp                              */
/*  创建时间：2025-07-10                                                     */
/*  开发人员：RXL                                                          */
/*  项目描述：基于纯跟踪算法的动态预瞄距离优化计算器                          */
/*           融合二次规划优化方法，实现最优预瞄距离Ld计算                    */
/****************************************************************************/

#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <GeographicLib/Geodesic.hpp>

/**
 * @brief 优化预瞄距离计算器类
 * @details 基于纯跟踪算法理论，融合二次规划优化方法
 *          实现动态预瞄距离计算，平衡跟踪精度与控制稳定性
 */
class OptimalLookaheadCalculator {
private:
    // 车辆硬件参数
    double wheelbase_;              // 轴距 (m)
    double max_steering_angle_;     // 最大转向角 (度)
    double min_turning_radius_;     // 最小转弯半径 (m)
    
    // 算法参数
    double base_lookahead_gain_;    // 基础预瞄增益系数
    double min_lookahead_;          // 最小预瞄距离 (m)
    double max_lookahead_;          // 最大预瞄距离 (m)
    
    // 二次规划权重参数
    double tracking_weight_;        // 跟踪精度权重
    double stability_weight_;       // 稳定性权重
    double smoothness_weight_;      // 平滑性权重
    
    // 历史数据缓存
    std::vector<double> prev_lookaheads_;  // 历史预瞄距离
    std::vector<double> prev_errors_;      // 历史跟踪误差
    int history_size_;                     // 历史数据缓存大小
    
public:
    /**
     * @brief 构造函数
     * @param wheelbase 车辆轴距 (m)
     * @param max_steering_angle 最大转向角 (度)
     */
    OptimalLookaheadCalculator(double wheelbase = 2.0, double max_steering_angle = 33.0) 
        : wheelbase_(wheelbase), 
          max_steering_angle_(max_steering_angle),
          min_turning_radius_(wheelbase / tan(max_steering_angle * M_PI / 180.0)),
          base_lookahead_gain_(1.8),
          min_lookahead_(2.0),
          max_lookahead_(20.0),
          tracking_weight_(2.5),
          stability_weight_(1.2),
          smoothness_weight_(0.8),
          history_size_(5) {
        prev_lookaheads_.reserve(history_size_);
        prev_errors_.reserve(history_size_);
    }
    
    /**
     * @brief 计算两点间距离
     * @param lat1 起点纬度
     * @param lon1 起点经度
     * @param lat2 终点纬度
     * @param lon2 终点经度
     * @return 距离 (m)
     */
    double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        double distance;
        const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
        geod.Inverse(lat1, lon1, lat2, lon2, distance);
        return distance;
    }
    
    /**
     * @brief 动态计算index_factor，基于轨迹特征和曲率变化
     * @param path_points 路径点集合
     * @param current_index 当前点索引
     * @return 动态调整的index_factor
     */
    int calculateDynamicIndexFactor(const std::vector<std::pair<double, double>>& path_points, 
                                   int current_index) {
        const double POINT_SPACING = 0.08;  // GPS轨迹标准点间距 (m)
        const int MIN_FACTOR = 3;           // 最小index_factor (约0.24m间距)
        const int MAX_FACTOR = 15;          // 最大index_factor (约1.2m间距)
        const int DEFAULT_FACTOR = 5;       // 默认index_factor (约0.4m间距)
        
        // 检查边界条件
        if (path_points.size() < 10 || current_index < 5 || current_index >= path_points.size() - 5) {
            return DEFAULT_FACTOR;
        }
        
        // 计算局部平均点间距
        double total_distance = 0.0;
        int count = 0;
        for (int i = current_index - 5; i < current_index + 5 && i < path_points.size() - 1; i++) {
            if (i >= 0) {
                double dist = calculateDistance(path_points[i].first, path_points[i].second,
                                              path_points[i+1].first, path_points[i+1].second);
                total_distance += dist;
                count++;
            }
        }
        double avg_spacing = (count > 0) ? total_distance / count : POINT_SPACING;
        
        // 计算局部曲率变化率（用于识别弯道复杂程度）
        std::vector<double> local_curvatures;
        for (int i = current_index - 3; i <= current_index + 3; i++) {
            if (i >= 2 && i < path_points.size() - 2) {
                // 简化的三点曲率计算
                auto p1 = path_points[i-2];
                auto p2 = path_points[i];
                auto p3 = path_points[i+2];
                
                double d12 = calculateDistance(p1.first, p1.second, p2.first, p2.second);
                double d23 = calculateDistance(p2.first, p2.second, p3.first, p3.second);
                double d13 = calculateDistance(p1.first, p1.second, p3.first, p3.second);
                
                if (d12 > 0.01 && d23 > 0.01 && d13 > 0.01) {
                    double s = (d12 + d23 + d13) / 2.0;
                    double area = sqrt(std::max(0.0, s * (s - d12) * (s - d23) * (s - d13)));
                    double curvature = 4.0 * area / (d12 * d23 * d13);
                    local_curvatures.push_back(curvature);
                }
            }
        }
        
        // 计算曲率变化率（标准差）
        double curvature_variation = 0.0;
        if (local_curvatures.size() > 1) {
            double mean = 0.0;
            for (double c : local_curvatures) mean += c;
            mean /= local_curvatures.size();
            
            double variance = 0.0;
            for (double c : local_curvatures) {
                variance += (c - mean) * (c - mean);
            }
            curvature_variation = sqrt(variance / local_curvatures.size());
        }
        
        // 基于点间距调整基础因子
        double spacing_factor = POINT_SPACING / avg_spacing;
        int base_factor = static_cast<int>(DEFAULT_FACTOR * spacing_factor);
        
        // 基于曲率变化率进行微调
        int dynamic_factor = base_factor;
        if (curvature_variation > 0.1) {
            // 复杂弯道：减小index_factor提高精度
            dynamic_factor = std::max(MIN_FACTOR, base_factor - 2);
        } else if (curvature_variation > 0.05) {
            // 中等弯道：适度减小index_factor
            dynamic_factor = std::max(MIN_FACTOR, base_factor - 1);
        } else if (curvature_variation < 0.01) {
            // 直线或缓弯：增大index_factor提高稳定性
            dynamic_factor = std::min(MAX_FACTOR, base_factor + 2);
        }
        
        // 确保在有效范围内
        dynamic_factor = std::max(MIN_FACTOR, std::min(MAX_FACTOR, dynamic_factor));
        
        std::cout << "动态index_factor计算: 平均点间距=" << avg_spacing 
                  << "m, 曲率变化率=" << curvature_variation 
                  << ", 计算得到index_factor=" << dynamic_factor << std::endl;
        
        return dynamic_factor;
    }
    
    /**
     * @brief 计算路径曲率
     * @param path_points 路径点集合 (纬度, 经度)
     * @param current_index 当前点索引
     * @return 路径曲率 (1/m)
     */
    double calculatePathCurvature(const std::vector<std::pair<double, double>>& path_points, 
                                 int current_index) {
        // 详细的调试输出，帮助诊断曲率计算问题
        std::cout << "曲率计算调试 - 路径点总数: " << path_points.size() 
                  << ", 当前索引: " << current_index << std::endl;
        
        // 检查路径点数量和索引有效性
        if (path_points.empty()) {
            std::cout << "曲率计算错误: 路径点为空" << std::endl;
            return 0.0;
        }
        
        if (path_points.size() < 3) {
            std::cout << "曲率计算错误: 路径点数量不足，至少需要3个点" << std::endl;
            return 0.0;
        }
        
        if (current_index < 1 || current_index >= path_points.size() - 1) {
            std::cout << "曲率计算错误: 当前索引无效 (" << current_index << ")，有效范围应为 1 到 " 
                      << (path_points.size() - 2) << std::endl;
            
            // // 尝试调整索引到有效范围
            // if (current_index < 1) {
            //     current_index = 1;
            // } else if (current_index >= path_points.size() - 1) {
            //     current_index = path_points.size() - 2;
            // }
            
            // std::cout << "曲率计算: 已调整索引至 " << current_index << std::endl;
        }
        // 动态计算index_factor，基于车速和轨迹特征
        int dynamic_factor = calculateDynamicIndexFactor(path_points, current_index);
        
        // 确保索引在安全范围内
        int safe_factor = std::min(dynamic_factor, 
                                  std::min(current_index, (int)path_points.size() - 1 - current_index));
        if (safe_factor < 1) safe_factor = 1;
        
        std::cout << "动态index_factor: 计算值=" << dynamic_factor 
                  << ", 安全值=" << safe_factor << std::endl;
        
        // 使用三点法计算曲率
        auto p1 = path_points[current_index - safe_factor];
        auto p2 = path_points[current_index];
        auto p3 = path_points[current_index + safe_factor];
        
        // 输出三点坐标，便于调试
        std::cout << "曲率计算三点坐标: " << std::endl
                  << "  P1(" << p1.first << ", " << p1.second << ")" << std::endl
                  << "  P2(" << p2.first << ", " << p2.second << ")" << std::endl
                  << "  P3(" << p3.first << ", " << p3.second << ")" << std::endl;
        
        double d12 = calculateDistance(p1.first, p1.second, p2.first, p2.second);
        double d23 = calculateDistance(p2.first, p2.second, p3.first, p3.second);
        double d13 = calculateDistance(p1.first, p1.second, p3.first, p3.second);
        
        std::cout << "曲率计算三边长: d12=" << d12 << "m, d23=" << d23 << "m, d13=" << d13 << "m" << std::endl;
        
        // 检查点间距离是否过小
        const double MIN_DISTANCE = 0.5; // 最小有效距离阈值 (m)
        if (d12 < MIN_DISTANCE || d23 < MIN_DISTANCE || d13 < MIN_DISTANCE) {
            std::cout << "曲率计算警告: 点间距离过小，可能导致计算不准确" << std::endl;
            
            // 尝试寻找更远的点来计算曲率
            int offset = 1;
            while (offset < 10 && (d12 < MIN_DISTANCE || d23 < MIN_DISTANCE || d13 < MIN_DISTANCE)) {
                // 确保索引在有效范围内
                if (current_index - offset < 0 || current_index + offset >= path_points.size()) {
                    break;
                }
                
                p1 = path_points[current_index - (safe_factor + offset)];
                p3 = path_points[current_index + (safe_factor + offset)];
                
                d12 = calculateDistance(p1.first, p1.second, p2.first, p2.second);
                d23 = calculateDistance(p2.first, p2.second, p3.first, p3.second);
                d13 = calculateDistance(p1.first, p1.second, p3.first, p3.second);
                
                std::cout << "尝试更远点(偏移=" << offset << "): d12=" << d12 
                          << "m, d23=" << d23 << "m, d13=" << d13 << "m" << std::endl;
                offset++;
            }
            
            // 如果仍然无法找到合适的点，使用小的默认曲率值而不是0
            if (d12 < MIN_DISTANCE || d23 < MIN_DISTANCE || d13 < MIN_DISTANCE) {
                std::cout << "曲率计算: 无法找到合适的点，使用默认小曲率值" << std::endl;
                return 0.001; // 使用小的默认曲率，避免完全为0
            }
        }
        
        // 使用海伦公式计算三角形面积
        double s = (d12 + d23 + d13) / 2.0;
        double area = sqrt(std::max(0.0, s * (s - d12) * (s - d23) * (s - d13)));
        
        // 曲率 = 4 * 面积 / (三边长乘积)
        double curvature = 4.0 * area / (d12 * d23 * d13);
        
        // 曲率平滑处理，避免突变
        static double prev_curvature = 0.0;
        double smoothed_curvature = 0.88 * curvature + 0.12 * prev_curvature;
        prev_curvature = smoothed_curvature;
        
        std::cout << "曲率计算结果: 原始曲率=" << curvature 
                  << ", 平滑后曲率=" << smoothed_curvature << std::endl;
        
        return smoothed_curvature;
    }
    
    /**
     * @brief 计算横向误差
     * @param current_lat 当前纬度
     * @param current_lon 当前经度
     * @param target_lat 目标点纬度
     * @param target_lon 目标点经度
     * @param heading_angle 当前航向角 (度)
     * @return 横向误差 (m)
     */
    double calculateLateralError(double current_lat, double current_lon,
                               double target_lat, double target_lon,
                               double heading_angle) {
        // 计算目标点相对于当前位置的方位角
        double azimuth;
        const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
        geod.Inverse(current_lat, current_lon, target_lat, target_lon, azimuth);
        
        // 计算角度差
        double angle_diff = azimuth - heading_angle;
        while (angle_diff > 180.0) angle_diff -= 360.0;
        while (angle_diff < -180.0) angle_diff += 360.0;
        
        // 计算距离
        double distance = calculateDistance(current_lat, current_lon, target_lat, target_lon);
        
        // 横向误差 = 距离 * sin(角度差)
        return distance * sin(angle_diff * M_PI / 180.0);
    }
    
    /**
     * @brief 基于二次规划的预瞄距离优化
     * @param base_lookahead 基础预瞄距离
     * @param curvature 路径曲率
     * @param lateral_error 横向误差
     * @param vehicle_speed 车辆速度 (km/h)
     * @return 优化后的预瞄距离 (m)
     */
    double quadraticOptimization(double base_lookahead, double curvature, 
                               double lateral_error, double vehicle_speed) {
        // 将车速转换为 m/s
        double speed_ms = vehicle_speed / 3.6;
        
        // 定义优化变量范围
        double ld_min = std::max(min_lookahead_, speed_ms * 0.5);
        double ld_max = std::min(max_lookahead_, speed_ms * 3.0);
        

        // 确保最小预瞄距离不为0，即使车速为0
        if (ld_min < 2.0 || vehicle_speed < 2.0) {
            ld_min = 2.0;  // 设置最小预瞄距离为2米
            std::cout << "车速接近0或过低（<2km/h），设置最小预瞄距离为" << ld_min << "m" << std::endl;
        }
        
        // 二次规划目标函数系数
        // J = a*Ld^2 + b*Ld + c
        double a = tracking_weight_ * curvature * curvature + 
                  stability_weight_ * 0.25 + 
                  smoothness_weight_ * 0.15;
        
        double b = -2.0 * tracking_weight_ * base_lookahead + 
                  stability_weight_ * fabs(lateral_error) * 0.2;
        
        double c = tracking_weight_ * base_lookahead * base_lookahead;
        
        // 求解二次函数最小值点
        double optimal_ld;
        if (a > 1e-6) {
            optimal_ld = -b / (2.0 * a);
        } else {
            optimal_ld = base_lookahead;
        }
        
        // 约束到合理范围
        optimal_ld = std::clamp(optimal_ld, ld_min, ld_max);
        
        // 考虑历史平滑性
        if (!prev_lookaheads_.empty()) {
            double prev_avg = 0.0;
            for (double prev_ld : prev_lookaheads_) {
                prev_avg += prev_ld;
            }
            prev_avg /= prev_lookaheads_.size();
            
            // 限制变化幅度
            double max_change = 2.0; // 最大变化2米
            optimal_ld = std::clamp(optimal_ld, prev_avg - max_change, prev_avg + max_change);
        }
        
        // 确保最终预瞄距离不接近0
        if (optimal_ld < 0.1 || vehicle_speed < 2.0) {
            optimal_ld = ld_min;
            std::cout << "警告: 计算的预瞄距离接近0或车速过低（<2km/h），使用最小预瞄距离" << optimal_ld << "m" << std::endl;
        }
        
        return optimal_ld;
    }
    
    /**
     * @brief 计算最优预瞄距离
     * @param vehicle_speed 车辆速度 (km/h)
     * @param current_lat 当前纬度
     * @param current_lon 当前经度
     * @param heading_angle 当前航向角 (度)
     * @param path_points 路径点集合 (纬度, 经度)
     * @param current_path_index 当前路径点索引
     * @return 最优预瞄距离 (m)
     */
    double calculateOptimalLookahead(double vehicle_speed,
                                   double current_lat, double current_lon,
                                   double heading_angle,
                                   const std::vector<std::pair<double, double>>& path_points,
                                   int current_path_index) {
        // std::cout << "\n===== 开始计算最优预瞄距离 =====" << std::endl;
        // std::cout << "输入参数: 速度=" << vehicle_speed << "km/h, 当前位置=(" 
        //           << current_lat << ", " << current_lon << "), 航向角=" << heading_angle 
        //           << "°, 路径点数=" << path_points.size() << ", 当前索引=" << current_path_index << std::endl;
        
        // 1. 计算基础预瞄距离（基于速度的线性关系）
        double speed_ms = vehicle_speed / 3.6;
        double base_lookahead = base_lookahead_gain_ * speed_ms;
        base_lookahead = std::clamp(base_lookahead, min_lookahead_, max_lookahead_);
        // std::cout << "步骤1 - 基础预瞄距离: " << base_lookahead << "m (速度=" << speed_ms << "m/s)" << std::endl;
        
        // 2. 计算路径曲率 - 增强鲁棒性
        double curvature = 0.0;
        bool curvature_valid = false;
        
        // 检查路径点和索引有效性
        if (!path_points.empty() && path_points.size() >= 3) {
            // 尝试计算当前位置的曲率
            curvature = calculatePathCurvature(path_points, current_path_index);
            curvature_valid = true;
            
            // 如果曲率仍为0，尝试计算前方路径的曲率
            if (curvature < 0.001 && current_path_index + 2 < path_points.size()) {
                // std::cout << "当前位置曲率接近0，尝试计算前方路径曲率..." << std::endl;
                double forward_curvature = calculatePathCurvature(path_points, current_path_index + 1);
                
                // 如果前方曲率有效，使用前方曲率
                if (forward_curvature > 0.001) {
                    // std::cout << "使用前方路径曲率: " << forward_curvature << std::endl;
                    curvature = forward_curvature;
                }
            }
        } else {
            // std::cout << "警告: 路径点数据无效，无法计算曲率" << std::endl;
        }
        
        // std::cout << "步骤2 - 路径曲率计算: " << (curvature_valid ? "成功" : "失败") 
                //   << ", 曲率值=" << curvature << std::endl;
        
        // 3. 计算横向误差（如果有目标点的话）
        double lateral_error = 0.0;
        bool lateral_error_valid = false;
        
        if (!path_points.empty() && current_path_index < path_points.size()) {
            auto target_point = path_points[current_path_index];
            lateral_error = this->calculateLateralError(current_lat, current_lon,
                                                target_point.first, target_point.second,
                                                heading_angle);
            lateral_error_valid = true;
        } else if (!path_points.empty()) {
            // 如果索引超出范围但路径点存在，使用最后一个点
            auto target_point = path_points.back();
            lateral_error = this->calculateLateralError(current_lat, current_lon,
                                                target_point.first, target_point.second,
                                                heading_angle);
            lateral_error_valid = true;
            std::cout << "警告: 当前索引超出范围，使用最后一个路径点计算横向误差" << std::endl;
        } else {
            std::cout << "警告: 无法计算横向误差，路径点数据无效" << std::endl;
        }
        
        // std::cout << "步骤3 - 横向误差计算: " << (lateral_error_valid ? "成功" : "失败") 
                //   << ", 误差值=" << lateral_error << "m" << std::endl;
        
        // 4. 基于曲率的自适应调整 - 增强鲁棒性
        double curvature_factor = 1.0;
        
        // 即使曲率为0，也应用最小的调整因子，避免预瞄距离过大
        if (curvature > 0.106) { // 高曲率路段
            // 修改高曲率路段的曲率因子计算公式，增大系数以提高转向响应性
            curvature_factor = 1.0 / (1.0 + curvature * 180.0);
            // 降低最小限制值，使高曲率路段预瞄距离更短，提高转向灵敏度
            curvature_factor = std::clamp(curvature_factor, 0.06, 1.0);
            std::cout << "高曲率路段: 应用曲率因子=" << curvature_factor << std::endl;
        } else if (curvature > 0.001) { // 低曲率路段
            curvature_factor = 1.0 / (1.0 + curvature * 40.0);
            curvature_factor = std::clamp(curvature_factor, 0.5, 1.0);
            std::cout << "低曲率路段: 应用曲率因子=" << curvature_factor << std::endl;
        } else { // 几乎直线路段
            // 即使在直线上，也略微减小预瞄距离以提高响应性
            curvature_factor = 0.95;
            std::cout << "直线路段: 应用默认曲率因子=" << curvature_factor << std::endl;
        }
        
        double adaptive_lookahead = base_lookahead * curvature_factor;
        // std::cout << "步骤4 - 曲率自适应调整: 基础Ld=" << base_lookahead 
                //   << "m × 因子=" << curvature_factor 
                //   << " = 调整后Ld=" << adaptive_lookahead << "m" << std::endl;
        
        // 5. 二次规划优化
        double optimal_lookahead = quadraticOptimization(adaptive_lookahead, curvature, 
                                                        lateral_error, vehicle_speed);
        
        // std::cout << "步骤5 - 二次规划优化: 输入Ld=" << adaptive_lookahead 
                //   << "m, 优化后Ld=" << optimal_lookahead << "m" << std::endl;
        
        // 6. 更新历史数据
        updateHistory(optimal_lookahead, lateral_error);
        
        // 7. 平滑处理，避免预瞄距离突变
        static double prev_lookahead = optimal_lookahead;
        double smoothed_lookahead = 0.87 * optimal_lookahead + 0.13 * prev_lookahead;
        prev_lookahead = smoothed_lookahead;
        
        // std::cout << "步骤6 - 平滑处理: 原始Ld=" << optimal_lookahead 
                //   << "m, 平滑后Ld=" << smoothed_lookahead << "m" << std::endl;
        
        // 确保平滑后的预瞄距离不接近0
        if (smoothed_lookahead < 0.1 || vehicle_speed < 2.0) {
            smoothed_lookahead = std::max(min_lookahead_, 2.0);
            std::cout << "警告: 平滑后的预瞄距离接近0或车速过低（<2km/h），使用最小预瞄距离" << smoothed_lookahead << "m" << std::endl;
        }
        
        // 8. 输出最终结果
        std::cout << "===== 预瞄距离计算结果 =====" << std::endl;
        std::cout << "速度=" << vehicle_speed << "km/h, "
                  << "基础Ld=" << base_lookahead << "m, "
                  << "曲率=" << curvature << ", "
                  << "横向误差=" << lateral_error << "m, "
                  << "最终Ld=" << smoothed_lookahead << "m" << std::endl;
        std::cout << "============================\n" << std::endl;
        
        // 修正：确保最终返回的预瞄距离不小于min_lookahead_
        smoothed_lookahead = std::clamp(smoothed_lookahead, min_lookahead_, max_lookahead_);
        return smoothed_lookahead;
    }
    
    /**
     * @brief 更新历史数据
     * @param lookahead 当前预瞄距离
     * @param error 当前跟踪误差
     */
    void updateHistory(double lookahead, double error) {
        prev_lookaheads_.push_back(lookahead);
        prev_errors_.push_back(error);
        
        if (prev_lookaheads_.size() > history_size_) {
            prev_lookaheads_.erase(prev_lookaheads_.begin());
        }
        if (prev_errors_.size() > history_size_) {
            prev_errors_.erase(prev_errors_.begin());
        }
    }
    
    /**
     * @brief 设置算法参数
     */
    void setParameters(double base_gain, double min_ld, double max_ld,
                      double track_weight, double stab_weight, double smooth_weight) {
        base_lookahead_gain_ = base_gain;
        min_lookahead_ = min_ld;
        max_lookahead_ = max_ld;
        tracking_weight_ = track_weight;
        stability_weight_ = stab_weight;
        smoothness_weight_ = smooth_weight;
    }
    
    /**
     * @brief 获取性能统计信息
     */
    void getPerformanceStats(double& avg_error, double& error_variance) {
        if (prev_errors_.empty()) {
            avg_error = 0.0;
            error_variance = 0.0;
            return;
        }
        
        // 计算平均误差
        avg_error = 0.0;
        for (double error : prev_errors_) {
            avg_error += fabs(error);
        }
        avg_error /= prev_errors_.size();
        
        // 计算误差方差
        error_variance = 0.0;
        for (double error : prev_errors_) {
            error_variance += (fabs(error) - avg_error) * (fabs(error) - avg_error);
        }
        error_variance /= prev_errors_.size();
    }
};

// 全局优化器实例
OptimalLookaheadCalculator g_lookahead_optimizer;

/**
 * @brief 优化版本的calc_Lf函数
 * @details 替换原有的简单线性计算，使用二次规划优化方法
 * @param vehicle_speed 车辆速度 (km/h)
 * @param current_lat 当前纬度
 * @param current_lon 当前经度
 * @param heading_angle 当前航向角 (度)
 * @param path_points 路径点集合
 * @param current_path_index 当前路径点索引
 * @return 优化后的预瞄距离 (m)
 */
double calc_Lf_optimized(double vehicle_speed,
                        double current_lat, double current_lon,
                        double heading_angle,
                        const std::vector<std::pair<double, double>>& path_points,
                        int current_path_index) {
    return g_lookahead_optimizer.calculateOptimalLookahead(
        vehicle_speed, current_lat, current_lon, heading_angle,
        path_points, current_path_index
    );
}

/**
 * @brief 简化版本的优化calc_Lf函数（兼容原有接口）
 * @param vehicle_speed 车辆速度 (km/h)
 * @return 优化后的预瞄距离 (m)
 */
double calc_Lf_optimized_simple(double vehicle_speed) {
    // 使用默认参数进行简化计算
    std::vector<std::pair<double, double>> empty_path;
    return g_lookahead_optimizer.calculateOptimalLookahead(
        vehicle_speed, 0.0, 0.0, 0.0, empty_path, 0
    );
}

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
void setLookaheadOptimizerParams(double wheelbase, double max_steering_angle,
                                double base_gain, double min_ld, double max_ld,
                                double tracking_weight = 3.5, 
                                double stability_weight = 0.8, 
                                double smoothness_weight = 0.7) {
    std::cout << "\n===== 设置预瞄距离优化器参数 =====" << std::endl;
    std::cout << "车辆参数: 轴距=" << wheelbase << "m, 最大转向角=" << max_steering_angle << "度" << std::endl;
    std::cout << "预瞄距离参数: 基础增益=" << base_gain << ", 最小Ld=" << min_ld << "m, 最大Ld=" << max_ld << "m" << std::endl;
    std::cout << "优化权重: 跟踪精度=" << tracking_weight << ", 稳定性=" << stability_weight << ", 平滑性=" << smoothness_weight << std::endl;
    
    // 设置优化器参数
    g_lookahead_optimizer = OptimalLookaheadCalculator(wheelbase, max_steering_angle);
    g_lookahead_optimizer.setParameters(base_gain, min_ld, max_ld, 
                                      tracking_weight, stability_weight, smoothness_weight);
    
    std::cout << "预瞄距离优化器参数设置完成" << std::endl;
    std::cout << "注意: 如果曲率计算为0，优化器将尝试使用前方路径曲率或默认曲率因子" << std::endl;
    std::cout << "============================\n" << std::endl;
}