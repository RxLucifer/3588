/****************************************************************************/
/*  项目名称：中邮快递车自动驾驶控制系统 - 预瞄距离计算器测试程序              */
/*  文件名称：test_lookahead.cpp                                            */
/*  创建时间：2025-06-05                                                     */
/*  开发人员：RXL                                                          */
/*  项目描述：测试优化预瞄距离计算器的功能和性能                            */
/****************************************************************************/

#include "optimal_lookahead_calculator.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <fstream>
#include <string>
#include <regex>
#include <GeographicLib/Geodesic.hpp>

/**
 * @brief 生成测试路径点
 * @param center_lat 中心纬度
 * @param center_lon 中心经度
 * @param radius 半径 (m)
 * @param num_points 点数
 * @return 路径点集合
 */
std::vector<std::pair<double, double>> generateTestPath(double center_lat, double center_lon, 
                                                       double radius, int num_points) {
    std::vector<std::pair<double, double>> path;
    
    for (int i = 0; i < num_points; ++i) {
        double angle = 2.0 * M_PI * i / num_points;
        
        // 简化的坐标转换（适用于小范围）
        double lat_offset = radius * cos(angle) / 111320.0; // 1度纬度约111320米
        double lon_offset = radius * sin(angle) / (111320.0 * cos(center_lat * M_PI / 180.0));
        
        double lat = center_lat + lat_offset;
        double lon = center_lon + lon_offset;
        
        path.push_back(std::make_pair(lat, lon));
    }
    
    return path;
}

/**
 * @brief 测试基本功能
 */
void testBasicFunctionality() {
    std::cout << "\n=== 基本功能测试 ===" << std::endl;
    
    // 初始化优化器
    setLookaheadOptimizerParams(2.0, 33.0, 2.5, 3.5, 25.0);
    
    // 测试简化版本
    std::cout << "\n简化版本测试：" << std::endl;
    std::vector<double> test_speeds = {0, 10, 20, 30, 40, 50, 60};
    
    for (double speed : test_speeds) {
        double ld = calc_Lf_optimized_simple(speed);
        std::cout << "速度: " << std::setw(3) << speed << " km/h, 预瞄距离: " 
                  << std::setw(6) << std::fixed << std::setprecision(2) << ld << " m" << std::endl;
    }
}

/**
 * @brief 测试完整功能
 */
void testFullFunctionality() {
    std::cout << "\n=== 完整功能测试 ===" << std::endl;
    
    // 生成测试路径（圆形路径）
    double center_lat = 39.9042;  // 北京天安门广场纬度
    double center_lon = 116.4074; // 北京天安门广场经度
    auto path = generateTestPath(center_lat, center_lon, 100.0, 20); // 100米半径，20个点
    
    std::cout << "\n生成测试路径，共 " << path.size() << " 个点" << std::endl;
    
    // 模拟车辆在路径上行驶
    std::cout << "\n模拟车辆行驶测试：" << std::endl;
    std::cout << "索引  速度(km/h)  纬度        经度        航向角(度)  预瞄距离(m)" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    
    for (int i = 0; i < std::min(10, (int)path.size()); ++i) {
        double speed = 20.0 + i * 3.0; // 速度从20km/h递增
        double heading = i * 18.0;     // 航向角变化
        
        double ld = calc_Lf_optimized(
            speed,
            path[i].first,
            path[i].second,
            heading,
            path,
            i
        );
        
        std::cout << std::setw(4) << i 
                  << std::setw(10) << std::fixed << std::setprecision(1) << speed
                  << std::setw(12) << std::fixed << std::setprecision(6) << path[i].first
                  << std::setw(12) << std::fixed << std::setprecision(6) << path[i].second
                  << std::setw(11) << std::fixed << std::setprecision(1) << heading
                  << std::setw(12) << std::fixed << std::setprecision(2) << ld
                  << std::endl;
    }
}

/**
 * @brief 测试不同场景
 */
void testDifferentScenarios() {
    std::cout << "\n=== 不同场景测试 ===" << std::endl;
    
    // 场景1：直线路径
    std::cout << "\n场景1：直线路径" << std::endl;
    std::vector<std::pair<double, double>> straight_path;
    for (int i = 0; i < 10; ++i) {
        straight_path.push_back(std::make_pair(39.9042 + i * 0.001, 116.4074));
    }
    
    double ld1 = calc_Lf_optimized(30.0, 39.9042, 116.4074, 0.0, straight_path, 2);
    std::cout << "直线路径预瞄距离: " << ld1 << " m" << std::endl;
    
    // 场景2：急转弯路径
    std::cout << "\n场景2：急转弯路径" << std::endl;
    auto sharp_turn_path = generateTestPath(39.9042, 116.4074, 20.0, 8); // 小半径急转弯
    
    double ld2 = calc_Lf_optimized(30.0, 39.9042, 116.4074, 45.0, sharp_turn_path, 2);
    std::cout << "急转弯路径预瞄距离: " << ld2 << " m" << std::endl;
    
    // 场景3：高速直线
    std::cout << "\n场景3：高速直线" << std::endl;
    double ld3 = calc_Lf_optimized(80.0, 39.9042, 116.4074, 0.0, straight_path, 2);
    std::cout << "高速直线预瞄距离: " << ld3 << " m" << std::endl;
    
    // 场景4：低速转弯
    std::cout << "\n场景4：低速转弯" << std::endl;
    double ld4 = calc_Lf_optimized(10.0, 39.9042, 116.4074, 30.0, sharp_turn_path, 2);
    std::cout << "低速转弯预瞄距离: " << ld4 << " m" << std::endl;
}

/**
 * @brief 性能测试
 */
void testPerformance() {
    std::cout << "\n=== 性能测试 ===" << std::endl;
    
    auto path = generateTestPath(39.9042, 116.4074, 100.0, 50);
    
    const int test_iterations = 1000;
    
    // 测试优化算法性能
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < test_iterations; ++i) {
        calc_Lf_optimized(30.0, 39.9042, 116.4074, 0.0, path, i % path.size());
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    std::cout << "优化算法执行 " << test_iterations << " 次耗时: " 
              << duration.count() << " 微秒" << std::endl;
    std::cout << "平均每次计算耗时: " 
              << (double)duration.count() / test_iterations << " 微秒" << std::endl;
    
    // 测试简化算法性能
    start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < test_iterations; ++i) {
        calc_Lf_optimized_simple(30.0);
    }
    
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    std::cout << "简化算法执行 " << test_iterations << " 次耗时: " 
              << duration.count() << " 微秒" << std::endl;
    std::cout << "平均每次计算耗时: " 
              << (double)duration.count() / test_iterations << " 微秒" << std::endl;
}

/**
 * @brief 对比测试（与原有算法对比）
 */
void testComparison() {
    std::cout << "\n=== 算法对比测试 ===" << std::endl;
    
    // 原有算法实现
    auto calc_Lf_original = [](double v) -> double {
        double k = 2.9, b = 0;
        double Ld = k * v + b;
        if (Ld < 3.5) {
            Ld = 3.5;
        }
        return Ld;
    };
    
    std::cout << "\n速度对比测试：" << std::endl;
    std::cout << "速度(km/h)  原算法(m)  优化算法(m)  差值(m)   改进率(%)" << std::endl;
    std::cout << "---------------------------------------------------" << std::endl;
    
    std::vector<double> test_speeds = {5, 10, 15, 20, 30, 40, 50, 60};
    
    for (double speed : test_speeds) {
        double original_ld = calc_Lf_original(speed / 3.6); // 原算法使用m/s
        double optimized_ld = calc_Lf_optimized_simple(speed);
        double diff = optimized_ld - original_ld;
        double improvement = (diff / original_ld) * 100.0;
        
        std::cout << std::setw(9) << std::fixed << std::setprecision(1) << speed
                  << std::setw(11) << std::fixed << std::setprecision(2) << original_ld
                  << std::setw(12) << std::fixed << std::setprecision(2) << optimized_ld
                  << std::setw(9) << std::fixed << std::setprecision(2) << diff
                  << std::setw(11) << std::fixed << std::setprecision(1) << improvement
                  << std::endl;
    }
}

/**
 * @brief 从文件读取GPS路径点
 * @param filepath 文件路径
 * @return 路径点集合（纬度、经度、航向角）
 */
std::vector<std::tuple<double, double, double>> readPathFromFile(const std::string& filepath) {
    std::vector<std::tuple<double, double, double>> path_points;
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filepath << std::endl;
        return path_points;
    }
    
    std::string line;
    std::regex pattern("Latitude: ([\\-0-9\\.]+), Longitude: ([\\-0-9\\.]+), Heading: ([\\-0-9\\.]+), Status: ([0-9]+)");
    
    while (std::getline(file, line)) {
        std::smatch matches;
        if (std::regex_search(line, matches, pattern) && matches.size() > 4) {
            double lat = std::stod(matches[1].str());
            double lon = std::stod(matches[2].str());
            double heading = std::stod(matches[3].str());
            int status = std::stoi(matches[4].str());
            
            // 只添加状态为2的点（假设2表示有效点）
            if (status == 2) {
                path_points.push_back(std::make_tuple(lat, lon, heading));
            }
        }
    }
    
    file.close();
    std::cout << "从文件读取了 " << path_points.size() << " 个有效路径点" << std::endl;
    return path_points;
}

/**
 * @brief 计算路径曲率
 * @param path_points 路径点集合
 * @return 曲率数组
 */
std::vector<double> calculatePathCurvatures(const std::vector<std::tuple<double, double, double>>& path_points) {
    std::vector<double> curvatures;
    if (path_points.size() < 3) {
        return curvatures;
    }
    
    const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
    
    for (size_t i = 1; i < path_points.size() - 1; ++i) {
        double lat1 = std::get<0>(path_points[i-1]);
        double lon1 = std::get<1>(path_points[i-1]);
        double lat2 = std::get<0>(path_points[i]);
        double lon2 = std::get<1>(path_points[i]);
        double lat3 = std::get<0>(path_points[i+1]);
        double lon3 = std::get<1>(path_points[i+1]);
        
        double d12, d23, d13;
        geod.Inverse(lat1, lon1, lat2, lon2, d12);
        geod.Inverse(lat2, lon2, lat3, lon3, d23);
        geod.Inverse(lat1, lon1, lat3, lon3, d13);
        
        if (d12 < 0.1 || d23 < 0.1 || d13 < 0.1) {
            curvatures.push_back(0.0);
            continue;
        }
        
        // 使用海伦公式计算三角形面积
        double s = (d12 + d23 + d13) / 2.0;
        double area = sqrt(s * (s - d12) * (s - d23) * (s - d13));
        
        // 曲率 = 4 * 面积 / (三边长乘积)
        double curvature = 4.0 * area / (d12 * d23 * d13);
        curvatures.push_back(curvature);
    }
    
    // 为首尾点添加曲率值
    if (!curvatures.empty()) {
        curvatures.insert(curvatures.begin(), curvatures.front());
        curvatures.push_back(curvatures.back());
    }
    
    return curvatures;
}

/**
 * @brief 测试低速情况下的预瞄距离
 */
void testLowSpeedLookahead() {
    std::cout << "\n=== 低速情况预瞄距离测试 (1-10km/h) ===" << std::endl;
    
    // 初始化优化器参数
    setLookaheadOptimizerParams(2.0, 33.0, 2.5, 3.5, 25.0);
    
    // 从文件读取路径点
    std::string filepath = "/home/ztl/gps_path/path.txt";
    auto path_points_tuple = readPathFromFile(filepath);
    
    if (path_points_tuple.empty()) {
        std::cout << "无法读取路径点，使用生成的测试路径代替" << std::endl;
        double center_lat = 39.9042;
        double center_lon = 116.4074;
        auto path_points_pair = generateTestPath(center_lat, center_lon, 100.0, 20);
        
        // 转换为需要的格式
        for (const auto& point : path_points_pair) {
            path_points_tuple.push_back(std::make_tuple(point.first, point.second, 0.0));
        }
    }
    
    // 计算路径曲率
    auto curvatures = calculatePathCurvatures(path_points_tuple);
    
    // 转换为pair格式用于计算
    std::vector<std::pair<double, double>> path_points_pair;
    for (const auto& point : path_points_tuple) {
        path_points_pair.push_back(std::make_pair(std::get<0>(point), std::get<1>(point)));
    }
    
    // 测试1-10km/h的预瞄距离
    std::cout << "\n低速情况下的预瞄距离计算结果：" << std::endl;
    std::cout << "速度(km/h)  预瞄距离(m)  曲率      航向角(度)" << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    
    for (double speed = 1.0; speed <= 10.0; speed += 1.0) {
        // 选择路径中间点作为当前位置
        size_t current_index = path_points_tuple.size() / 2;
        if (current_index >= path_points_tuple.size()) {
            current_index = 0;
        }
        
        double current_lat = std::get<0>(path_points_tuple[current_index]);
        double current_lon = std::get<1>(path_points_tuple[current_index]);
        double heading = std::get<2>(path_points_tuple[current_index]);
        double curvature = (current_index < curvatures.size()) ? curvatures[current_index] : 0.0;
        
        double ld = calc_Lf_optimized(
            speed,
            current_lat,
            current_lon,
            heading,
            path_points_pair,
            current_index
        );
        
        std::cout << std::setw(9) << std::fixed << std::setprecision(1) << speed
                  << std::setw(12) << std::fixed << std::setprecision(2) << ld
                  << std::setw(10) << std::fixed << std::setprecision(6) << curvature
                  << std::setw(12) << std::fixed << std::setprecision(1) << heading
                  << std::endl;
    }
    
    // 测试简化版本
    std::cout << "\n简化版本在低速情况下的预瞄距离：" << std::endl;
    std::cout << "速度(km/h)  预瞄距离(m)" << std::endl;
    std::cout << "---------------------" << std::endl;
    
    for (double speed = 1.0; speed <= 10.0; speed += 1.0) {
        double ld = calc_Lf_optimized_simple(speed);
        std::cout << std::setw(9) << std::fixed << std::setprecision(1) << speed
                  << std::setw(12) << std::fixed << std::setprecision(2) << ld
                  << std::endl;
    }
}

/**
 * @brief 主测试函数
 */
int main() {
    std::cout << "优化预瞄距离计算器测试程序" << std::endl;
    std::cout << "============================" << std::endl;
    
    try {
        // 添加低速情况测试作为首要测试
        testLowSpeedLookahead();
        
        testBasicFunctionality();
        testFullFunctionality();
        testDifferentScenarios();
        testPerformance();
        testComparison();
        
        std::cout << "\n=== 测试完成 ===" << std::endl;
        std::cout << "所有测试均已完成，请检查输出结果。" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "测试过程中发生错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

/****************************************************************************/
/*  编译和运行说明：                                                         */
/*                                                                          */
/*  编译命令：                                                               */
/*  g++ -std=c++14 -O2 -o test_lookahead test_lookahead.cpp \              */
/*      optimal_lookahead_calculator.cpp -lGeographic -lm                  */
/*                                                                          */
/*  运行命令：                                                               */
/*  ./test_lookahead                                                       */
/*                                                                          */
/*  预期输出：                                                               */
/*  - 基本功能测试结果                                                       */
/*  - 完整功能测试结果                                                       */
/*  - 不同场景下的预瞄距离计算结果                                           */
/*  - 性能测试数据                                                           */
/*  - 与原算法的对比结果                                                     */
/****************************************************************************/