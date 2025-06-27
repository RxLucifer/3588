/**
 * @file MPC_示例.cpp
 * @brief MPC算法使用示例
 * @version 1.0
 * @date 2023-05-10
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>  // 添加线程头文件
#include <algorithm> // 添加algorithm头文件，用于std::remove
#include "MPC.h"

// 全局变量
std::vector<double> map_latitude_v;  // 地图纬度点
std::vector<double> map_longitude_v; // 地图经度点
std::vector<double> map_heading_v;   // 地图航向角点
std::vector<int> map_status_v;       // 地图点状态
int _target_ind = 0;                 // 当前目标点索引

/**
 * @brief 从文件加载地图数据
 * @param filename 地图文件名
 * @return 是否加载成功
 */
bool load_map_data(const std::string& filename) {
    std::ifstream map_file(filename);
    if (!map_file.is_open()) {
        std::cerr << "无法打开地图文件: " << filename << std::endl;
        return false;
    }
    
    std::string line;
    
    // 清空现有数据
    map_latitude_v.clear();
    map_longitude_v.clear();
    map_heading_v.clear();
    map_status_v.clear();
    
    // 读取地图点
    while (std::getline(map_file, line)) {
        // 解析格式: Latitude: 39.20777878, Longitude: 117.22783491, Heading: -107.09000000, Status: 2
        double lat = 0.0, lon = 0.0, heading = 0.0;
        int status = 0;
        bool parsed = false;
        
        // 查找并提取纬度
        size_t lat_pos = line.find("Latitude:");
        size_t lon_pos = line.find("Longitude:");
        size_t heading_pos = line.find("Heading:");
        size_t status_pos = line.find("Status:");
        
        if (lat_pos != std::string::npos && lon_pos != std::string::npos && 
            heading_pos != std::string::npos && status_pos != std::string::npos) {
            
            // 提取纬度值
            std::string lat_str = line.substr(lat_pos + 9, lon_pos - lat_pos - 10);
            lat_str.erase(std::remove(lat_str.begin(), lat_str.end(), ','), lat_str.end());
            lat_str.erase(std::remove(lat_str.begin(), lat_str.end(), ' '), lat_str.end());
            
            // 提取经度值
            std::string lon_str = line.substr(lon_pos + 10, heading_pos - lon_pos - 11);
            lon_str.erase(std::remove(lon_str.begin(), lon_str.end(), ','), lon_str.end());
            lon_str.erase(std::remove(lon_str.begin(), lon_str.end(), ' '), lon_str.end());
            
            // 提取航向角值
            std::string heading_str = line.substr(heading_pos + 8, status_pos - heading_pos - 9);
            heading_str.erase(std::remove(heading_str.begin(), heading_str.end(), ','), heading_str.end());
            heading_str.erase(std::remove(heading_str.begin(), heading_str.end(), ' '), heading_str.end());
            
            // 提取状态值
            std::string status_str = line.substr(status_pos + 7);
            status_str.erase(std::remove(status_str.begin(), status_str.end(), ' '), status_str.end());
            
            try {
                lat = std::stod(lat_str);
                lon = std::stod(lon_str);
                heading = std::stod(heading_str);
                status = std::stoi(status_str);
                parsed = true;
            } catch (const std::exception& e) {
                std::cerr << "解析错误: " << e.what() << " 在行: " << line << std::endl;
            }
        }
        
        if (parsed) {
            map_latitude_v.push_back(lat);
            map_longitude_v.push_back(lon);
            map_heading_v.push_back(heading);
            map_status_v.push_back(status);
        }
    }
    
    map_file.close();
    
    if (map_latitude_v.empty()) {
        std::cerr << "地图文件为空或格式不正确" << std::endl;
        return false;
    }
    
    std::cout << "成功加载地图数据，共 " << map_latitude_v.size() << " 个点" << std::endl;
    return true;
}

/**
 * @brief 模拟车辆运动
 * @param current_lat 当前纬度
 * @param current_lon 当前经度
 * @param heading 当前航向角(度)
 * @param speed 当前速度(km/h)
 * @param steer_angle 转向角(度)
 * @param dt 时间步长(秒)
 * @param wheelbase 轴距(m)
 * @param new_lat 新纬度(输出)
 * @param new_lon 新经度(输出)
 * @param new_heading 新航向角(输出)
 */
void simulate_vehicle_motion(double current_lat, double current_lon, double heading, 
                            double speed, double steer_angle, double dt, double wheelbase,
                            double& new_lat, double& new_lon, double& new_heading) {
    // 转换速度单位 km/h -> m/s
    double speed_ms = speed / 3.6;
    
    // 计算航向角变化率 (rad/s)
    double heading_rate = speed_ms * tan(steer_angle * M_PI / 180.0) / wheelbase;
    
    // 更新航向角 (度)
    new_heading = heading + heading_rate * dt * 180.0 / M_PI;
    
    // 规范化航向角到 [0, 360)
    while (new_heading >= 360.0) new_heading -= 360.0;
    while (new_heading < 0.0) new_heading += 360.0;
    
    // 计算位移
    double distance = speed_ms * dt;
    
    // 计算新位置 (简化模型，不考虑地球曲率)
    double heading_rad = heading * M_PI / 180.0;
    double lat_change = distance * cos(heading_rad) / 111320.0; // 纬度1度约等于111.32km
    double lon_change = distance * sin(heading_rad) / (111320.0 * cos(current_lat * M_PI / 180.0)); // 经度1度随纬度变化
    
    new_lat = current_lat + lat_change;
    new_lon = current_lon + lon_change;
}

/**
 * @brief 记录控制数据到CSV文件
 * @param filename 文件名
 * @param timestamp 时间戳
 * @param lat 当前纬度
 * @param lon 当前经度
 * @param heading 当前航向角
 * @param speed 当前速度
 * @param target_lat 目标点纬度
 * @param target_lon 目标点经度
 * @param mpc_steer MPC转向角
 * @param pp_steer 纯跟踪转向角
 * @param selected_steer 选择的转向角
 */
void log_control_data(const std::string& filename, double timestamp,
                     double lat, double lon, double heading, double speed,
                     double target_lat, double target_lon,
                     double mpc_steer, double pp_steer, double selected_steer) {
    static bool file_exists = false;
    std::ofstream log_file;
    
    if (!file_exists) {
        log_file.open(filename);
        log_file << "Timestamp,Latitude,Longitude,Heading,Speed,TargetLat,TargetLon,"
                 << "MPCSteer,PurePursuitSteer,SelectedSteer" << std::endl;
        file_exists = true;
    } else {
        log_file.open(filename, std::ios::app);
    }
    
    if (log_file.is_open()) {
        log_file << timestamp << ","
                << lat << ","
                << lon << ","
                << heading << ","
                << speed << ","
                << target_lat << ","
                << target_lon << ","
                << mpc_steer << ","
                << pp_steer << ","
                << selected_steer << std::endl;
        log_file.close();
    }
}

/**
 * @brief 纯跟踪算法计算转向角
 * @param speed 车速(km/h)
 * @param current_lat 当前纬度
 * @param current_lon 当前经度
 * @param heading 当前航向角(度)
 * @param target_lat 目标点纬度
 * @param target_lon 目标点经度
 * @param wheelbase 轴距(m)
 * @param Ld 预瞄距离(m)
 * @param alpha 输出航向偏差角(度)
 * @return 转向角(度)
 */
double GPS2Steer(double speed, double current_lat, double current_lon, double heading,
               double target_lat, double target_lon, double wheelbase, double Ld, double& alpha) {
    // 计算当前位置到目标点的方位角
    double target_angle = Azimuth_withGeo(current_lat, current_lon, target_lat, target_lon);
    
    // 计算航向偏差角
    alpha = calculate_delta_angle(target_angle, heading);
    
    // 计算转向角
    return atan2(2.0 * wheelbase * sin(alpha * M_PI / 180.0) / Ld, 1.0) * 180.0 / M_PI;
}

int main() {
    // 加载地图数据
    if (!load_map_data("/home/ztl/gps_path/path.txt")) {
        return 1;
    }
    
    // 车辆参数
    double wheelbase = 0.9;  // 轴距(m)
    double max_steer = 27.0; // 最大转向角(度)
    
    // 初始状态
    double current_lat = map_latitude_v[0];
    double current_lon = map_longitude_v[0];
    double heading = 0.0;    // 初始航向角(度)
    double speed = 10.0;     // 初始速度(km/h)
    
    // 控制参数
    double dt = 0.1;         // 控制周期(秒)
    double sim_time = 60.0;  // 模拟时间(秒)
    bool use_mpc = true;     // 是否使用MPC控制
    
    // 日志文件
    std::string log_filename = "control_comparison.csv";
    
    // 模拟循环
    double t = 0.0;
    while (t < sim_time) {
        // 计算预瞄距离
        double Ld = calc_Lf(speed / 3.6);
        
        // 获取目标点
        int target_idx = calc_target_index(current_lat, current_lon, Ld, map_latitude_v, map_longitude_v);
        if (_target_ind >= target_idx) {
            target_idx = _target_ind;
        }
        _target_ind = target_idx;
        
        // 检查是否到达终点
        if (target_idx >= map_latitude_v.size() - 1) {
            std::cout << "到达终点!" << std::endl;
            break;
        }
        
        // 计算MPC转向角
        double mpc_steer = GPS2Steer_MPC(speed, current_lat, current_lon, heading,
                                      map_latitude_v, map_longitude_v, wheelbase, target_idx);
        
        // 计算纯跟踪转向角
        double alpha, pp_steer;
        pp_steer = GPS2Steer(speed, current_lat, current_lon, heading,
                           map_latitude_v[target_idx], map_longitude_v[target_idx], wheelbase, Ld, alpha);
        
        // 选择使用的转向角
        double selected_steer = use_mpc ? mpc_steer : pp_steer;
        
        // 记录数据
        log_control_data(log_filename, t, current_lat, current_lon, heading, speed,
                        map_latitude_v[target_idx], map_longitude_v[target_idx],
                        mpc_steer, pp_steer, selected_steer);
        
        // 模拟车辆运动
        double new_lat, new_lon, new_heading;
        simulate_vehicle_motion(current_lat, current_lon, heading, speed, selected_steer, dt, wheelbase,
                               new_lat, new_lon, new_heading);
        
        // 更新状态
        current_lat = new_lat;
        current_lon = new_lon;
        heading = new_heading;
        
        // 输出当前状态
        std::cout << "时间: " << t << "s, 位置: (" << current_lat << ", " << current_lon
                  << "), 航向: " << heading << "°, 速度: " << speed << "km/h" << std::endl;
        std::cout << "目标点: (" << map_latitude_v[target_idx] << ", " << map_longitude_v[target_idx]
                  << "), MPC转向: " << mpc_steer << "°, 纯跟踪转向: " << pp_steer << "°" << std::endl;
        std::cout << "-------------------------------------" << std::endl;
        
        // 更新时间
        t += dt;
        
        // 模拟延时
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
    
    std::cout << "模拟完成，数据已保存到 " << log_filename << std::endl;
    
    return 0;
}