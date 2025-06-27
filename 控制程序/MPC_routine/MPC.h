/**
 * @file MPC.h
 * @brief 模型预测控制(MPC)算法头文件
 * @version 1.0
 * @date 2023-05-10
 */

#ifndef MPC_H
#define MPC_H

#include <vector>
#include <Eigen/Dense>

using Eigen::VectorXd;

/**
 * @brief 计算两个角度之间的差值
 * @param angle1 第一个角度(度)
 * @param angle2 第二个角度(度)
 * @return 角度差值(度)
 */
double calculate_delta_angle(double angle1, double angle2);

/**
 * @brief 计算两点之间的方位角
 * @param lat1 起点纬度
 * @param lon1 起点经度
 * @param lat2 终点纬度
 * @param lon2 终点经度
 * @return 方位角(度)
 */
double Azimuth_withGeo(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief 根据车速计算预瞄距离
 * @param v 车速(m/s)
 * @return 预瞄距离(m)
 */
double calc_Lf(double v);

/**
 * @brief 计算两点之间的距离
 * @param lat1 起点纬度
 * @param lon1 起点经度
 * @param lat2 终点纬度
 * @param lon2 终点经度
 * @return 距离(m)
 */
double Distance_withGeo(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief 查找最近的路径点
 * @param x 当前纬度
 * @param y 当前经度
 * @param maps_x 路径点纬度数组
 * @param maps_y 路径点经度数组
 * @return 最近点索引
 */
int findWayPoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y);

/**
 * @brief 计算目标路径点索引
 * @param lat 当前纬度
 * @param lon 当前经度
 * @param Ld 预瞄距离
 * @param cx 路径点纬度数组
 * @param cy 路径点经度数组
 * @return 目标点索引
 */
int calc_target_index(double lat, double lon, double Ld, std::vector<double> cx, std::vector<double> cy);

/**
 * @brief 将全局坐标转换为局部坐标
 * @param ref_lat 参考点纬度
 * @param ref_lon 参考点经度
 * @param ref_heading 参考点航向角(度)
 * @param point_lat 目标点纬度
 * @param point_lon 目标点经度
 * @param x 输出局部x坐标
 * @param y 输出局部y坐标
 */
void global_to_local(double ref_lat, double ref_lon, double ref_heading, 
                    double point_lat, double point_lon, double& x, double& y);

/**
 * @brief 多项式拟合
 * @param xvals x坐标数组
 * @param yvals y坐标数组
 * @param order 多项式阶数
 * @return 多项式系数
 */
VectorXd polyfit(const std::vector<double>& xvals, const std::vector<double>& yvals, int order);

/**
 * @brief 求解MPC优化问题
 * @param state 当前状态向量 [x, y, psi, v, cte, epsi]
 * @param coeffs 路径多项式系数
 * @param wheelbase 车辆轴距
 * @return 最优控制输入和预测轨迹
 */
std::vector<double> solve_mpc(VectorXd state, VectorXd coeffs, double wheelbase);

/**
 * @brief MPC版本的GPS2Steer函数
 * @param vehicle_speed 车速(km/h)
 * @param latitude 当前纬度
 * @param longitude 当前经度
 * @param heading_angle 当前航向角(度)
 * @param map_latitudes 路径点纬度数组
 * @param map_longitudes 路径点经度数组
 * @param wheelbase 车辆轴距(m)
 * @param target_idx 目标路径点索引
 * @return 最优转向角(度)
 */
double GPS2Steer_MPC(double vehicle_speed, double latitude, double longitude, double heading_angle,
                   const std::vector<double>& map_latitudes, const std::vector<double>& map_longitudes, 
                   double wheelbase, const int& target_idx);

#endif // MPC_H