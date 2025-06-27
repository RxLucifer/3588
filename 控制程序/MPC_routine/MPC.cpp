/**
 * @file MPC.cpp
 * @brief 模型预测控制(MPC)算法实现
 * @version 1.0
 * @date 2023-05-10
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <GeographicLib/Geodesic.hpp>
#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace std;
using CppAD::AD;
using Eigen::VectorXd;

// MPC相关常量和参数定义
size_t N = 8;           // 预测步长
double dt = 0.1;        // 时间步长(秒)
double ref_v = 5.0;     // 参考速度(m/s)

// 状态向量索引
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// MPC优化权重
double w_cte = 1500.0;      // 横向误差权重
double w_epsi = 1500.0;     // 航向误差权重
double w_v = 1.0;           // 速度误差权重
double w_delta = 10.0;      // 转向输入权重
double w_a = 5.0;           // 加速度输入权重
double w_ddelta = 500.0;    // 转向变化率权重
double w_da = 10.0;         // 加速度变化率权重

/**
 * @brief 计算两个角度之间的差值
 * @param angle1 第一个角度(度)
 * @param angle2 第二个角度(度)
 * @return 角度差值(度)
 */
double calculate_delta_angle(double angle1, double angle2)
{
    while (angle1 > 360) { angle1 -= 360; }
    while (angle1 < 0) { angle1 += 360; }
    while (angle2 > 360) { angle2 -= 360; }
    while (angle2 < 0) { angle2 += 360; }
    double delta = angle1 - angle2;
    if (delta > 180)
    {
        delta = delta - 360;
    }
    else if (delta < -180)
    {
        delta = 360 + delta;
    }
    // 增加航向角误差反馈，提升急弯跟踪精度
    delta *= 1.15;
    return delta;
}

/**
 * @brief 计算两点之间的方位角
 * @param lat1 起点纬度
 * @param lon1 起点经度
 * @param lat2 终点纬度
 * @param lon2 终点经度
 * @return 方位角(度)
 */
double Azimuth_withGeo(double lat1, double lon1, double lat2, double lon2)
{
    double s12(0), azi1(0), azi2(0);
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
    geod.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2);
    cout << "geodesic lib:" << azi1 << endl;
    return azi1;
}

/**
 * @brief 根据车速计算预瞄距离
 * @param v 车速(m/s)
 * @return 预瞄距离(m)
 */
double calc_Lf(double v)
{
    double k = 0.25, b = 0.3;
    double Ld = k * v + b;
    if (Ld < 1){
        Ld = 1; 
    }
    return Ld;
}

/**
 * @brief 计算两点之间的距离
 * @param lat1 起点纬度
 * @param lon1 起点经度
 * @param lat2 终点纬度
 * @param lon2 终点经度
 * @return 距离(m)
 */
double Distance_withGeo(double lat1, double lon1, double lat2, double lon2)
{
    double s12(0);
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
    geod.Inverse(lat1, lon1, lat2, lon2, s12);
    return s12;
}

/**
 * @brief 查找最近的路径点
 * @param x 当前纬度
 * @param y 当前经度
 * @param maps_x 路径点纬度数组
 * @param maps_y 路径点经度数组
 * @return 最近点索引
 */
int findWayPoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y)
{
    int x_size = maps_x.size(), y_size = maps_y.size();
    int n = x_size <= y_size ? x_size : y_size;
    std::vector<double> disList(n, 0);
    for( int i = 0; i != n; ++i)
    {
        disList.at(i) = Distance_withGeo(x, y, maps_x.at(i), maps_y.at(i));
    }

    int closest = min_element(disList.begin(),disList.end()) - disList.begin();
    return closest;
}

/**
 * @brief 计算目标路径点索引
 * @param lat 当前纬度
 * @param lon 当前经度
 * @param Ld 预瞄距离
 * @param cx 路径点纬度数组
 * @param cy 路径点经度数组
 * @return 目标点索引
 */
int calc_target_index(double lat, double lon, double Ld, vector<double> cx, vector<double> cy)
{
    int ind = findWayPoint(lat, lon, cx, cy);
    cout << "最近点下标: " << ind << "/" << cx.size() << endl;
    double L = 0;
    while ((Ld > L) && ((ind + 1) < cx.size())) {
        L += Distance_withGeo(cx[ind + 1], cy[ind + 1], cx[ind], cy[ind]);
        ind += 1;
    }
    return ind;
}

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
                    double point_lat, double point_lon, double& x, double& y) {
    // 计算距离和方位角
    double distance = Distance_withGeo(ref_lat, ref_lon, point_lat, point_lon);
    double bearing = Azimuth_withGeo(ref_lat, ref_lon, point_lat, point_lon);
    
    // 将方位角转换为相对于车辆航向的角度
    double relative_angle = (bearing - ref_heading) * M_PI / 180.0;
    
    // 计算局部坐标
    x = distance * cos(relative_angle);
    y = distance * sin(relative_angle);
}

/**
 * @brief 多项式拟合
 * @param xvals x坐标数组
 * @param yvals y坐标数组
 * @param order 多项式阶数
 * @return 多项式系数
 */
VectorXd polyfit(const std::vector<double>& xvals, const std::vector<double>& yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  
  Eigen::MatrixXd A(xvals.size(), order + 1);
  
  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }
  
  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals[j];
    }
  }
  
  auto Q = A.householderQr();
  auto result = Q.solve(Eigen::VectorXd::Map(&yvals[0], yvals.size()));
  
  return result;
}

/**
 * @brief MPC优化问题定义类
 */
class FG_eval {
 public:
  // 多项式系数
  VectorXd coeffs;
  // 车辆轴距
  double wheelbase;
  
  // 构造函数
  FG_eval(VectorXd coeffs, double wheelbase) { 
    this->coeffs = coeffs; 
    this->wheelbase = wheelbase;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  
  // 计算约束和代价函数
  void operator()(ADvector& fg, const ADvector& vars) {
    // 代价函数
    fg[0] = 0;
    
    // 状态误差代价
    for (int t = 0; t < N; t++) {
      fg[0] += w_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += w_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += w_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    // 控制输入代价
    for (int t = 0; t < N - 1; t++) {
      fg[0] += w_delta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += w_a * CppAD::pow(vars[a_start + t], 2);
    }
    
    // 控制输入变化率代价
    for (int t = 0; t < N - 2; t++) {
      fg[0] += w_ddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += w_da * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    
    // 初始约束
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
    // 其余时间步的约束
    for (int t = 1; t < N; t++) {
      // t+1时刻的状态
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      
      // t时刻的状态
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      
      // t时刻的控制输入
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      
      // 计算f(x)和f'(x)
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0);
      
      // 模型约束 - 自行车模型
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / wheelbase * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / wheelbase * dt);
    }
  }
};

/**
 * @brief 求解MPC优化问题
 * @param state 当前状态向量 [x, y, psi, v, cte, epsi]
 * @param coeffs 路径多项式系数
 * @param wheelbase 车辆轴距
 * @return 最优控制输入和预测轨迹
 */
std::vector<double> solve_mpc(VectorXd state, VectorXd coeffs, double wheelbase) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  
  // 设置变量数量
  size_t n_vars = 6 * N + 2 * (N - 1);
  // 设置约束数量
  size_t n_constraints = 6 * N;
  
  // 初始化变量
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  // 设置初始状态
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  
  // 设置变量上下界
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // 设置状态变量的上下界为无穷大
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  
  // 设置转向角的上下界 [-25度, 25度]
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.47; // 约-27度
    vars_upperbound[i] = 0.47;  // 约27度
  }
  
  // 设置加速度的上下界 [-1.0, 1.0]
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  
  // 设置约束上下界
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  // 设置初始状态约束
  constraints_lowerbound[x_start] = x;
  constraints_upperbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_upperbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_upperbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_upperbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_upperbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  constraints_upperbound[epsi_start] = epsi;
  
  // 设置求解器选项
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";
  
  // 定义优化问题
  CppAD::ipopt::solve_result<Dvector> solution;
  
  // 求解优化问题
  FG_eval fg_eval(coeffs, wheelbase);
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
  
  // 检查求解是否成功
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  
  // 如果求解失败，返回默认值
  if (!ok) {
    std::cout << "求解MPC优化问题失败!" << std::endl;
    std::vector<double> default_result;
    default_result.push_back(0.0);  // 默认转向角为0
    default_result.push_back(0.0);  // 默认加速度为0
    
    // 添加一些默认轨迹点
    for (int i = 0; i < N; i++) {
      default_result.push_back(i * v * dt);  // 简单的直线轨迹
      default_result.push_back(0.0);
    }
    
    return default_result;
  }
  
  // 返回控制输入和预测轨迹
  std::vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  
  for (int i = 0; i < N; i++) {
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }
  
  return result;
}

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
                   double wheelbase, const int& target_idx) {
 
    // 获取未来路径点的局部坐标
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    
    // 选择未来的路径点进行拟合 - 减少点数以提高稳定性
    int num_points = std::min(15, (int)(map_latitudes.size() - target_idx));
    
    // 确保至少有3个点用于多项式拟合
    if (num_points < 3) {
        printf("\n警告：可用路径点不足，无法进行MPC控制\n");
        // 返回0度转向角作为安全值
        return 0.0;
    }
    
    // 收集路径点
    for (int i = 0; i < num_points; i++) {
        double x, y;
        global_to_local(latitude, longitude, heading_angle, 
                       map_latitudes[target_idx + i], map_longitudes[target_idx + i], x, y);
        waypoints_x.push_back(x);
        waypoints_y.push_back(y);
    }
    
    // 拟合多项式 - 使用2阶多项式以减少过拟合
    VectorXd coeffs = polyfit(waypoints_x, waypoints_y, 2);
    
    // 计算当前横向误差和航向误差
    double cte = coeffs[0];  // f(0) - 0，当前位置的横向误差
    double epsi = -atan(coeffs[1]);  // -arctan(f'(0))，当前位置的航向误差
    
    // 构建状态向量
    VectorXd state(6);
    state << 0, 0, 0, vehicle_speed, cte, epsi;
    
    // 求解MPC优化问题
    std::vector<double> solution = solve_mpc(state, coeffs, wheelbase);
    
    // 获取最优转向角(弧度)
    double delta_rad = solution[0];
    
    // 转换为角度
    double delta_deg = delta_rad * 180.0 / M_PI;
    
    // 限制转角范围
    double bound = 27.0;  // 前轮转向极限
    if (delta_deg > bound) { delta_deg = bound; }
    if (delta_deg < -bound) { delta_deg = -bound; }
    
    // 计算纯跟踪算法的转向角（用于参考比较）
    double alpha = calculate_delta_angle(Azimuth_withGeo(latitude, longitude, 
                                       map_latitudes[target_idx], map_longitudes[target_idx]), 
                                       heading_angle);
    double pure_pursuit_angle = atan2(2.0 * wheelbase * sin(alpha * M_PI / 180.0) / calc_Lf(vehicle_speed), 1.0) * 180.0 / M_PI;
    
    // 输出两种算法的转向角，便于比较
    printf("\nMPC steer_value = %.2f, Pure Pursuit = %.2f, 差值 = %.2f\n", 
           delta_deg, pure_pursuit_angle, delta_deg - pure_pursuit_angle);
    
    return delta_deg;
}