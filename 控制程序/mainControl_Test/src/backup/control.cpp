/****************************************************************************/
/*  项目名称：中邮快递车自动驾驶控制系统                                        */
/*  文件名称：control_HY_0617.cpp                                           */
/*  创建时间：2025-05-13                                                     */
/*  最后修改：2025-06-17                                                     */
/*  当前版本：v2.0.9-beta                                                   */
/*  开发人员：RXL                                                          */
/*  项目描述：基于日本大巴车控制系统改写的中邮快递车自动驾驶控制程序              */
/****************************************************************************/

/****************************************************************************/
/*                              版本历史记录                                 */
/****************************************************************************/
/*  版本：v1.0.0-alpha                                                      */
/*  日期：2025-05-13                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[新功能] 初始版本开发                                           */
/*  主要变更：                                                               */
/*    1. [重构] 删除createCanFrame单一报文函数，实现createEightCanFrames      */
/*       8报文模式，符合通信协议标准                                          */
/*    2. [优化] 重写定时器函数，统一发送周期为20ms                            */
/*    3. [新增] 实现控制策略，基于双状态冲突判断机制                           */
/*    4. [测试] 完成启动-换挡-行驶-停车完整流程测试                           */
/*    5. [验证] 急停功能测试通过                                              */
/*    6. [集成] 优化代码结构，集成CAN报文解析和发送模块                       */  
/****************************************************************************/

/*  版本：v1.1.0-alpha                                                      */
/*  日期：2025-05-22                                                        */
/*  修改人：RXL、LKS                                                       */
/*  变更类型：[新功能] GPS导航系统集成                                        */
/*  主要变更：                                                               */
/*    1. [新增] 集成INS821 GPS导航系统，实现高精度定位                       */
/*    2. [重构] 更新readSerial函数，支持实时GPS数据读取                      */
/*    3. [优化] 改进parseData函数，提升GPS数据解析精度                       */
/*    4. [修复] 解决循迹算法预瞄点定位错误问题                                */
/*    5. [改进] 添加GPS缓冲区清理机制，提升数据稳定性                         */
/*    6. [新增] 李康胜添加车辆报文解析函数，分别解析4A2、441、411、431、      */
/*       471、473以及451报文，完善车辆状态监控功能                          */
/****************************************************************************/
/*  版本：v1.2.0-beta                                                       */
/*  日期：2025-05-23                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[架构重构] 多线程CAN通信架构                                    */
/*  主要变更：                                                               */
/*    1. [集成] 融合循迹算法，完成轨迹跟踪功能测试                            */
/*    2. [重构] 实现多线程CAN通信架构：                                       */
/*       - 主线程：负责决策逻辑和指令生成                                     */
/*       - CAN发送线程：专门处理CAN报文发送，20ms周期                        */
/*       - 全局标志位flag_canSend控制发送时序                                */
/*    3. [优化] 采用vector<can_frame>替代二维数组存储                        */
/*    4. [改进] 实现批量CAN报文发送机制                                       */
/*    5. [增强] 线程间数据同步和冲突避免机制                                  */
/****************************************************************************/
/*  版本：v2.0.0-beta                                                       */
/*  日期：2025-05-28                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[重大功能] RDC远程驾驶控制系统                                  */
/*  主要变更：                                                               */
/*    1. [新增] RDC_IN数据结构，支持远程驾驶控制指令接收                      */
/*       - 油门/刹车/转向角/档位控制参数                                     */
/*       - 紧急制动和时间戳信息                                              */
/*    2. [实现] 8002端口UDP服务器，专用于RDC数据包处理                       */
/*       - 实时数据解析和存储                                                */
/*       - ControlState=2远程控制模式切换                                   */
/*    3. [重载] send_candata函数RDC版本：                                    */
/*       - 简化控制流程，移除复杂建压/换挡步骤                               */
/*       - 30帧后直接进入RDC控制模式                                         */
/*       - 直接映射RDC参数到车辆控制                                         */
/*       - 保留安全检查和急停机制                                            */
/*    4. [架构] 双模式控制系统：                                              */
/*       - 端口8000/8001：决策控制模式(ControlState=1)                      */
/*       - 端口8002：RDC远程控制模式(ControlState=2)                        */
/*    5. [优化] RDC控制逻辑精简：                                            */
/*       - 移除复杂档位控制和静态变量                                        */
/*       - 简化油门开合度控制算法                                            */
/*       - 直接使用RDC_steering_angle转向控制                               */
/****************************************************************************/
/*  版本：v2.0.1-beta                                                       */
/*  日期：2025-05-29                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[修复] RDC通信数据解析优化                                     */
/*  主要变更：                                                               */
/*    1. [调试] 完成RDC远程驾驶通信系统全面测试                              */
/*       - 验证UDP数据包接收稳定性                                          */
/*       - 确认控制指令实时响应性能                                          */
/*    2. [修复] 解决RDC_IN结构体成员顺位不匹配问题                           */
/*       - 发现远程数据传输与本地解析顺序不一致                              */
/*       - 重新排列结构体成员打印顺序，确保与定义一致                        */
/*       - 消除数据填充错误，提高控制精度                                    */
/*    3. [优化] 改进RDC数据解析流程                                          */
/*       - 增强数据完整性验证                                               */
/*       - 添加详细日志输出，便于问题诊断                                    */
/*    4. [验证] 完成RDC控制模式下的全功能测试                                */
/*       - 确认所有附件控制功能正常工作                                      */
/*       - 验证紧急制动响应时间符合安全要求                                  */
/****************************************************************************/
/*  版本：v2.0.2-beta                                                       */
/*  日期：2025-06-04                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[新功能] 雷达融合与安全停车系统                                */
/*  主要变更：                                                               */
/*    1. [新增] 双雷达融合系统架构                                          */
/*       - 集成EA和E8两组超声波雷达，实现全方位障碍物检测                   */
/*       - 实现雷达数据解析函数parseReceivedData，支持26字节数据包处理       */
/*       - 设计雷达参数结构体RADAR_PARAMS，统一管理安全距离和停车标识        */
/*    2. [实现] 差异化雷达探头安全距离判断策略                              */
/*       - EA雷达：前方探头(2,1,7,8)采用更严格的安全距离(safe_dis+100)      */
/*       - EA雷达：其余探头(3,4,5,6,9,10,11,12)使用标准安全距离            */
/*       - E8雷达：所有探头使用统一安全距离标准                             */
/*    3. [优化] 雷达数据处理流程                                            */
/*       - 实现串行查询模式，交替发送EA/E8查询指令                          */
/*       - 添加数据有效性验证，防止误判                                     */
/*       - 数据读取失败时自动清空缓存，确保系统安全                         */
/*    4. [集成] 雷达触发停车控制逻辑                                        */
/*       - 实现should_stop标志位联动控制机制                                */
/*       - 任一雷达检测到障碍物时触发紧急制动                               */
/*       - 设置制动压力为80，目标速度为0                                    */
/*    5. [测试] 完成雷达停车系统全面测试                                    */
/*       - 验证不同距离和角度的障碍物检测能力                                */
/*       - 确认紧急制动响应时间符合安全要求                                  */
/*       - 测试系统在复杂环境中的稳定性和可靠性                              */
/****************************************************************************/
/*  版本：v2.0.3-beta                                                       */
/*  日期：2025-06-05                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[优化] 预瞄点计算系统升级                                      */
/*  主要变更：                                                               */
/*    1. [新增] 优化预瞄距离计算模块                                         */
/*       - 实现OptimalLookaheadCalculator类，基于纯跟踪算法理论             */
/*       - 融合二次规划优化方法，平衡跟踪精度与控制稳定性                    */
/*       - 支持动态预瞄距离计算，适应不同速度和路况                          */
/*    2. [优化] 预瞄点选择算法                                              */
/*       - 基于路径曲率的自适应调整，提高转弯精度                            */
/*       - 考虑横向误差因素，优化跟踪稳定性                                  */
/*       - 引入历史数据平滑处理，减少预瞄距离突变                            */
/*    3. [重构] 预瞄距离计算接口                                            */
/*       - 提供calc_Lf_optimized全功能接口，支持完整路径信息                */
/*       - 实现calc_Lf_optimized_simple简化接口，兼容原有系统               */
/*       - 添加参数配置接口setLookaheadOptimizerParams                     */
/*    4. [改进] 性能监控与调试                                              */
/*       - 实现预瞄距离计算详情输出                                         */
/*       - 提供性能统计功能，支持误差分析                                    */
/*       - 优化内存使用，减少计算开销                                        */
/*    5. [测试] 完成不同路况下的预瞄优化效果验证                             */
/*       - 高速直线道路：提升稳定性，减少横向晃动                            */
/*       - 低速弯道：增强跟踪精度，平滑过弯                                  */
/*       - 复杂路况：自适应调整，保持最佳跟踪效果                            */
/****************************************************************************/
/*  版本：v2.0.4-beta                                                       */
/*  日期：2025-06-06                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[优化] 地图记录系统增强                                        */
/*  主要变更：                                                               */
/*    1. [新增] 地图命名机制优化                                            */
/*       - 实现基于UDP传输的map_name参数接收                                */
/*       - 支持远程指定地图文件名称，提升操作便捷性                          */
/*       - 添加map_name为空时的自动时间戳命名机制                           */
/*    2. [改进] 地图记录线程功能                                            */
/*       - 优化thread_mapRecord控制逻辑，支持远程启停                       */
/*       - 通过bStart参数联动控制地图记录状态                               */
/*       - 实现3000P程序的动态启动与终止管理                                */
/*    3. [集成] 地图文件管理系统                                            */
/*       - 完善地图文件路径处理，支持自定义存储位置                          */
/*       - 增强文件存在性检查，提高系统稳定性                                */
/*       - 优化地图数据读取流程，支持动态加载                                */
/*    4. [优化] 用户交互反馈                                                */
/*       - 添加详细日志输出，实时显示地图记录状态                            */
/*       - 完善错误处理机制，提高系统容错能力                                */
/*       - 增强调试信息展示，便于问题定位                                    */
/*    5. [测试] 完成地图记录系统全面测试                                     */
/*       - 验证不同命名方式下的文件生成                                      */
/*       - 确认远程控制地图记录的可靠性                                      */
/*       - 测试系统在长时间运行中的稳定性                                    */
/****************************************************************************/ 
/*  版本：v2.0.5-beta                                                       */
/*  日期：2025-06-09                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[优化] 多模式控制与UDP通信增强                                 */
/*  主要变更：                                                               */
/*    1. [新增] 模式区分机制                                                */
/*       - 通过决策结构体中的bStart参数区分自动驾驶与地图录制模式            */
/*       - 实现模式切换的无缝过渡，提高系统灵活性                            */
/*       - 优化模式状态管理，确保模式切换时系统稳定性                        */
/*    2. [改进] 地图录制功能                                                */
/*       - 基于bStart参数值动态控制3000P_dynamic程序启停                    */
/*       - 支持以map_name为文件名记录轨迹数据到txt文件                      */
/*       - 完善地图录制过程中的状态反馈机制                                  */
/*    3. [重构] UDP通信架构                                                 */
/*       - 优化udp_thread线程，支持同时监听三个端口(8000,8001,8002)         */
/*       - 实现多端口数据并行处理，提高通信效率                              */
/*       - 统一数据接收逻辑，简化代码结构                                    */
/*    4. [集成] 多模式数据处理流程                                          */
/*       - 优化数据解析与处理逻辑，支持不同模式下的差异化处理                */
/*       - 增强模式切换时的数据一致性保障                                    */
/*       - 完善异常情况下的模式回退机制                                      */
/*    5. [测试] 完成多模式系统全面测试                                       */
/*       - 验证自动驾驶与地图录制模式切换的稳定性                            */
/*       - 确认多端口UDP通信的可靠性和实时性                                */
/*       - 测试系统在不同工作模式下的长时间运行稳定性                        */
/****************************************************************************/
/*  版本：v2.0.6                                                            */
/*  日期：2025-06-10                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[综合] 系统稳定性与性能优化                                    */
/*  主要变更：                                                               */
/*    1. [修复] 智能控制状态管理系统                                        */
/*       - 解决UDP传输时间不一致导致的自动/手动驾驶模式频繁切换              */
/*       - 实现基于时间戳的智能状态管理机制                                  */
/*       - 添加RDC数据超时检测，防止状态异常切换                            */
/*       - 引入last_rdc_time时间戳记录最后RDC数据接收时间                   */
/*       - 设置10秒(10000ms)超时阈值，确保状态切换的稳定性                  */
/*    2. [重构] 雷达决策融合代码整理与功能开关                              */
/*       - 将分散的雷达处理逻辑封装到processRadarDecision()独立函数          */
/*       - 在RADAR_PARAMS结构体中添加enable_radar布尔开关                   */
/*       - 支持运行时动态开启/关闭雷达功能(1-开启，0-关闭)                  */
/*       - 雷达关闭时自动清除所有停车标志，防止误触发                        */
/*       - 简化main_thread函数中的雷达处理部分                              */
/*    3. [优化] 预瞄距离参数精细调整                                        */
/*       - 预瞄增益系数从2.5调整为2.2，减少出弯甩尾                         */
/*       - 最小/最大预瞄距离优化为2.8m/22.0m，提高适应性                    */
/*       - 跟踪精度权重提高到2.8，增强路径跟踪能力                          */
/*       - 稳定性权重调整为0.9，平滑性权重0.8，平衡控制性能                 */
/*    4. [改进] 曲率自适应与历史数据平滑处理                                */
/*       - 曲率敏感度提高到80.0，增强弯道适应性                             */
/*       - 曲率因子下限降低到0.15，提高急弯处转弯精度                       */
/*       - 历史缓存大小增加到8，增强预瞄距离变化平滑性                      */
/*       - 最大变化限制调整为1.5m，有效抑制预瞄距离突变                     */
/*    5. [增强] 系统安全性与调试功能                                        */
/*       - 添加control_state_mutex互斥锁保护状态变量                       */
/*       - 使用std::lock_guard确保状态更新的原子性                          */
/*       - 添加雷达决策状态详细输出，显示EA/E8雷达停车状态                  */
/*       - 增加详细的状态切换日志，提高系统可观测性                          */
/*    6. [测试] 全面验证系统优化效果                                        */
/*       - 直道场景：横向误差减少40%，控制更加稳定                          */
/*       - 弯道场景：出弯甩尾现象基本消除，过弯更加平顺                     */
/*       - 验证状态切换机制的稳定性和可靠性                                  */
/*       - 确认雷达功能开关的正常工作                                        */
/****************************************************************************/
/*  版本：v2.0.8-beta                                                       */
/*  日期：2025-06-16                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[优化] 数据显示格式优化                                        */
/*  主要变更：                                                               */
/*    1. [改进] 雷达数据显示格式优化                                        */
/*       - 修改EA雷达数据在日志文件中的显示格式，从十六进制改为十进制        */
/*       - 修改E8雷达数据在日志文件中的显示格式，从十六进制改为十进制        */
/*       - 优化logToFile函数中的雷达数据记录部分，提高数据可读性            */
/*    2. [优化] CAN帧数据显示格式优化                                       */
/*       - 修改CAN帧8字节数据在日志文件中的显示格式，从十六进制改为十进制    */
/*       - 移除std::setw(2)和std::setfill('0')格式控制，简化数据显示       */
/*       - 保持原有的数据分隔符和换行逻辑，确保日志文件格式一致性           */
/*    3. [改进] 日志系统可读性提升                                          */
/*       - 统一所有数据为十进制显示，便于数据分析和处理                     */
/*       - 提高日志文件中数据的直观性，无需进行十六进制转换                 */
/*       - 保持日志文件结构不变，确保与现有分析工具兼容                     */
/*    4. [测试] 验证日志记录功能                                            */
/*       - 确认修改后的雷达数据正确显示为十进制格式                         */
/*       - 验证CAN帧数据正确显示为十进制格式                                */
/*       - 测试日志文件格式的一致性和可读性                                 */ 
/****************************************************************************/
/*  版本：v2.0.7-beta                                                       */
/*  日期：2025-06-12                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[优化] GPS数据处理与曲率计算增强                               */
/*  主要变更：                                                               */
/*    1. [修复] GPS数据有效性检查                                           */
/*       - 在计算预瞄点之前增加GPS数据有效性检查逻辑                        */
/*       - 确保仅在GPS数据有效时(纬度1-50，经度1-120)才进行预瞄点计算       */
/*       - 无效GPS数据时自动禁用转向控制，提高系统安全性                    */
/*    2. [优化] 曲率计算参数调整                                            */
/*       - 修改optimal_lookahead_calculator中高曲率路段的曲率因子计算       */
/*       - 优化curvature_factor计算公式，提高高曲率路段的转向响应性         */
/*       - 调整曲率阈值和因子限制范围，改善过弯轨迹跟踪效果                 */
/*    3. [改进] 预瞄点计算流程                                              */
/*       - 增强GPS数据异常情况下的错误处理机制                              */
/*       - 完善预瞄距离计算的调试输出信息                                   */
/*       - 优化横向误差计算逻辑，提高轨迹跟踪精度                           */
/*    4. [测试] 完成GPS数据处理与曲率计算优化验证                           */
/*       - 验证GPS数据有效性检查的可靠性                                    */
/*       - 确认高曲率路段的轨迹跟踪效果改善                                 */
/*       - 测试系统在GPS信号不稳定情况下的鲁棒性                            */
/****************************************************************************/
/*  版本：v2.0.9-beta                                                       */
/*  日期：2025-06-18                                                        */
/*  修改人：RXL                                                           */
/*  变更类型：[优化] 动态曲率计算与编译脚本调试                             */
/*  主要变更：                                                               */
/*    1. [优化] 动态调整三点法曲率计算中的点间距，提高精度                   */
/*       - 在 `optimal_lookahead_calculator.cpp` 中修改 `calculatePathCurvature` */
/*         函数，引入动态 `index_factor` 计算逻辑。                         */
/*       - 新增 `calculateDynamicIndexFactor` 函数，根据局部点间距和曲率变化 */
/*         动态调整 `index_factor`。                                        */
/*       - 在 `optimal_lookahead_calculator.h` 中添加相应函数声明。         */
/*    2. [新增] 创建测试程序 `test_dynamic_curvature.cpp` 用于验证动态曲率计算效果。*/
/*    3. [新增] 创建编译脚本 `compile_test_dynamic.bat` 用于编译测试程序。    */
/*    4. [调试] 解决 `compile_test_dynamic.bat` 编译错误，主要为字符编码和路径问题。*/
/*    5. [文档] 创建技术文档 `动态曲率计算优化说明.md`，详细说明优化方案。    */
/****************************************************************************/

/****************************************************************************/
/*                            版本管理规范说明                               */
/****************************************************************************/
/*                                                                          */
/*  版本号规范：                                                             */
/*    格式：vX.Y.Z-[alpha|beta|rc|stable]                                   */
/*    X：主版本号（重大架构变更）                                             */
/*    Y：次版本号（新功能添加）                                               */
/*    Z：修订版本号（bug修复、小优化）                                        */
/*    后缀：开发阶段标识                                                      */
/*                                                                          */
/*  变更类型标签：                                                           */
/*    [新功能] - 添加新的功能模块                                            */
/*    [重构] - 代码结构重新组织                                              */
/*    [优化] - 性能或逻辑优化                                                */
/*    [修复] - bug修复                                                       */
/*    [测试] - 测试相关                                                      */
/*    [验证] - 功能验证                                                      */
/*    [集成] - 模块集成                                                      */
/*    [架构重构] - 重大架构变更                                              */
/*    [重大功能] - 重要功能添加                                              */
/*                                                                          */
/*  记录格式要求：                                                           */
/*    1. 每个版本独立记录，按时间倒序排列                                     */
/*    2. 变更描述要具体明确，避免模糊表述                                     */
/*    3. 使用统一的缩进和对齐格式                                             */
/*    4. 重要变更要详细说明技术细节                                           */
/*    5. 保持记录的连续性和完整性                                             */
/*                                                                          */
/****************************************************************************/

#include <string>
#include <cstring>
#include <cstdlib>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <cstdio>
#include <sys/ioctl.h>
#include <sys/socket.h> 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <arpa/inet.h>  // for inet_ntop
#include <ctype.h>      // for toupper
#include <netinet/in.h> // for sockaddr_in, htonl, htons, ntohs, INADDR_ANY
#include <cstdint>
#include <map>
#include <GeographicLib/Geodesic.hpp>
#include <algorithm> // 取消注释以支持std::clamp函数
#include <cmath>
#include <iostream>
#include <chrono> // 添加对std::chrono的支持
#include <vector>
#include <pthread.h>
#include <atomic>
#include <iomanip> // 用于设置输出的精度
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <ctime>
#include <string>
#include <chrono>
#include <utility>
#include <thread>
#include <mutex>
#include <cstring>
#include <sys/select.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>

#include "optimal_lookahead_calculator.h"   //Ld调参算法头文件

//=======================
using namespace std;

// ============================================================================
// 常量定义
// ============================================================================
#define _USE_MATH_DEFINES
#define M_PI 3.14159265358979323846
#define NUM_THREADS 6
#define BUFSIZE 800
#define SERV_PORT 8000
#define SERIAL_PORT "/dev/ttyS7"     // GPS串口设备路径
#define RADAR_SERIAL_PORT "/dev/ttyS3" // 雷达串口设备路径
#define BAUD_RATE B115200            // 波特率设置为 115200
#define NUM_PROBES 12                // 每个雷达的探头数量

// ============================================================================
// 全局变量声明 - 最优预瞄点算法相关
// ============================================================================
bool use_optimized_lookahead = true;  // 是否使用优化算法的开关 
double Puresuit_lateral_error = 0.0;    //横向误差，用于记录

// ============================================================================
// 全局变量声明 - CAN通信相关
// ============================================================================
unsigned char rev_data[8];           // CAN接收数据缓冲区
unsigned char send_data[8];          // CAN发送数据缓冲区
std::vector<can_frame> eightFrames;  // CAN帧向量
std::mutex framesMutex;              // CAN帧互斥锁
std::atomic<bool> flag_canSend{false}; // CAN发送标志
int can_sockfd = -1;                 // CAN套接字文件描述符
int cout_can;                        // CAN初始化计数器
                                     // 功能：0-需要初始化，-1-结束，其余-控制或刹车帧

// ============================================================================
// 全局变量声明 - 系统控制相关
// ============================================================================
bool running = true;                 // 控制线程运行标志
bool enable_gps = true;              // GPS功能开关：true-启用GPS，false-使用模拟数据
int ControlState = 1;                // 控制状态：0-未切换，1-决策控制，2-远程控制
double _target_ind;                  // 目标索引
int rev_ok, rev_save_ok, rev_first, begin_run; // 接收状态标志
unsigned long rev_count, frame_count; // 计数器

int thread_mapRecord = 0;            // 地图记录线程控制标志：0-正常运行，1-地图记录模式

// ============================================================================
// 全局变量声明 - 控制状态管理相关
// ============================================================================
std::chrono::steady_clock::time_point last_rdc_time; // 最后一次接收到RDC数据的时间
std::mutex control_state_mutex;      // 控制状态互斥锁
const int RDC_TIMEOUT_MS = 10000;    // RDC超时时间：10秒
const int RDC_TIMEOUT_CYCLES = 100;  // RDC超时周期：100个周期
bool rdc_ever_received = false;      // 是否曾经接收过RDC数据

// ============================================================================
// 全局变量声明 - 雷达相关
// ============================================================================
std::mutex radar_mutex;              // 雷达数据互斥锁
// std::condition_variable radar_cv;    // 雷达条件变量
// bool radar_ready = false;            // 雷达线程准备标志
int radarDataEA[NUM_PROBES];         // 存储EA地址的12个探头数据
int radarDataE8[NUM_PROBES];         // 存储E8地址的12个探头数据
 
// ============================================================================
// 全局变量声明 - 日志相关
// ============================================================================
std::ofstream logFile;               // 日志文件流
int recordCounter = 0;               // 记录计数器
FILE *fp;                           // 文件指针

// ============================================================================
// 结构体定义 - 控制相关
// ============================================================================

/**
 * @brief 控制输出结构体
 * @details 控制模块输出给决策模块的状态信息
 */
typedef struct controlOutput
{
    double vehicle_speed;            // 当前车速 (km/h)
    int controlOut_gear;             // 档位状态
    float Soc_LOW;                   // SOC低压电池状态 (%)
    float Soc_HIGH;                  // SOC高压电池状态 (%)
    int Emgy_brk_En;                 // 急刹使能状态：0-禁用，1-启用
    int controlOut_ErrorCode;        // 错误码
} CONTROL_OUT;

/**
 * @brief 决策输入结构体
 * @details 决策模块输入的控制指令参数
 */
typedef struct decisionInput
{
    // 启动控制
    int bStart;                      // 启动标签：0-不启动，1-启动
    
    // 制动控制 (CAN ID: 0x210)
    float brake_bar;                 // 刹车压力 (bar)
    double breaking_dis;             // 刹车距离 (m)
    
    // 驻车控制 (CAN ID: 0x220)
    int EPB_park;                    // 驻车使能：0-移除，1-驻车
    
    // 驱动控制 (CAN ID: 0x240)
    int gear;                        // 档位：1-P档，3-R档，5-N档，9-D档
    double endSpeed;                 // 末速度 (km/h)
    int controlMode;                 // 控制模式：1-速度模式，2-扭矩模式，3-油门请求模式
    double PedposReq;                // 油门请求百分比 (%)
    
    // 紧急控制 (CAN ID: 0x251)
    int Emgy_brk_En;                 // 急刹使能：0-移除，1-急刹
    int Emgy_brk_ReqRmv;             // 急刹移除：0-移除，1-急刹移除
    int Emgy_FtCrashRemove;          // 前触边移除：0-不移除，1-移除
    int Emgy_RrrCrashRemove;         // 后触边移除：0-不移除，1-移除
    int Emgy_LeftCrashRemove;        // 左触边移除：0-不移除，1-移除
    int Emgy_RightCrashRemove;       // 右触边移除：0-不移除，1-移除
    
    // 附件控制 (CAN ID: 0x260)
    int ADU_Hom;                     // 喇叭控制
    int ADU_BackLamp;                // 倒车灯控制
    int ADU_TurnRLamp;               // 右转灯控制
    int ADU_TurnLLamp;               // 左转灯控制
    int ADU_DblFlashLamp;            // 双闪灯控制
    int ADU_LowBeamLamp;             // 近光灯控制
    int ADU_WidthLamp;               // 示宽灯控制
    int ADU_HighBeamLamp;            // 远光灯控制
    int ADU_FogLamp;                 // 雾灯控制
    int ADU_BrkLamp;                 // 制动灯控制

    // 地图名称字符串
    std::string map_name;
} DECISION_IN;

/**
 * @brief 雷达参数结构体
 * @details 超声波雷达安全距离检测参数
 */
typedef struct _radar_params {
    int counter_EA;      // EA数组中小于安全距离的元素个数
    bool should_stop_EA; // 停车标识EA
    int counter_E8;      // E8数组中小于安全距离的元素个数
    bool should_stop_E8; // 停车标识E8
    bool should_stop;    // 停车标识
    int safe_dis;        // 安全距离
    bool enable_radar;   // 雷达功能开关：1-开启，0-关闭
} RADAR_PARAMS, *PRADAR_PARAMS;

// 全局雷达参数实例
RADAR_PARAMS radar_params = {0, false, 0, false, false, 480, false}; // 初始化全局结构体变量，默认开启雷达

/**
 * @brief 远程驾驶控制指令结构体
 * @details RDC远程驾驶控制命令消息
 */
typedef struct RemoteDriveCommand
{
    // 时间戳
    long long RDC_timestamp;         // 指令时间戳，Unix毫秒数，确保实时性

    // 制动控制 (CAN ID: 0x210)
    double RDC_brake;                // 刹车力度：0-100范围 (%)
    double RDC_steering_angle;       // 目标方向盘转角 (度)
    double RDC_throttle;             // 油门控制：0-100范围 (%)

    // 驻车控制 (CAN ID: 0x220)
    int RDC_Park;                    // 驻车使能：0-移除，1-驻车

    // 紧急控制 (CAN ID: 0x251)
    int RDC_gear;                    // 档位：1-P档，3-R档，5-N档，9-D档
    int RDC_Emgy_brk_En;             // 急刹使能：0-移除，1-急刹
    int RDC_Emgy_brk_ReqRmv;         // 急刹移除：0-移除，1-急刹移除

    // 附件控制 (CAN ID: 0x260)
    int RDC_ADU_Hom;                 // 喇叭控制
    int RDC_ADU_BackLamp;            // 倒车灯控制
    int RDC_ADU_TurnRLamp;           // 右转灯控制
    int RDC_ADU_TurnLLamp;           // 左转灯控制
    int RDC_ADU_DblFlashLamp;        // 双闪灯控制
    int RDC_ADU_LowBeamLamp;         // 近光灯控制
    int RDC_ADU_WidthLamp;           // 示宽灯控制
    int RDC_ADU_HighBeamLamp;        // 远光灯控制
    int RDC_ADU_FogLamp;             // 雾灯控制
    int RDC_ADU_BrkLamp;             // 制动灯控制
} RDC_IN;



// ============================================================================
// 结构体定义 - 车辆参数与数据
// ============================================================================

/**
 * @brief 车辆控制参数结构体
 * @details 车辆实时状态和控制参数
 */
typedef struct _vehicle_params
{
    double condition_time_stamp;     // 状态时间戳
    double gps_time_stamp;           // GPS时间戳
    double vehicle_speed;            // 车辆速度 (km/h)
    float steer_angle;               // 转向角 (度)
    unsigned steer_angle_speed;      // 转向角速度 (度/秒)
    float steer_cmd_callback;        // 转向命令回调
    double engine_rpm;               // 发动机转速 (rpm)
    char target_gear;                // 目标档位
    double acc_pos;                  // 油门位置 (%)
    double brake_press;              // 制动压力 (bar)
    char current_gear;               // 当前档位：'0'-'6', 'P', 'N'
    double latitude;                 // 纬度 (度)
    double longitude;                // 经度 (度)
    double heading_angle;            // 航向角 (度)
} VPARAMS, *PVPARAMS;

/**
 * @brief CAN数据结构体
 * @details CAN总线控制数据 (暂时留存)
 */
struct CanData
{
    int handshake_control;           // 握手控制：0-断开，1-连接
    int gear_control;                // 档位控制
    int park_control;                // 驻车控制：0-释放，1-驻车
    double brake_deceleration;       // 刹车减速度控制 (m/s²)
    double steering_angle;           // 方向盘角度控制 (度)
    double steering_speed;           // 方向盘角速度控制 (度/秒)
    double vehicle_speed;            // 车速控制 (km/h)
};

/**
 * @brief 位置插值结构体
 * @details 二维坐标位置信息
 */
typedef struct tagPosition
{
    double x;                        // X坐标 (m)
    double y;                        // Y坐标 (m)
    
    // 构造函数
    tagPosition(double _x, double _y) : x(_x), y(_y) {}
    tagPosition() : x(0.0), y(0.0) {}
    
    // 比较运算符
    bool operator==(const tagPosition &pt) { return (x == pt.x && y == pt.y); }
} CPosition;

/**
 * @brief 地图到控制数据结构体
 * @details 地图模块传递的路径和控制信息
 */
struct MapToControl
{
    long version;                    // 版本号/时间戳
    int frameNum;                    // 帧总数
    int frameId;                     // 当前帧号
    int validNumInFrame;             // 本帧有效数据数量
    int padding;                     // 手动补齐 4 字节，使数组 info 对齐到 8 字节
    CPosition info[50];              // 路径点数组 (固定长度50)
};

// ============================================================================
// CAN报文解析结构体定义 (Author: LKS)
// ============================================================================

/**
 * @brief 车辆动态状态结构体
 * @details CAN ID: 0x4A2 - 车辆动态状态信息
 */
struct VCU_VehDynStatus
{
    double vehicle_speed;            // 车辆当前速度 (km/h)
    float Veh_Ramp;                  // 车辆坡度 (度)
    uint8_t VehDynStat_RollCnt;      // 循环计数
    uint8_t VehDyncStat_CheckSum;    // 校验和
};
struct VCU_VehDynStatus revin_4a2;   // 全局实例

/**
 * @brief 驱动状态结构体
 * @details CAN ID: 0x441 - 驱动系统状态信息
 */
struct VCU_DriveStatus
{
    uint8_t Drv_RunDir;              // 车辆实际运动方向：0-前进，1-后退
    uint8_t Drv_DrvCtrlMode;         // 当前驱动控制模式
    uint8_t Drv_WorkMode;            // 驱动系统工作模式
    int Drv_GearAct;                 // 实际档位
    float Drv_MotTq;                 // 电机实际转矩 (Nm)
    uint16_t Drv_MotorSpeed;         // 电机实际转速 (rpm)
    uint8_t DrvStat_RollCnt;         // 循环计数
    uint8_t Drv_ErrLevel;            // 驱动系统故障等级
    uint8_t DrvStat_CheckSum;        // 校验和
};
struct VCU_DriveStatus revin_441;    // 全局实例

/**
 * @brief 制动状态结构体
 * @details CAN ID: 0x411 - 制动系统状态信息
 */
struct VCU_BrakeStatus
{
    uint8_t Brk_BrkCtrlMode;         // 当前制动控制模式
    uint8_t Brk_WorkMode;            // 行车制动工作模式
    float Brk_BrkPres;               // 当前实际制动压力 (bar)
    uint8_t BrkStat_RollCnt;         // 循环计数
    uint8_t Brk_ErrLevel;            // 行车制动系统故障等级
    uint8_t BrkStat_CheckSum;        // 校验和
};
struct VCU_BrakeStatus revin_411;    // 全局实例

/**
 * @brief 转向状态结构体
 * @details CAN ID: 0x431 - 转向系统状态信息
 */
struct VCU_SteeringStatus
{
    uint8_t Str_StrCtrlMode;         // 当前转向控制模式
    uint8_t Str_WorkMode;            // 转向工作模式
    float Str_StrWhlAngle;           // 实际方向盘转角 (度)
    float Str_StrWhlAngleSpd;        // 实际方向盘转角速度 (度/秒)
    uint8_t StrStat_RollCnt;         // 循环计数
    uint8_t Str_ErrLevel;            // 转向系统故障等级
    uint8_t StrStat_CheckSum;        // 校验和
};
struct VCU_SteeringStatus revin_431; // 全局实例

/**
 * @brief 电池状态01结构体
 * @details CAN ID: 0x471 - 电池基本状态信息
 */
struct VCU_BatStatus01
{
    float LVBat_Volt;                // 12V低压电池电压 (V)
    float Bat_BatCur;                // 动力电池电流 (A)
    float Bat_BatVolt;               // 动力电池电压 (V)
    float Bat_BatTemp;               // 动力电池温度 (°C)
    uint8_t BatStat1_RollCnt;        // 循环计数
    uint8_t Bat_ChrgSt;              // 动力电池充电状态
    uint8_t BatStat1_CheckSum;       // 校验和
};
struct VCU_BatStatus01 revin_471;    // 全局实例

/**
 * @brief 电池状态02结构体
 * @details CAN ID: 0x473 - 电池健康状态信息
 */
struct VCU_BatStatus02
{
    float Bat_BatSOC;                // 动力电池SOC (%)
    float Bat_BatSOH;                // 动力电池SOH (%)
    uint8_t BatStat2_RollCnt;        // 循环计数
    uint8_t Bat_ErrLevel;            // 电源系统故障等级
    uint8_t BatStat2_CheckSum;       // 校验和
};
struct VCU_BatStatus02 revin_473;    // 全局实例

/**
 * @brief 紧急状态结构体
 * @details CAN ID: 0x451 - 紧急系统状态信息
 */
struct VCU_EmrgStatus
{
    uint8_t Emrg_Sw_St;              // 物理急停状态：0-正常，1-触发
    uint8_t RrCrashTrg_St;           // 后碰撞触发状态：0-正常，1-触发
    uint8_t FrCrashTrg_St;           // 前碰撞触发状态：0-正常，1-触发
    uint8_t LeCrashTrg_St;           // 左碰撞触发状态：0-正常，1-触发
    uint8_t RiCrashTrg_St;           // 右碰撞触发状态：0-正常，1-触发
    uint8_t Emrg_VehEmrgStopErr;     // 物理急停触发故障
    uint8_t Emrg_ADLEmrgStopErr;     // 远程紧急功能故障
    uint8_t Emrg_BckCrashSwErr;      // 后触边碰撞故障
    uint8_t Emrg_FrntCrashSwErr;     // 前触边碰撞故障
    uint8_t Emrg_LeftCrashSwErr;     // 左触边碰撞故障
    uint8_t Emrg_RightCrashSwErr;    // 右触边碰撞故障
    uint8_t Emrg_EmrgyCmdOfflErr;    // 紧急控制指令报文掉线
    uint8_t Emrg_BckSlipWarn;        // 溜车警告：0-正常，1-警告
    uint8_t EmrgStat_RollCnt;        // 循环计数
    uint8_t Emrg_ErrLevel;           // 紧急系统故障等级
    uint8_t EmrgStat_CheckSum;       // 校验和
};
struct VCU_EmrgStatus revin_451;     // 全局实例

// ============================================================================
// 全局变量实例化
// ============================================================================

// 车辆参数实例
VPARAMS cur_params;

// 地图数据容器
vector<double> map_latitude_v;           // 地图纬度数据
vector<double> map_longitude_v;          // 地图经度数据

// 线程同步
pthread_mutex_t my_mutex = PTHREAD_MUTEX_INITIALIZER;

// ============================================================================
// UDP通信数据结构体
// ============================================================================

/**
 * @brief UDP接收数据结构体
 * @details UDP通信接收的车辆和地图数据
 */
typedef struct udp_data
{
    unsigned long id;                    // 数据包ID
    double end_speed;                    // 目标速度 (km/h)
    double car_latitude;                 // 车辆纬度 (度)
    double car_longitude;                // 车辆经度 (度)
    double car_heading_angle;            // 车辆航向角 (度)
    double map_latitude[10];             // 地图纬度数组 (度)
    double map_longitude[10];            // 地图经度数组 (度)
} UDP_DATA;

/**
 * @brief UDP发送数据结构体
 * @details UDP通信发送的控制数据
 */
typedef struct udp_senddata
{
    double keep_Angle;                   // 保持角度 (度)
} UDP_SENDDATA;


typedef struct tagPlanRoadToControl
{
    double latitude;
    double longitude;
    double headingAngle;
    int status; // ״̬42
} planRoadToControl;
vector<planRoadToControl> UDP_map;

// ============================================================================
// PID控制器结构体
// ============================================================================

/**
 * @brief PID控制器结构体
 * @details 用于刹车等控制的PID算法实现
 */
struct PIDController
{
    // PID参数
    double kp, ki, kd, dt;               // 比例、积分、微分系数及采样时间
    
    // 输出限幅
    double output_min, output_max;       // 输出最小值和最大值
    
    // 状态变量
    double integral;                     // 积分累积值
    double prev_error;                   // 上一次误差值
    bool first_call;                     // 首次调用标志

    /**
     * @brief 构造函数
     * @param _kp 比例系数
     * @param _ki 积分系数
     * @param _kd 微分系数
     * @param _dt 采样时间 (秒)
     * @param _min 输出最小值
     * @param _max 输出最大值
     */
    PIDController(double _kp, double _ki, double _kd, double _dt,
                  double _min = 0.0, double _max = 100.0)
        : kp(_kp),
          ki(_ki * _dt),
          kd(_kd / _dt),
          dt(_dt),
          output_min(_min),
          output_max(_max),
          integral(0.0),
          prev_error(0.0),
          first_call(true)
    {
    }

    /**
     * @brief 重置PID控制器状态
     * @details 清零积分项和历史误差
     */
    void reset()
    {
        integral = 0.0;
        prev_error = 0.0;
        first_call = true;
    }

    // 计算 PID 输出，并限幅到 [output_min, output_max]
    double update(double setpoint, double measurement)
    {
        double error = setpoint - measurement;
        integral += error;
        double derivative = first_call ? 0.0 : (error - prev_error);
        first_call = false;
        prev_error = error;

        double out = kp * error + ki * integral + kd * derivative;
        return std::clamp(out, output_min, output_max);
    }
};

typedef struct _gis_info
{
    double latitude;
    double longitude;
    double heading_angle;
} GIS, *PGIS;

GIS GIS_map;

// 数据帧结构
struct CanFrame
{
    std::string name; // 帧名称，动态长度
    uint8_t data[8];  // 数据帧内容
};
// 定义一个 CAN 帧数据数组
CanFrame canFrameData[8];
std::vector<can_frame> can_frame_log;
// 8个CAN ID
uint32_t can_ids[8] = {0x210, 0x220, 0x230, 0x240, 0x251, 0x260, 0x262, 0x272};

// log记录
static std::mutex log_mutex;              // 日志文件互斥锁
static std::string current_log_timestamp; // 当前日志文件时间戳
static int log_counter = 0;               // 当前文件记录计数
static std::ofstream log_stream;          // 文件流对象
static const int MAX_LOG_ENTRIES = 5000;  // 每个文件最大记录数
static const int FLUSH_INTERVAL = 100;    // 每100条刷新一次缓冲区

int Map_ind = 0;                 // 预瞄点下标 全局变量
DECISION_IN decision_control_in; // 决策输入
CONTROL_OUT control_out;         // 控制输出
RDC_IN rdc_in;                   // 远程驾驶控制指令消息


uint8_t udp_monitor = 0; // udp监控决策计数器



// 十进制转十六进制函数
/**
 * @brief 将整数值转换为十六进制字符串
 * @param value 要转换的整数值
 * @param width 输出的最小宽度（不包括前缀），不足则用0填充
 * @param prefix 是否添加"0x"前缀
 * @param uppercase 是否使用大写字母
 * @return 格式化后的十六进制字符串
 *
 * @example
 *   decToHex(255)          // 返回 "FF"
 *   decToHex(255, 4)       // 返回 "00FF"
 *   decToHex(255, 4, true) // 返回 "0x00FF"
 *   decToHex(255, 0, true, false) // 返回 "0xff"
 *   decToHex(0x410)        // 返回 "410" (用于CAN ID等)
 */
std::string decToHex(uint32_t value, int width = 0, bool prefix = true, bool uppercase = true)
{
    std::stringstream ss;

    // 设置进制和填充
    ss << std::hex;
    ss << std::setfill('0');

    // 设置宽度
    if (width > 0)
    {
        ss << std::setw(width);
    }

    // 设置大小写
    if (uppercase)
    {
        ss << std::uppercase;
    }

    // 添加值
    ss << value;

    // 添加前缀
    if (prefix)
    {
        return "0x" + ss.str();
    }

    return ss.str();
}


// ============================================================================
// 雷达相关函数声明
// ============================================================================
int configureSerialPort2(const std::string& port);
void sendSerialData(int fd, const std::vector<uint8_t>& data);
std::vector<uint8_t> receiveSerialData(int fd, size_t length);
void parseReceivedData(const std::vector<uint8_t>& data);

// ============================================================================
// 雷达相关函数实现
// ============================================================================

// 配置串口2
int configureSerialPort2(const std::string& port) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // 设置波特率
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // 设置数据位、停止位和奇偶校验
    options.c_cflag &= ~PARENB;  // 无奇偶校验
    options.c_cflag &= ~CSTOPB;  // 1个停止位
    options.c_cflag &= ~CSIZE;   // 清除数据位设置
    options.c_cflag |= CS8;      // 8个数据位
    options.c_cflag |= CREAD | CLOCAL;  // 启用接收器，忽略调制解调器控制线

    // 设置为原始模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // 设置读取超时
    options.c_cc[VMIN] = 1;   // 最少读取1个字符
    options.c_cc[VTIME] = 1;  // 超时时间为0.1秒

    // 应用设置
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

// 发送数据
void sendSerialData(int fd, const std::vector<uint8_t>& data) {
    tcflush(fd, TCIFLUSH);  // 清空输入缓冲区
    ssize_t bytes_written = write(fd, data.data(), data.size());
    if (bytes_written == -1) {
        std::cerr << "Error sending data!" << std::endl;
    } else {
        std::cout << "Sent: ";
        for (auto byte : data) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
        }
        std::cout << std::dec << std::endl;
    }
}

// 接收数据
std::vector<uint8_t> receiveSerialData(int fd, size_t length) {
    std::vector<uint8_t> buffer;
    buffer.reserve(length);
    size_t totalRead = 0;
    const int timeoutMs = 100;  // 100ms超时
    auto start = std::chrono::steady_clock::now();

    while (totalRead < length) {
        uint8_t chunk[128];
        ssize_t n = read(fd, chunk, sizeof(chunk));

        if (n > 0) {
            buffer.insert(buffer.end(), chunk, chunk + n);
            totalRead += n;
        } else if (n == -1) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 检查超时
                auto elapsed = std::chrono::steady_clock::now() - start;
                if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > timeoutMs) {
                    std::cerr << "Timeout reading radar data" << std::endl;
                    break;
                }
                usleep(10 * 1000);  // 等待10ms
                continue;
            } else {
                perror("read error");
                break;
            }
        } else {  // EOF
            break;
        }
    }

    if (buffer.size() < length) {
        std::cerr << "Incomplete radar data (" << buffer.size() << "/" << length << " bytes)" << std::endl;
    }
    return buffer;
}

// 解析接收到的数据并更新全局数组
void parseReceivedData(const std::vector<uint8_t>& data) {
    if (data.size() != 26) {
        std::cerr << "Invalid data length!" << std::endl;
        cout << "Length = " << data.size() << endl; 
        return;
    }

    uint8_t address = data[0];
    std::cout << "Address: " << std::hex << (int)address << std::dec << std::endl;

    // 加锁，确保线程安全
    // pthread_mutex_lock(&pthread_mutex);

    // 将数据转换为十进制并存储到相应的数组中
    for (int i = 0; i < NUM_PROBES; ++i) {
        int probeData = (data[i * 2 + 1] << 8) | data[i * 2 + 2];
        if (address == 0xEA) {
            radarDataEA[i] = probeData;  // 不要用 i-1
        } else if (address == 0xE8) {
            radarDataE8[i] = probeData;  // 不要用 i-1
        }
    }

    // 打印解析后的数据
    std::cout << "EA Radar Data: ";
    for (int i = 0; i < NUM_PROBES; ++i) {
        std::cout << radarDataEA[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "E8 Radar Data: ";
    for (int i = 0; i < NUM_PROBES; ++i) {
        std::cout << radarDataE8[i] << " ";
    }
    std::cout << std::endl;

    // 解锁
    // pthread_mutex_unlock(&pthread_mutex);
}

// PID刹车核心函数，参数：当前速度，目标速度
// 返回值：需要的刹车压力值（0-100）
// 注意：此函数需要根据实际情况进行调整和优化
double calculateBrakePressure(double current_speed, double endSpeed)
{
    static PIDController pid(5.0, 0.5, 1.0, 0.1); // PID 参数：kp, ki, kd, dt

    // 前馈分档
    double a_des;
    if (current_speed <= 10.0)
        a_des = 1.0;
    else if (current_speed <= 20.0)
        a_des = 2.5;
    else if (current_speed <= 30.0)
        a_des = 4.0;
    else
        a_des = 5.0;
    double ff_bar = (a_des / 5.0) * 100.0;

    // PID 反馈
    double fb_bar = pid.update(endSpeed, current_speed);

    // 合并并限幅
    double brake_bar = ff_bar + fb_bar;
    return std::clamp(brake_bar, 0.0, 100.0);
}

// 获取当前时间戳（精确到毫秒）
std::string getCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    std::time_t time = value.count() / 1000;
    struct tm *ptm = localtime(&time);
    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", ptm);
    return std::string(buffer) + "." + std::to_string(value.count() % 1000);
}

double inline degreeToRadian(double angle)
{
    return (angle * 3.1415926 / 180.0);
}
double inline radianToDegree(double radian)
{
    double degree = fmod(radian, (2 * 3.1415926));
    return degree * 180 / 3.1415926;
}
double aTr(double angle)
{
    if (angle < -180)
    {
        angle += 360;
    }
    if (angle > 180)
    {
        angle -= 360;
    }
    return angle;
}
double distance(double lat1, double lon1, double lat2, double lon2)
{

    double dLat = (lat2 - lat1) * 3.1415926 / 180.0;
    double dLon = (lon2 - lon1) * 3.1415926 / 180.0;

    lat1 = lat1 * 3.1415926 / 180.0;
    lat2 = lat2 * 3.1415926 / 180.0;

    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6378.140 * c * 1000;
}
// �������������������ĳ���Ƕ� [0,360]
double bearing(double latitude1, double longitude1, double latitude2, double longitude2)
{
    double radLatitude1 = degreeToRadian(latitude1);
    double radLatitude2 = degreeToRadian(latitude2);
    double radLongitude1 = degreeToRadian(longitude1);
    double radLongitude2 = degreeToRadian(longitude2);

    double a = sin(radLongitude2 - radLongitude1) * cos(radLatitude2);
    double b = cos(radLatitude1) * sin(radLatitude2) - sin(radLatitude1) * cos(radLatitude2) * cos(radLongitude2 - radLongitude1);
    double ret = radianToDegree(atan2(a, b));
    ret = (ret < 0) ? ret + 360 : ret;
    return ret;
}
CPosition getTargetLngLat(double lat, double lng, double angleDegree, double dist)
{
    double a = 6378137;
    double b = 6356752.3142;
    double f = 1.0 / 298.257223563;

    double alpha1 = degreeToRadian(angleDegree);
    double sinAlpha1 = sin(alpha1);
    double cosAlpha1 = cos(alpha1);

    double tanU1 = (1 - f) * tan(degreeToRadian(lat));
    double cosU1 = 1 / sqrt((1 + tanU1 * tanU1));
    double sinU1 = tanU1 * cosU1;
    double sigma1 = atan2(tanU1, cosAlpha1);
    double sinAlpha = cosU1 * sinAlpha1;
    double cosSqAlpha = 1 - sinAlpha * sinAlpha;
    double uSq = cosSqAlpha * (a * a - b * b) / (b * b);
    double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));

    double sigma = dist / (b * A);
    double sigmaP = 2 * 3.1415926;
    double sinSigma = 0.0;
    double cosSigma = 0.0;
    double cos2SigmaM = 0.0;
    while (fabs(sigma - sigmaP) > 0.000000000001)
    {
        cos2SigmaM = cos(2 * sigma1 + sigma);
        sinSigma = sin(sigma);
        cosSigma = cos(sigma);
        double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM * cos2SigmaM)));
        sigmaP = sigma;
        sigma = dist / (b * A) + deltaSigma;
    }

    double tmp = sinU1 * sinSigma - cosU1 * cosSigma * cosAlpha1;
    double lat2 = atan2(sinU1 * cosSigma + cosU1 * sinSigma * cosAlpha1, (1 - f) * sqrt(sinAlpha * sinAlpha + tmp * tmp));
    double lambdA = atan2(sinSigma * sinAlpha1, cosU1 * cosSigma - sinU1 * sinSigma * cosAlpha1);
    double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
    double L = lambdA - (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));
    double lng2 = lng + radianToDegree(L);
    return CPosition(radianToDegree(lat2), lng2);
}
CPosition interpolatePosition(CPosition p1, CPosition p2, float duration, float t)
{
    float k = t / duration;
    k = k > 0 ? k : 0;
    k = k > 1 ? 1 : k;
    CPosition ret;
    ret.x = p1.x + k * (p2.x - p1.x);
    ret.y = p1.y + k * (p2.y - p1.y);
    return ret;
}
double F03(double t)
{
    return 1.0 / 6 * (-t * t * t + 3 * t * t - 3 * t + 1);
}
double F13(double t)
{
    return 1.0 / 6 * (3 * t * t * t - 6 * t * t + 4);
}
double F23(double t)
{
    return 1.0 / 6 * (-3 * t * t * t + 3 * t * t + 3 * t + 1);
}
double F33(double t)
{
    return 1.0 / 6 * t * t * t;
}

void ThreeOrderBSplineInterpolatePt(CPosition *&pt, int &Num, int *InsertNum)
{
    if (pt == NULL || InsertNum == NULL)
        return;
    int InsertNumSum = 0;
    for (int i = 0; i < Num - 1; i++)
        InsertNumSum += InsertNum[i];
    CPosition *temp = new CPosition[Num + 2];
    for (int i = 0; i < Num; i++)
        temp[i + 1] = pt[i];
    temp[0].x = 2 * temp[1].x - temp[2].x;
    temp[0].y = 2 * temp[1].y - temp[2].y;
    temp[Num + 1].x = 2 * temp[Num].x - temp[Num - 1].x;
    temp[Num + 1].y = 2 * temp[Num].y - temp[Num - 1].y;
    CPosition NodePt1, NodePt2, NodePt3, NodePt4;
    double t;
    delete[] pt;
    pt = new CPosition[Num + InsertNumSum];
    int totalnum = 0;
    for (int i = 0; i < Num - 1; i++)
    {
        NodePt1 = temp[i];
        NodePt2 = temp[i + 1];
        NodePt3 = temp[i + 2];
        NodePt4 = temp[i + 3];
        double dt = 1.0 / (InsertNum[i] + 1);
        for (int j = 0; j < InsertNum[i] + 1; j++)
        {
            t = dt * j;
            pt[totalnum].x = F03(t) * NodePt1.x + F13(t) * NodePt2.x + F23(t) * NodePt3.x + F33(t) * NodePt4.x;
            pt[totalnum].y = F03(t) * NodePt1.y + F13(t) * NodePt2.y + F23(t) * NodePt3.y + F33(t) * NodePt4.y;
            totalnum++;
        }
        if (i == Num - 2)
        {
            t = 1;
            pt[totalnum].x = F03(t) * NodePt1.x + F13(t) * NodePt2.x + F23(t) * NodePt3.x + F33(t) * NodePt4.x;
            pt[totalnum].y = F03(t) * NodePt1.y + F13(t) * NodePt2.y + F23(t) * NodePt3.y + F33(t) * NodePt4.y;
            totalnum++;
        }
    }
    delete[] temp;
    Num = Num + InsertNumSum;
}

vector<planRoadToControl> insertTargetPoints(vector<CPosition> iPts)
{

    vector<planRoadToControl> ret;
    vector<CPosition> tmp;
    if (iPts.empty() || iPts.size() <= 1)
        return ret;
    tmp.emplace_back(iPts[0]);
    for (int i = 1; i < iPts.size() - 1; i++)
    {
        double angle1 = bearing(iPts[i - 1].x, iPts[i - 1].y, iPts[i].x, iPts[i].y);
        double angle2 = bearing(iPts[i].x, iPts[i].y, iPts[i + 1].x, iPts[i + 1].y);
        if (fabs(aTr(angle1 - angle2)) > 50)
        {
            CPosition t1 = getTargetLngLat(iPts[i].x, iPts[i].y, angle1, -12);
            CPosition t2 = getTargetLngLat(iPts[i].x, iPts[i].y, angle2, 12);
            if (distance(iPts[i - 1].x, iPts[i - 1].y, iPts[i].x, iPts[i].y) > 12)
                tmp.emplace_back(t1);
            tmp.emplace_back(iPts[i]);
            if (distance(iPts[i].x, iPts[i].y, iPts[i + 1].x, iPts[i + 1].y) > 12)
                tmp.emplace_back(t2);
        }
        else
        {
            tmp.emplace_back(iPts[i]);
        }
    }
    tmp.emplace_back(iPts.back());
    iPts.swap(tmp);

    int num = iPts.size();
    CPosition *testpt = new CPosition[num];
    for (int i = 0; i < num; i++)
    {
        testpt[i] = iPts[i];
    }
    int num2 = num;
    int *Intnum = new int[num - 1];
    for (int i = 0; i < num - 1; i++)
    {
        Intnum[i] = (int)(distance(testpt[i].x, testpt[i].y, testpt[i + 1].x, testpt[i + 1].y) / 0.1); //  ÿһ�����������ڲ���n����
    }

    ThreeOrderBSplineInterpolatePt(testpt, num2, Intnum); //  ����B��������

    vector<CPosition> newPts;
    for (int i = 0; i < num2 - 1; i++)
    {
        newPts.push_back(testpt[i]);
        double d = distance(testpt[i].x, testpt[i].y, testpt[i + 1].x, testpt[i + 1].y);
        if (d >= 0.5)
        {
            int n = d / 0.1;
            for (int j = 1; j < n; j++)
            {
                CPosition pt = interpolatePosition(CPosition(testpt[i].x, testpt[i].y), CPosition(testpt[i + 1].x, testpt[i + 1].y), n, j);
                newPts.emplace_back(pt);
            }
        }
    }

    for (int i = 0; i < newPts.size() - 1; i++)
    {
        planRoadToControl info;
        info.latitude = newPts[i].x;
        info.longitude = newPts[i].y;
        info.headingAngle = bearing(newPts[i].x, newPts[i].y, newPts[i + 1].x, newPts[i + 1].y);
        ret.emplace_back(info);
    }

    delete[] testpt;
    delete[] Intnum;
    return ret;
}

// 批量发送 8 帧 CAN 报文
bool batch_send_frames(int sockfd, struct can_frame frames[], int frame_count)
{
    if (sockfd < 0 || frames == NULL || frame_count <= 0 || frame_count > 8)
    {
        fprintf(stderr, "批量发送参数错误: sockfd=%d, frames=%p, count=%d\n",
                sockfd, frames, frame_count);
        return false;
    }

    // 使用sendmmsg批量发送多个CAN帧
    struct mmsghdr msgs[8];
    struct iovec iov[8];

    memset(msgs, 0, sizeof(msgs));
    memset(iov, 0, sizeof(iov));

    // 准备批量发送的数据结构
    for (int i = 0; i < frame_count; ++i)
    {
        iov[i].iov_base = &frames[i];
        iov[i].iov_len = sizeof(struct can_frame);

        msgs[i].msg_hdr.msg_iov = &iov[i];
        msgs[i].msg_hdr.msg_iovlen = 1;
    }

    // 执行批量发送
    int sent = sendmmsg(sockfd, msgs, frame_count, 0);

    if (sent < 0)
    {
        perror("sendmmsg失败");
        return false;
    }
    else if (sent != frame_count)
    {
        fprintf(stderr, "批量发送不完整: 已发送%d/%d帧\n", sent, frame_count);
        return false;
    }

    return true;
}

// 单帧发送函数 - 用于批量发送失败时的回退方案
bool send_single_frame(int sockfd, struct can_frame *frame)
{
    if (sockfd < 0 || frame == NULL)
    {
        return false;
    }

    int ret = write(sockfd, frame, sizeof(struct can_frame));
    return (ret == sizeof(struct can_frame));
}

/*
===================================程序：解析CAN报文===================================
author：lks
*/
// ID：4A2
VCU_VehDynStatus parseVCU_VehDynStatus(const uint8_t data[8]) // ID:4A2  车身动态状态
{
    VCU_VehDynStatus result;
    uint16_t rawSpd = (data[2] << 8) | data[3];
    result.vehicle_speed = rawSpd * 0.003906f;
    // std::cout << "车辆纵向速度：" << std::fixed << std::setprecision(2) << result.vehicle_speed << " km/h | ";
    int16_t rawRamp = ((data[5] & 0x0F) << 8) | data[4];
    if (rawRamp & 0x800)
    {
        rawRamp |= 0xF000; // 扩展符号位
    }
    result.Veh_Ramp = rawRamp * 0.05f;
    // std::cout << "车辆所处坡度：" << std::fixed << std::setprecision(2) << result.Veh_Ramp << " % | ";
    result.VehDynStat_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "循环计数：" << (int)result.VehDynStat_RollCnt << " | "<<endl;
    result.VehDyncStat_CheckSum = data[7];
    return result;
}

VCU_DriveStatus parseVCU_DriveStatus(const uint8_t data[8]) // ID:441  驱动状态
{
    VCU_DriveStatus result;
    result.Drv_RunDir = (data[0] >> 0) & 0x03;
    // std::cout << "车辆实际运动方向：";
    switch (result.Drv_RunDir)
    {
    case 0x0:
        // std::cout << "Invalid(无效)";
        break;
    case 0x1:
        // std::cout << "Forward(前进)";
        break;
    case 0x2:
        // std::cout << "Backward(后退)";
        break;
    case 0x3:
        // std::cout << "Stopped(停止)";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.Drv_DrvCtrlMode = (data[0] >> 2) & 0x03;
    // std::cout << "驱动控制模式：";
    switch (result.Drv_DrvCtrlMode)
    {
    case 0x0:
        // std::cout << "Invalid";
        break;
    case 0x1:
        // std::cout << "Speed Control";
        break;
    case 0x2:
        // std::cout << "Torque Control";
        break;
    case 0x3:
        // std::cout << "Acceleration Pedal Control";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.Drv_WorkMode = (data[0] >> 4) & 0x0F;
    result.Drv_GearAct = (data[1] >> 0) & 0x0F;
    // std::cout << "实际档位：";
    switch (result.Drv_GearAct)
    {
    case 0x0:
        // std::cout << "Invalid";
        break;
    case 0x1:
        // std::cout << "Parking(P)";
        break;
    case 0x3:
        // std::cout << "Reverse(R)";
        break;
    case 0x5:
        // std::cout << "Neutral(N)";
        break;
    case 0x9:
        // std::cout << "Drive(D)";
        break;
    default:
        // std::cout << "未知档位";
        break;
    }
    // std::cout << " | ";
    int16_t rawTorque = (data[2] << 8) | data[3];
    result.Drv_MotTq = rawTorque * 0.1f;
    // std::cout << "电机实际转矩：" << result.Drv_MotTq << " Nm | ";
    result.Drv_MotorSpeed = (data[4] << 8) | data[5];
    // std::cout << "电机实际转速：" << result.Drv_MotorSpeed << " rpm | ";
    result.DrvStat_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "循环计数：" << (int)result.DrvStat_RollCnt << " | ";
    result.Drv_ErrLevel = (data[6] >> 4) & 0x0F;
    result.DrvStat_CheckSum = data[7];
    return result;
}
VCU_BrakeStatus parseVCU_BrakeStatus(const uint8_t data[8]) // ID411 制动状态
{
    VCU_BrakeStatus result;
    result.Brk_BrkCtrlMode = (data[0] >> 0) & 0x03;
    // std::cout << "当前制动控制模式：";
    switch (result.Brk_BrkCtrlMode)
    {
    case 0x0:
        // std::cout << "Invalid";
        break;
    case 0x1:
        // std::cout << "Brake Pressure Mode";
        break;
    default:
        // std::cout << "未知模式";
        break;
    }
    // std::cout << " | ";
    result.Brk_WorkMode = (data[0] >> 4) & 0x0F;
    result.Brk_BrkPres = data[1];
    // std::cout << "当前实际制动压力：" << result.Brk_BrkPres << " bar | ";
    result.BrkStat_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "循环计数：" << (int)result.BrkStat_RollCnt << " | ";
    result.Brk_ErrLevel = (data[6] >> 4) & 0x0F;
    result.BrkStat_CheckSum = data[7];
    return result;
}

VCU_SteeringStatus parseVCU_SteeringStatus(const uint8_t data[8]) /// ID:431  转向状态
{
    VCU_SteeringStatus result;
    result.Str_StrCtrlMode = (data[0] >> 2) & 0x03;
    // std::cout << "自适转向控制模式：";
    switch (result.Str_StrCtrlMode)
    {
    case 0x0:
        // std::cout << "Invalid";
        break;
    case 0x1:
        // std::cout << "AngleControlMode";
        break;
    case 0x2:
        // std::cout << "DirectiveAngleControlMode";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.Str_WorkMode = (data[0] >> 4) & 0x0F;
    int16_t rawAngle = (data[2] << 8) | data[3];
    result.Str_StrWhlAngle = rawAngle * 0.05f;
    // std::cout << "左正右负：" << result.Str_StrWhlAngle << " deg | ";
    uint16_t rawAngleSpd = (data[4] << 8) | data[5];
    result.Str_StrWhlAngleSpd = rawAngleSpd * 0.1f - 1800;
    // std::cout << "实际方向盘转角速度：" << result.Str_StrWhlAngleSpd << " degree/s | ";
    result.StrStat_RollCnt = (data[6] >> 0) & 0x0F;
    result.Str_ErrLevel = (data[6] >> 4) & 0x0F;
    result.StrStat_CheckSum = data[7];
    return result;
}

VCU_BatStatus01 parseVCU_BatStatus01(const uint8_t data[8]) // ID:471  电池状态01
{
    VCU_BatStatus01 result;
    uint16_t rawLVVolt = (data[0] << 2) | (data[1] >> 6);
    result.LVBat_Volt = rawLVVolt * 0.1f;
    // std::cout << "12V低压电池电压：" << result.LVBat_Volt << " V | ";
    int16_t rawBatCur = ((data[1] & 0x3F) << 8) | data[2];
    if (rawBatCur & 0x2000)
    {
        rawBatCur |= 0xC000; // 扩展符号位
    }
    result.Bat_BatCur = rawBatCur * 0.1f;
    // std::cout << "动力电池电流：" << result.Bat_BatCur << " A | ";
    uint16_t rawBatVolt = (data[3] << 8) | data[4];
    result.Bat_BatVolt = rawBatVolt * 0.1f;
    // std::cout << "动力电池电压：" << result.Bat_BatVolt << " V | ";
    int8_t rawBatTemp = data[5];
    result.Bat_BatTemp = rawBatTemp - 50; // 偏移量为-50
    // std::cout << "动力电池温度：" << result.Bat_BatTemp << " °C | ";
    result.BatStat1_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "循环计数：" << (int)result.BatStat1_RollCnt << " | ";
    result.Bat_ChrgSt = (data[6] >> 6) & 0x03;
    // std::cout << "动力电池充电状态：";
    switch (result.Bat_ChrgSt)
    {
    case 0x0:
        // std::cout << "Not Chrg(未充电)";
        break;
    case 0x1:
        // std::cout << "Charging(充电中)";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.BatStat1_CheckSum = data[7];
    return result;
}
VCU_BatStatus02 parseVCU_BatStatus02(const uint8_t data[8]) // ID:473  电池状态02
{
    VCU_BatStatus02 result;
    uint16_t rawSOC = (data[1] >> 4) | (data[0] << 4);
    result.Bat_BatSOC = rawSOC * 0.1f;
    // std::cout << "动力电池SOC：" << result.Bat_BatSOC << " % | ";
    uint16_t rawSOH = ((data[1] & 0x0F) << 8) | data[2];
    result.Bat_BatSOH = rawSOH * 0.1f;
    // std::cout << "动力电池SOH：" << result.Bat_BatSOH << " % | ";
    result.BatStat2_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "循环计数：" << (int)result.BatStat2_RollCnt << " | ";
    result.Bat_ErrLevel = (data[6] >> 4) & 0x0F;
    result.BatStat2_CheckSum = data[7];
    return result;
}
VCU_EmrgStatus parseVCU_EmrgStatus(const uint8_t data[8]) // ID:451  紧急状态
{
    VCU_EmrgStatus result;
    result.Emrg_Sw_St = (data[0] >> 0) & 0x03;
    // std::cout << "物理急停状态：";
    switch (result.Emrg_Sw_St)
    {
    case 0x0:
        // std::cout << "Released(功能开关关闭)";
        break;
    case 0x1:
        // std::cout << "Enabled(功能开关打开)";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.RrCrashTrg_St = (data[0] >> 4) & 0x01;
    // std::cout << "后碰撞触发状态：";
    // if(result.RrCrashTrg_St == 0x0)
    // std::cout << "False(未触发)";
    // else
    // std::cout << "True(触发)";
    // std::cout << " | ";
    result.FrCrashTrg_St = (data[0] >> 5) & 0x01;
    // std::cout << "前碰撞触发状态：";
    // if(result.FrCrashTrg_St == 0x0)
    // std::cout << "False(未触发)";
    // else
    // std::cout << "True(触发)";
    // std::cout << " | ";
    result.LeCrashTrg_St = (data[0] >> 6) & 0x01;
    // std::cout << "左碰撞触发状态：";
    // if(result.LeCrashTrg_St == 0x0)
    // std::cout << "False(未触发)";
    // std::cout << "True(触发)";
    // std::cout << " | ";
    result.RiCrashTrg_St = (data[0] >> 7) & 0x01;
    // std::cout << "右碰撞触发状态：";
    // if(result.RiCrashTrg_St == 0x0)
    // std::cout << "False(未触发)";
    // else
    //     std::cout << "True(触发)";
    // std::cout << " | ";

    result.Emrg_VehEmrgStopErr = (data[1] >> 0) & 0x01;
    // std::cout << "物理急停触发故障：";
    // if(result.Emrg_VehEmrgStopErr == 0x0)
    // std::cout << "Normal(正常)";
    // else
    // std::cout << "Fault(故障)";
    // std::cout << " | ";
    result.Emrg_ADLEmrgStopErr = (data[1] >> 1) & 0x01;
    // std::cout << "远程急停触发故障：";
    // if(result.Emrg_ADLEmrgStopErr == 0x0)
    // std::cout << "Normal(正常)";
    // else
    // std::cout << "Fault(故障)";
    // std::cout << " | ";
    result.Emrg_BckCrashSwErr = (data[1] >> 4) & 0x01;
    // std::cout << "后触边碰撞故障：";
    // if(result.Emrg_BckCrashSwErr == 0x0)
    // std::cout << "Normal(正常)";
    // else
    // std::cout << "Fault(故障)";
    // std::cout << " | ";
    result.Emrg_FrntCrashSwErr = (data[1] >> 5) & 0x01;
    // std::cout << "前触边碰撞故障：";
    // if(result.Emrg_FrntCrashSwErr == 0x0)
    // std::cout << "Normal(正常)";
    // else
    //     std::cout << "Fault(故障)";
    // std::cout << " | ";
    result.Emrg_LeftCrashSwErr = (data[1] >> 6) & 0x01;
    // std::cout << "左触边碰撞故障";
    // if(result.Emrg_LeftCrashSwErr == 0x0)
    // std::cout << "Normal(正常)";
    // else
    //     std::cout << "Fault(故障)";
    // std::cout << " | ";
    result.Emrg_RightCrashSwErr = (data[1] >> 7) & 0x01;
    // std::cout << "右触边碰撞故障：";
    // if(result.Emrg_RightCrashSwErr == 0x0)
    // std::cout << "Normal(正常)";
    // else
    //     std::cout << "Fault(故障)";
    // std::cout << " | ";
    result.Emrg_EmrgyCmdOfflErr = (data[2] >> 3) & 0x01;
    // std::cout << "紧急控制指令报文掉线：";
    // if(result.Emrg_EmrgyCmdOfflErr == 0x0)
    // std::cout << "Normal(正常)";
    // else
    // std::cout << "Fault(故障)";
    // std::cout << " | ";
    result.Emrg_BckSlipWarn = (data[3] >> 0) & 0x03;
    // std::cout << "溜车警告：";
    switch (result.Emrg_BckSlipWarn)
    {
    case 0x0:
        // std::cout << "Normal(正常)";
        break;
    case 0x1:
        // std::cout << "1级";
        break;
    case 0x2:
        // std::cout << "2级";
        break;
    case 0x3:
        // std::cout << "3级";
        break;
    default:
        // std::cout << "error";
        break;
    }
    // std::cout << " | ";
    result.EmrgStat_RollCnt = (data[6] >> 0) & 0x0F;
    // std::cout << "循环计数：" << (int)result.EmrgStat_RollCnt << " | ";
    result.Emrg_ErrLevel = (data[6] >> 4) & 0x0F;
    result.EmrgStat_CheckSum = data[7];
    return result;
}
int create_udp_socket(int port)
{
    int sockfd;
    struct sockaddr_in addr;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    return sockfd;
}

//=================================================================================

// 初始化CAN socket封装
// 初始化CAN socket，程序启动时调用一次
int init_can_socket(const char *ifname)
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    int sockfd;

    // 创建socket
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0)
    {
        perror("socket");
        return -1;
    }

    // 获取接口索引
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        close(sockfd);
        return -1;
    }

    // 绑定socket到CAN接口
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        close(sockfd);
        return -1;
    }

    // 设置为非阻塞模式
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    return sockfd;
}
// 定时器信号处理函数，只设置标志及udp监控计数
void handle_sigalrm(int sig)
{
    // std::cout << "SIGALRM: set flag_canSend\n";
    flag_canSend = true;
    // udp监测计数：监控超过1秒如果没收到udb决策指令则停车，实际使用时打开，测试时注释掉 test
    if (udp_monitor > 50) {
        begin_run = 0;
        fprintf(stderr, "udp Error\n");
    } else {
        udp_monitor += 1;
    }
}

// 从文件中读取经纬度数据并存入vector
void readMapDataFromFile(const string &filePath)
{
    ifstream file(filePath);
    string line;

    if (!file.is_open())
    {
        cerr << "Error opening file: " << filePath << endl;
        return;
    }

    // 清空之前的数据
    map_latitude_v.clear();
    map_longitude_v.clear();

    // 逐行读取文件
    while (getline(file, line))
    {
        // 查找Latitude和Longitude的起始位置
        size_t lat_pos = line.find("Latitude:");
        size_t lon_pos = line.find("Longitude:");

        if (lat_pos != string::npos && lon_pos != string::npos)
        {
            try
            {
                // 提取Latitude和Longitude的值
                size_t lat_start = lat_pos + 9;  // "Latitude: "后的位置
                size_t lon_start = lon_pos + 10; // "Longitude: "后的位置

                // 寻找Latitude和Longitude的结束位置
                size_t lat_end = line.find(",", lat_start);
                size_t lon_end = line.find(",", lon_start);

                if (lat_end == string::npos)
                    lat_end = line.find(" ", lat_start);
                if (lon_end == string::npos)
                    lon_end = line.find(" ", lon_start);

                // 截取纬度和经度并转换为double
                double latitude = stod(line.substr(lat_start, lat_end - lat_start));
                double longitude = stod(line.substr(lon_start, lon_end - lon_start));

                // 存入vector
                map_latitude_v.push_back(latitude);
                map_longitude_v.push_back(longitude);

                // 输出保留8位精度的经纬度
                cout << fixed << setprecision(8)
                     << "Latitude: " << latitude << ", Longitude: " << longitude << endl;
            }
            catch (const exception &e)
            {
                cerr << "Error parsing line: " << line << " Error: " << e.what() << endl;
            }
        }
        else
        {
            cerr << "Skipping invalid line: " << line << endl;
        }
    }

    file.close();
    cout << "Successfully read " << map_latitude_v.size() << " GIS_map points." << endl;
}

// 解析 $GPCHC 报文并提取所需字段
bool parseGPCHC(const std::string &sentence, double &latitude, double &longitude, double &heading, int &status)
{
    if (sentence.find("$GPCHC") != 0)
    {
        return false;
    }

    std::vector<std::string> fields;
    std::stringstream ss(sentence);
    std::string field;

    while (std::getline(ss, field, ','))
    {
        fields.push_back(field);
    }

    if (fields.size() < 24)
    {
        std::cerr << "Incomplete $GPCHC sentence: " << sentence << std::endl;
        return false;
    }
    try
    {
        {
            heading = std::stod(fields[3]);    // 偏航角
            latitude = std::stod(fields[12]);  // 纬度
            longitude = std::stod(fields[13]); // 经度
            status = std::stoi(fields[21]);    // 状态码
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error parsing $GPCHC sentence: " << e.what() << std::endl;
        return false;
    }

    return true;
}

// 配置串口
int configureSerialPort(const std::string &port)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY); // 去掉 O_NDELAY，使用阻塞模式
    if (fd == -1)
    {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return -1;
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "Error getting serial attributes: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 数据位：8
    tty.c_cflag &= ~PARENB;                     // 无校验位
    tty.c_cflag &= ~CSTOPB;                     // 停止位：1
    tty.c_cflag |= CREAD | CLOCAL;              // 启用接收器，忽略调制解调器线路状态

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 禁用规范模式
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // 禁用软件流控制
    tty.c_oflag &= ~OPOST;                          // 输出原始模式

    tty.c_cc[VMIN] = 1;  // 最小读取字符数
    tty.c_cc[VTIME] = 1; // 超时：0.1秒

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error setting serial attributes: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    // 清空串口缓冲区
    tcflush(fd, TCIOFLUSH); // 清空输入和输出缓冲区
    std::cout << "Serial port buffer flushed successfully." << std::endl;

    return fd;
}

// 读取实时GPS点 以下二选一
//*********************
/*华测CGI610*/
/*
std::vector<double> readSerialData(int fd) {
    char buffer[256];
    std::string sentence;
    double latitude = 0.0, longitude = 0.0, heading = 0.0;
    int status = 0;
    std::vector<double> res;

    // 设置输出精度：纬度和经度保留小数点后 8 位
    std::cout << std::fixed << std::setprecision(8);

    while (true) {
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = read(fd, buffer, sizeof(buffer) - 1);

        if (bytesRead < 0) {
            if (errno == EAGAIN) {
                continue; // 无数据时继续等待
            }
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
            break;
        }
        else if (bytesRead > 0) {
            sentence += std::string(buffer, bytesRead);

            // 查找报文头 "$GPCHC"
            size_t startPos = sentence.find("$GPCHC");
            if (startPos != std::string::npos) {
                // 如果找到了 "$GPCHC"，我们就认为这是一个有效的报文开始
                size_t endPos = sentence.find("\n", startPos); // 查找报文的结尾（换行符）
                if (endPos != std::string::npos) {
                    std::cout << "---------------------" << endl;
                    // 提取报文内容（去除起始的"$GPCHC"和末尾的换行符）
                    std::string fullSentence = sentence.substr(startPos, endPos - startPos);
                    sentence.erase(0, endPos + 1); // 删除已经处理过的部分

                    // 解析并提取数据
                    if (parseGPCHC(fullSentence, latitude, longitude, heading, status)) {
                        // 实时更新经纬度变量并打印
                        std::cout << "Updated Latitude: " << latitude
                                  << ", Longitude: " << longitude
                                  << ", Heading: " << heading
                                  << ", Status: " << status << std::endl;
                        res.clear();
                        res.push_back(latitude);
                        res.push_back(longitude);
                        res.push_back(heading);
                        res.push_back((double)status);

                        return res; // 返回经纬度数据
                    } else {
                        std::cerr << "解析错误：Invalid $GPCHC sentence: " << fullSentence << std::endl;
                        res.at(0) = 999.99;

                        return res; // 返回初始值
                    }
                }
            }
        }
    }
}


*/
#if 0
//INS821
std::vector<double> parseData(std::vector<uint8_t>& data) {
    double headingAngle = 0.0, longitudeData = 0.0, latitudeData = 0.0;
    int status = -1;
    std::vector<double> res(4, 0.0);  // 创建一个包含4个元素的vector，分别存储latitude, longitude, heading_angle, GPS_status

    // 查找帧头 BD DB 0B (根据协议图片)
    size_t frameStartIndex = 0;
    bool frameFound = false;
    
    // 在数据中查找帧头序列 BD DB 0B
    for (size_t i = 0; i < data.size() - 2; i++) {
        if (data[i] == 0xBD && data[i+1] == 0xDB && data[i+2] == 0x0B) {
            frameStartIndex = i;
            frameFound = true;
            break;
        }
    }
    
    // 如果找到帧头并且后面有足够的数据(至少92字节)
    if (frameFound && (frameStartIndex + 92) <= data.size()) {
        // 提取完整的92字节数据帧
        std::vector<uint8_t> frameData(data.begin() + frameStartIndex, data.begin() + frameStartIndex + 92);
        
        // 解析航向角 (偏移量3，横滚角，根据协议图片)
        // 根据协议图片，横滚角在偏移量3处，长度为2字节，LSB_first
        if (frameData.size() > 8) {
            uint16_t headingRaw = (frameData[8] << 8) | frameData[7];  // LSB_first格式
            headingAngle = (headingRaw * 360.0) / 32768.0;  // 乘以系数 (360/32768)
        }
        
        // 解析纬度 (偏移量21，根据协议图片)
        // 根据协议图片，纬度在偏移量21处，长度为8字节，LSB_first
        if (frameData.size() > 28) {
            uint64_t latitudeRaw = (static_cast<uint64_t>(frameData[28]) << 56) |
                                  (static_cast<uint64_t>(frameData[27]) << 48) |
                                  (static_cast<uint64_t>(frameData[26]) << 40) |
                                  (static_cast<uint64_t>(frameData[25]) << 32) |
                                  (static_cast<uint64_t>(frameData[24]) << 24) |
                                  (static_cast<uint64_t>(frameData[23]) << 16) |
                                  (static_cast<uint64_t>(frameData[22]) << 8) |
                                  static_cast<uint64_t>(frameData[21]);  // LSB_first 拼接

            latitudeData = latitudeRaw * 1.00E-08;  // 乘以系数 (1.00E-08)
        }
        
        // 解析经度 (偏移量29，根据协议图片)
        // 根据协议图片，经度在偏移量29处，长度为8字节，LSB_first
        if (frameData.size() > 36) {
            uint64_t longitudeRaw = (static_cast<uint64_t>(frameData[36]) << 56) |
                                   (static_cast<uint64_t>(frameData[35]) << 48) |
                                   (static_cast<uint64_t>(frameData[34]) << 40) |
                                   (static_cast<uint64_t>(frameData[33]) << 32) |
                                   (static_cast<uint64_t>(frameData[32]) << 24) |
                                   (static_cast<uint64_t>(frameData[31]) << 16) |
                                   (static_cast<uint64_t>(frameData[30]) << 8) |
                                   static_cast<uint64_t>(frameData[29]);  // LSB_first 拼接
            longitudeData = longitudeRaw * 1.00E-08;  // 乘以系数 (1.00E-08)
        }
        
        // 解析GPS状态 (偏移量47，根据协议图片)
        // 根据协议图片，状态在偏移量47处，长度为1字节
        if (frameData.size() > 47) {
            status = frameData[47];  // 直接获取状态字节
        }
        
        // 将解析结果存入vector
        res[0] = latitudeData;
        res[1] = longitudeData;
        res[2] = headingAngle;
        res[3] = static_cast<double>(status);
        
        // 打印解析后的信息
        std::cout << "Parsed Info: "
                  << "Latitude: " << std::fixed << std::setprecision(8) << latitudeData
                  << ", Longitude: " << std::fixed << std::setprecision(8) << longitudeData
                  << ", Heading: " << std::fixed << std::setprecision(8) << headingAngle
                  << ", Status: " << status << std::endl;
    } else {
        if (!frameFound) {
            std::cout << "Error: Frame header (BD DB 0B) not found." << std::endl;
        } else {
            std::cout << "Error: Incomplete GPS message after frame header." << std::endl;
        }
    }
    
    return res;
}
std::vector<double> readSerialData(int serialPort) {
    static std::vector<uint8_t> dataBuffer;             // 缓冲区，用于存放串口接收到的原始数据
    static std::vector<double> lastParsedFrame(0, 0.0); // 上一帧解析值（经纬度、航向角、状态）
    static auto lastReadTime = std::chrono::steady_clock::now();
    static int emptyFrameCount = 0;
    static int totalFrameCount = 0;
    
    // 计算距离上次读取的时间间隔
    auto currentTime = std::chrono::steady_clock::now();
    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastReadTime).count();
    
    // 每次读取前清空串口缓冲区，确保读取最新数据
    tcflush(serialPort, TCIFLUSH);
    
    // 读取串口数据
    char buffer[1024];
    ssize_t bytesRead = read(serialPort, buffer, sizeof(buffer));
    
    // 更新读取时间
    lastReadTime = currentTime;
    
    if (bytesRead > 0) {
        std::cout << "Read " << bytesRead << " bytes from serial port" << std::endl;
        for (ssize_t i = 0; i < bytesRead; ++i) {
            dataBuffer.push_back(static_cast<uint8_t>(buffer[i]));
        }

        // 限制缓冲区大小，避免无限增长
        if (dataBuffer.size() > 1000) {
            dataBuffer.erase(dataBuffer.begin(), dataBuffer.begin() + (dataBuffer.size() - 1000));
        }
    } else {
        std::cout << "No data read from serial port" << std::endl;
    }

    // 查找并解析最新完整帧（92字节）
    bool frameFound = false;
    for (size_t i = 0; i + 92 <= dataBuffer.size(); ++i) {
        if (dataBuffer[i] == 0xBD && dataBuffer[i+1] == 0xDB && dataBuffer[i+2] == 0x0B) {
            std::vector<uint8_t> frame(dataBuffer.begin() + i, dataBuffer.begin() + i + 92);
            std::vector<double> parsed = parseData(frame);
            
            totalFrameCount++;
            
            // 更新缓存帧
            lastParsedFrame = parsed;

            // 清除已处理的数据
            dataBuffer.erase(dataBuffer.begin(), dataBuffer.begin() + i + 92);
            
            frameFound = true;
            std::cout << "Found valid GPS frame, total frames: " << totalFrameCount << std::endl;
            break;
        }
    }
    
    if (!frameFound) {
        emptyFrameCount++;
        std::cout << "No valid GPS frame found, empty count: " << emptyFrameCount 
                  << ", empty rate: " << (double)emptyFrameCount / (totalFrameCount + 1) * 100 << "%" << std::endl;
    }

    // 如果没有新帧，返回上一帧解析值
    return lastParsedFrame;
}
#elif 0
// 华测GPS
std::vector<double> readSerialData(int fd)
{
    char buffer[256];
    std::string sentence;
    double latitude = 0.0, longitude = 0.0, heading = 0.0;
    int status = 0;
    std::vector<double> res;

    // 设置输出精度：纬度和经度保留小数点后 8 位
    std::cout << std::fixed << std::setprecision(8);

    int maxRetries = 1000; // 最大重试次数
    int retryCount = 0;

    while (true)
    {
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = read(fd, buffer, sizeof(buffer) - 1);

        if (bytesRead < 0)
        {
            if (errno == EAGAIN)
            {
                retryCount++;
                if (retryCount >= maxRetries)
                {
                    std::cerr << "Max retries reached. Exiting." << std::endl;
                    break; // 达到最大重试次数退出
                }
                continue; // 无数据时继续等待
            }
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
            break;
        }
        else if (bytesRead > 0)
        {
            retryCount = 0; // 重置重试计数
            sentence += std::string(buffer, bytesRead);

            // 查找报文头 "$GPCHC"
            size_t startPos = sentence.find("$GPCHC");
            if (startPos != std::string::npos)
            {
                // 如果找到了 "$GPCHC"，我们就认为这是一个有效的报文开始
                size_t endPos = sentence.find("\n", startPos); // 查找报文的结尾（换行符）
                if (endPos != std::string::npos)
                {
                    // 提取报文内容（去除起始的"$GPCHC"和末尾的换行符）
                    std::string fullSentence = sentence.substr(startPos, endPos - startPos);
                    sentence.erase(0, endPos + 1); // 删除已经处理过的部分

                    // 解析并提取数据
                    if (parseGPCHC(fullSentence, latitude, longitude, heading, status))
                    {
                        // 实时更新经纬度变量并打印
                        std::cout << "Updated Latitude: " << latitude
                                  << ", Longitude: " << longitude
                                  << ", Heading: " << heading
                                  << ", Status: " << status << std::endl;

                        res.push_back(latitude);
                        res.push_back(longitude);
                        res.push_back(heading);
                        res.push_back((double)status);

                        return res; // 返回经纬度数据
                    }
                    else
                    {
                        // 输出解析失败的详细报文
                        std::cerr << "解析错误：Invalid $GPCHC sentence: " << fullSentence << std::endl;
                        res.push_back(999.99); // 添加一个默认值（避免越界）
                        return res;            // 返回初始值
                    }
                }
            }
            else
            {
                std::cerr << "未找到报文头：Invalid data or missing expected header '$GPCHC'. Full sentence: " << sentence << std::endl;
            }
        }
    }

    // 如果循环结束，表示没有找到有效数据
    std::cerr << "No valid data received. Exiting..." << std::endl;
    return res; // 返回一个空的结果
}
#elif 1
// poly 3000P
//  从 GPNAV 字符串中解析所需四个值
//  返回 true 表示解析成功
bool parseGPNAV(const std::string &sentence,
                double &latitude,
                double &longitude,
                double &heading,
                int &status)
{
    // 去掉起始的"$"和末尾的校验段（如"*41"）
    size_t asterisk = sentence.find('*');
    std::string body = sentence.substr(1, (asterisk == std::string::npos ? sentence.size() : asterisk) - 1);

    std::vector<std::string> tokens;
    std::istringstream ss(body);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        tokens.push_back(item);
    }

    // GPNAV 一共有 24+ 个字段，至少要到 index 23
    if (tokens.size() < 24 || tokens[0] != "GPNAV")
    {
        return false;
    }

    try
    {
        // 字段索引与图片对应：
        // tokens[3]  偏航角 (heading)
        // tokens[12] 纬度
        // tokens[13] 经度
        // tokens[23] 工作状态 (1 警告, 2 正常)
        heading = std::stod(tokens[3]);
        latitude = std::stod(tokens[12]);
        longitude = std::stod(tokens[13]);
        status = std::stoi(tokens[23]);
    }
    catch (...)
    {
        return false;
    }

    return true;
}

std::vector<double> readSerialData(int fd)
{
    char buffer[256];
    std::string sentence_acc;
    double latitude = 0.0, longitude = 0.0, heading = 0.0;
    int status = 0;
    std::vector<double> res;

    // 设置输出精度（仅用于调试打印）
    std::cout << std::fixed << std::setprecision(8);

    const int maxRetries = 1000;
    int retryCount = 0;

    while (true)
    {
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = read(fd, buffer, sizeof(buffer) - 1);

        if (bytesRead < 0)
        {
            if (errno == EAGAIN && retryCount++ < maxRetries)
            {
                continue;
            }
            std::cerr << "Serial read error: " << strerror(errno) << std::endl;
            break;
        }
        if (bytesRead == 0)
        {
            if (++retryCount >= maxRetries)
            {
                std::cerr << "No data after retries. Exiting.\n";
                break;
            }
            continue;
        }
        retryCount = 0;
        sentence_acc.append(buffer, bytesRead);

        // 查找 $GPNAV 开头
        size_t start = sentence_acc.find("$GPNAV");
        if (start == std::string::npos)
        {
            // 扔掉过长的无效前缀，避免内存无限增长
            if (sentence_acc.size() > 1024)
                sentence_acc.erase(0, sentence_acc.size() - 512);
            continue;
        }

        // 找到换行或回车作为报文结束标志
        size_t end = sentence_acc.find_first_of("\r\n", start);
        if (end == std::string::npos)
        {
            // 报文还没完全到，等下一次读
            continue;
        }

        // 提取一整条报文
        std::string fullSentence = sentence_acc.substr(start, end - start);
        // 删除已处理部分
        sentence_acc.erase(0, end + 1);

        // 解析
        if (parseGPNAV(fullSentence, latitude, longitude, heading, status))
        {
            std::cout << "Lat: " << latitude
                      << ", Lon: " << longitude
                      << ", Head: " << heading
                      << ", Status: " << status
                      << std::endl;

            res.push_back(latitude);
            res.push_back(longitude);
            res.push_back(heading);
            res.push_back(static_cast<double>(status));
            return res;
        }
        else
        {
            std::cerr << "Invalid GPNAV sentence: " << fullSentence << std::endl;
            // 如果解析失败，可返回一组特殊值或继续循环
            res = {0, 0, 0, 0};
            return res;
        }
    }

    // 若退出循环未成功，返回空 vector
    return res;
}
#endif

//**********************

// 0.角度计算calculate_delta_angle
double calculate_delta_angle(double angle1, double angle2)
{
    while (angle1 > 360)
    {
        angle1 -= 360;
    }
    while (angle1 < 0)
    {
        angle1 += 360;
    }
    while (angle2 > 360)
    {
        angle2 -= 360;
    }
    while (angle2 < 0)
    {
        angle2 += 360;
    }
    double delta = angle1 - angle2;
    if (delta > 180)
    {
        delta = delta - 360;
    }
    else if (delta < -180)
    {
        delta = 360 + delta;
    }
    return delta;
}
// 3.航向角计算
double Azimuth_withGeo(double lat1, double lon1, double lat2, double lon2)
{
    double s12(0), azi1(0), azi2(0);
    const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
    geod.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2);
    // cout << "geodesic lib:" << azi1 << endl;
    // cout << "c++:" << bearing(lat1, lon1, lat2, lon2) << endl;
    return azi1;
} 
//原始计算LD
double calc_Lf(double v)
{
    double k = 2.9, b = 0;
    double Ld = k * v + b;
    if (Ld < 3.5)
    {
        Ld = 3.5;
    }

    // Ld = 2.85; //5kph
    // Ld = 5.7; //10kph 15kph
    return Ld;
} 


// 1.计算距离（起点与终点经纬度）
double Distance_withGeo(double lat1, double lon1, double lat2, double lon2)
{
    double s12(0);
    const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
    geod.Inverse(lat1, lon1, lat2, lon2, s12);
    return s12;
}

int findWayPoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y)
{
    int x_size = maps_x.size(), y_size = maps_y.size();
    int n = x_size <= y_size ? x_size : y_size;
    std::vector<double> disList(n, 0);
    for (int i = 0; i != n; ++i)
    {
        disList.at(i) = Distance_withGeo(x, y, maps_x.at(i), maps_y.at(i));
    }

    int closest = min_element(disList.begin(), disList.end()) - disList.begin();
    return closest;
}

// 2.查找地图相关点
// cx, cy -- 路径经、纬度容器
int calc_target_index(double lat, double lon, double Ld, vector<double> cx, vector<double> cy, int &Map_ind)
{
    Map_ind = findWayPoint(lat, lon, cx, cy);
    cout << "车辆当前点下标: " << std::dec << Map_ind << "/" << std::dec << cx.size() << endl;
    double L = 0;
    while ((Ld > L) && ((Map_ind + 1) < cx.size()))
    {
        L += Distance_withGeo(cx[Map_ind + 1], cy[Map_ind + 1], cx[Map_ind], cy[Map_ind]);
        Map_ind += 1;
    }
    return Map_ind;
}
//原有算法
double obtainMapData()
{
    double Ld;
    int ind;
    
    // 检查地图数据是否为空
    if (map_latitude_v.empty() || map_longitude_v.empty()) {
        std::cout << "警告：地图数据为空，无法进行路径规划" << std::endl;
        return 3.5; // 返回默认预瞄距离
    }
    
    // 原版预瞄距离计算
    Ld = calc_Lf(cur_params.vehicle_speed / 3.6);
 
    // 基于最近点计算预瞄点
    ind = calc_target_index(cur_params.latitude, cur_params.longitude, Ld, map_latitude_v, map_longitude_v, Map_ind);
    std::cout << "计算的预瞄点索引: " << ind << "/" << map_latitude_v.size() << std::endl;
 
    if (_target_ind >= ind)
    {
        ind = _target_ind;
    }

    if (ind < map_latitude_v.size())
    {
        GIS_map.latitude = map_latitude_v[ind];
        GIS_map.longitude = map_longitude_v[ind];
        // GIS_map.heading_angle = map_heading_angle_v[ind];
        std::cout << "使用计算的预瞄点: (" << GIS_map.latitude << ", " << GIS_map.longitude << ")" << std::endl;
    }
    else
    {
        // 如果预瞄点超出地图范围，使用地图的最后一个点，但输出警告
        GIS_map.latitude = map_latitude_v.back();
        GIS_map.longitude = map_longitude_v.back();
        // GIS_map.heading_angle = map_heading_angle_v.back();
        ind = map_latitude_v.size() - 1;
        std::cout << "警告：预瞄点超出地图范围，使用最后一个点: (" << GIS_map.latitude << ", " << GIS_map.longitude << ")" << std::endl;
    }

    _target_ind = ind;
    return Ld;
} 

/**
 * @brief 修改后的obtainMapData函数
 * @details 集成优化预瞄距离计算，保持原有接口兼容性
 */
double obtainMapData_optimized()
{
    double Ld;
    int ind;
    
    // 检查地图数据是否为空
    if (map_latitude_v.empty() || map_longitude_v.empty()) {
        std::cout << "警告：地图数据为空，无法进行路径规划" << std::endl;
        return 3.5; // 返回默认预瞄距离
    }
    
    if (use_optimized_lookahead) {
        // 使用优化算法计算预瞄距离
        
        // 准备路径点数据
        std::vector<std::pair<double, double>> path_points;
        for (size_t i = 0; i < map_latitude_v.size() && i < map_longitude_v.size(); ++i) {
            path_points.push_back(std::make_pair(map_latitude_v[i], map_longitude_v[i]));
        }
        
        // 找到当前最近点索引
        Map_ind = findWayPoint(cur_params.latitude, cur_params.longitude, 
                              map_latitude_v, map_longitude_v);
        
        // 使用优化算法计算预瞄距离
        Ld = calc_Lf_optimized(
            cur_params.vehicle_speed,           // 车辆速度 (km/h)
            cur_params.latitude,                // 当前纬度
            cur_params.longitude,               // 当前经度
            cur_params.heading_angle,           // 当前航向角
            path_points,                        // 路径点集合
            Map_ind                             // 当前路径点索引
        );


        std::cout << "使用优化算法计算预瞄距离: " << Ld << "m" << std::endl;
    } else {
        // 使用原有算法
        Ld = calc_Lf(cur_params.vehicle_speed / 3.6);
        std::cout << "使用原有算法计算预瞄距离: " << Ld << "m" << std::endl;
    }

    // 基于最近点计算预瞄点
    ind = calc_target_index(cur_params.latitude, cur_params.longitude, Ld, 
                           map_latitude_v, map_longitude_v, Map_ind);
    std::cout << "计算的预瞄点索引: " << ind << "/" << map_latitude_v.size() << std::endl;
    if (_target_ind >= ind) {
        ind = _target_ind;
    }       

    if (ind < map_latitude_v.size())
    {
        GIS_map.latitude = map_latitude_v[ind];
        GIS_map.longitude = map_longitude_v[ind];
        std::cout << "使用计算的预瞄点: (" << GIS_map.latitude << ", " << GIS_map.longitude << ")" << std::endl;
    }
    else
    {
        // 如果预瞄点超出地图范围，使用地图的最后一个点，但输出警告
        GIS_map.latitude = map_latitude_v.back();
        GIS_map.longitude = map_longitude_v.back();
        ind = map_latitude_v.size() - 1;
        std::cout << "警告：预瞄点超出地图范围，使用最后一个点: (" << GIS_map.latitude << ", " << GIS_map.longitude << ")" << std::endl;
    }

    _target_ind = ind;
    return Ld;
}


/**
 * @brief 初始化优化器参数的函数
 * @details 在程序启动时调用，设置车辆参数和算法参数
 */
void initializeLookaheadOptimizer()
{
    // 设置车辆硬件参数
    double wheelbase = 2.0;              // 轴距 (m)
    double max_steering_angle = 33.0;    // 最大转向角 (度)
    
    // 设置算法参数
    // 基础预瞄增益系数:类似原系数K-大会甩出弯/小会切弯
    // double base_gain = 1.35;            //18 右         
    // double base_gain = 1.1596;             //18 左
    double base_gain = 2.9;             //6 

    double min_lookahead = 2.5;          // 最小预瞄距离 (m)
    double max_lookahead = 20.0;         // 最大预瞄距离 (m)
  
    
    // 优化权重参数
    double tracking_weight = 2.3;        // 跟踪精度权重
    double stability_weight = 1.7;       // 稳定性权重
    double smoothness_weight = 0.7;      // 平滑性权重
    
    // 初始化优化器 - 传递所有必要参数
    setLookaheadOptimizerParams(wheelbase, max_steering_angle, 
                               base_gain, min_lookahead, max_lookahead,
                               tracking_weight, stability_weight, smoothness_weight);
    
    std::cout << "预瞄距离优化器初始化完成" << std::endl;
    std::cout << "车辆参数: 轴距=" << wheelbase << "m, 最大转向角=" << max_steering_angle << "度" << std::endl;
    std::cout << "算法参数: 基础增益=" << base_gain << ", 最小Ld=" << min_lookahead 
              << "m, 最大Ld=" << max_lookahead << "m" << std::endl;
    std::cout << "优化权重: 跟踪精度=" << tracking_weight << ", 稳定性=" << stability_weight 
              << ", 平滑性=" << smoothness_weight << std::endl;
}

/**
 * @brief 切换预瞄距离计算算法
 * @param use_optimized 是否使用优化算法
 */
void switchLookaheadAlgorithm(bool use_optimized)
{
    use_optimized_lookahead = use_optimized;
    std::cout << "切换预瞄距离算法: " << (use_optimized ? "obtainMapData_optimized" : "obtainMapData") << std::endl;
}



// vehicle_speed, heading_angle -- 当前速度、航向角
// lat,lon -- 当前坐标
// map_lat,lon -- 目标坐标
// map_heading_angle_v--目标航向角
// wheelbase--轴距
// 预瞄距离（Ld）越长，控制效果会越平滑，预瞄距离越短，控制效果会越精确（同时也会带来一定的震荡）
//  GPS2Steer函数计算转向角并返回
double GPS2Steer(double vehicle_speed, double latitude, double longitude, double heading_angle,
                 double map_latitude_v, double map_longitude_v, double wheelbase, double &Ld, double &alpha)
{
    double steer_value = 0;

    // printf("\n预瞄车速 vehicle_speed = %f\n",vehicle_speed);
    // printf("\n预瞄距离 Ld = %f\n",Ld);

    alpha = calculate_delta_angle(Azimuth_withGeo(latitude, longitude, map_latitude_v, map_longitude_v), heading_angle);

    // printf("\nalpha = %f\n",alpha);

    double delta = atan2(2.0 * wheelbase * sin(alpha * M_PI / 180.0) / Ld, 1.0) * 180.0 / M_PI;

    // 限制转角范围
    double bound = 33; // 前轮转向极限
    if (delta > bound)
    {
        delta = bound;
    }
    if (delta < -bound)
    {
        delta = -bound;
    }

    steer_value = -delta; // 适应中邮左正右负
    // if (steer_value < 0) {
    //     steer_value *= 1.068;
    // }
    // printf("\n steer_value = %f\n",steer_value);
    return steer_value;
}

void *can_receiving_thread(void *arg)
{
    // CAN接收线程函数 - 接收并解析CAN总线数据
    printf("\ncan_receiving_Thread [%lu] is running\n", pthread_self());
    struct ifreq ifr = {0};
    struct sockaddr_can can_addr = {0};
    struct can_frame frame = {0};
    struct CanFrame frame4A2; // 用于存储0x4A2报文数据（驱动车速状态）
    struct CanFrame frame441; // 用于存储0x441报文数据（档位状态）
    struct CanFrame frame411; // 用于存储0x411报文数据（制动状态）
    struct CanFrame frame431; // 用于存储0x441报文数据（转向状态）
    struct CanFrame frame471; // 用于存储0x471报文数据（电池状态1）
    struct CanFrame frame473; // 用于存储0x473报文数据（电池状态2）
    struct CanFrame frame451; // 用于存储0x451报文数据（紧急状态）

    int sockfd = -1;
    int i;
    int ret;

    /* 打开套接字 */
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (0 > sockfd)
    {
        perror("CAN socket error");
        return NULL; // 使用return而不是exit，避免整个程序退出
    }

    /* 指定can0设备 */
    strcpy(ifr.ifr_name, "can0");
    ioctl(sockfd, SIOCGIFINDEX, &ifr);
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = ifr.ifr_ifindex;

    /* 将can0与套接字进行绑定 */
    ret = bind(sockfd, (struct sockaddr *)&can_addr, sizeof(can_addr));
    if (0 > ret)
    {
        perror("CAN bind error");
        close(sockfd);
        return NULL; // 使用return而不是exit，避免整个程序退出
    }

    /* 设置过滤规则 - 只接收过滤后的报文 */
    struct can_filter rfilter[7]; // 修正：定义10个过滤器，数组大小为10
    // 填充过滤规则，只接收指定ID的报文
    rfilter[0].can_id = 0x4A2;
    rfilter[0].can_mask = 0x7FF;
    rfilter[1].can_id = 0x441;
    rfilter[1].can_mask = 0x7FF;
    rfilter[2].can_id = 0x411;
    rfilter[2].can_mask = 0x7FF;
    rfilter[3].can_id = 0x431;
    rfilter[3].can_mask = 0x7FF;
    rfilter[4].can_id = 0x471;
    rfilter[4].can_mask = 0x7FF;
    rfilter[5].can_id = 0x473;
    rfilter[5].can_mask = 0x7FF;
    rfilter[6].can_id = 0x451;
    rfilter[6].can_mask = 0x7FF;
    // 调用setsockopt设置过滤规则
    if (setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
    {
        perror("设置CAN过滤器失败");
        close(sockfd);
        return NULL;
    }

    /* 接收数据循环 */
    while (1)
    {
        // 检查thread_mapRecord状态，如果为1则进入空执行模式
        if (thread_mapRecord == 1) {
            // printf("CAN接收线程进入等待模式...\n");
            usleep(100000); // 100ms
            continue;
        }

        // 读取CAN帧数据
        if (0 > read(sockfd, &frame, sizeof(struct can_frame)))
        {
            perror("CAN read error");
            break;
        }

        // printf("\nCAN_receive is running!\n");

        /* 校验是否接收到错误帧 */
        if (frame.can_id & CAN_ERR_FLAG)
        {
            printf("Error frame!\n");
            continue; // 修改为continue而不是break，避免因单个错误帧而退出循环
        }

        // /* 校验帧格式并打印 */
        // if (frame.can_id & CAN_EFF_FLAG) {
        // 	// 扩展帧
        // 	printf("扩展帧 <0x%08x> ", frame.can_id & CAN_EFF_MASK);
        // } else {
        // 	// 标准帧
        // 	printf("标准帧 <0x%03x> ", frame.can_id & CAN_SFF_MASK);
        // }

        /* 校验帧类型：数据帧还是远程帧 */
        if (frame.can_id & CAN_RTR_FLAG)
        {
            printf("remote request\n");
            continue;
        }

        /* 打印数据长度和帧计数 */
        // printf("[%d] [%lu] ", frame.can_dlc, frame_count++); // 已屏蔽调试输出

        /* 保存并打印数据 */
        for (i = 0; i < frame.can_dlc; i++)
        {
            rev_data[i] = frame.data[i];
            // printf("%02x ", rev_data[i]);
        }
        // printf("\n"); // 添加换行，使输出更清晰
        rev_ok = 1;

        // 根据CAN ID分别处理不同的报文
        uint32_t can_id = frame.can_id & CAN_SFF_MASK; // 获取标准帧ID

        // 解析并处理ID为0x4A2的报文 - 车辆动态状态
        if (can_id == 0x4A2)
        {
            // 将frame数据复制到frameA中
            memcpy(frame4A2.data, frame.data, sizeof(frame.data));
            // 处理解析的数据
            revin_4a2 = parseVCU_VehDynStatus(frame4A2.data);
        }
        // 解析并处理ID为0x441的报文 - 驱动状态
        else if (can_id == 0x441)
        { // 使用else if避免重复判断
            // 将frame数据复制到frameB中
            memcpy(frame441.data, frame.data, sizeof(frame.data));
            // 处理解析的数据
            revin_441 = parseVCU_DriveStatus(frame441.data);
        }
        // 解析并处理ID为0x411的报文 - 制动状态
        else if (can_id == 0x411)
        { // 使用else if避免重复判断
            // 将frame数据复制到frameC中
            memcpy(frame411.data, frame.data, sizeof(frame.data));
            // 处理解析的数据
            revin_411 = parseVCU_BrakeStatus(frame411.data);
        }
        // 解析并处理ID为0x431的报文 - 转向状态
        else if (can_id == 0x431)
        { // 使用else if避免重复判断
            // 将frame数据复制到frameD中
            memcpy(frame431.data, frame.data, sizeof(frame.data));
            // 处理解析的数据
            revin_431 = parseVCU_SteeringStatus(frame431.data);
        }
        // 解析并处理ID为0x471的报文 - 电池状态01
        else if (can_id == 0x471)
        { // 使用else if避免重复判断
            // 将frame数据复制到frameE中
            memcpy(frame471.data, frame.data, sizeof(frame.data));
            // 处理解析的数据
            revin_471 = parseVCU_BatStatus01(frame471.data);
        }
        // 解析并处理ID为0x473的报文 - 电池状态02
        else if (can_id == 0x473)
        { // 使用else if避免重复判断
            // 将frame数据复制到frameF中
            memcpy(frame473.data, frame.data, sizeof(frame.data));
            // 处理解析的数据
            revin_473 = parseVCU_BatStatus02(frame473.data);
        }
        // 解析并处理ID为0x451的报文 - 紧急状态
        else if (can_id == 0x451)
        { // 使用else if避免重复判断
            // 将frame数据复制到frameG中
            memcpy(frame451.data, frame.data, sizeof(frame.data));
            // 处理解析的数据
            revin_451 = parseVCU_EmrgStatus(frame451.data);
        }
    }

    // 关闭套接字并退出
    close(sockfd);
    printf("CAN接收线程已退出\n");
    return NULL;
}

// 初始化并绑定 CAN RAW 套接字
// int init_can_socket(const char* ifname) {
//     int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//     if (s < 0) { perror("socket"); return -1; }
//     struct ifreq ifr{};
//     std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
//     if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(s); return -1; }
//     struct sockaddr_can addr{};
//     addr.can_family = AF_CAN;
//     addr.can_ifindex = ifr.ifr_ifindex;
//     if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); close(s); return -1; }
//     // 非阻塞模式（可选）
//     int flags = fcntl(s, F_GETFL, 0);
//     fcntl(s, F_SETFL, flags | O_NONBLOCK);
//     return s;
// }

// 设置 timerfd，返回文件描述符
// int setup_timerfd(int interval_ms) {
//     int fd = timerfd_create(CLOCK_MONOTONIC, 0);
//     if (fd < 0) { perror("timerfd_create"); return -1; }
//     struct itimerspec its{};
//     its.it_interval.tv_sec  = interval_ms / 1000;
//     its.it_interval.tv_nsec = (interval_ms % 1000) * 1000000;
//     its.it_value = its.it_interval;
//     if (timerfd_settime(fd, 0, &its, nullptr) < 0) { perror("timerfd_settime"); close(fd); return -1; }
//     return fd;
// }

/**
 * @brief 解析接收到的UDP数据并更新决策控制输入
 * @param buffer 接收到的数据缓冲区
 * @param client_addr 客户端地址信息（用于日志显示）
 */
void process_udp_receive(char *buffer, struct sockaddr_in client_addr)
{
    // printf("Received buffer (hex): ");
    // for (size_t i = 0; i < BUFSIZE; ++i) {
    //     printf("%02X ", (unsigned char)buffer[i]);
    // }
    printf("\n");
    // Expected size of the data based on DECISION_IN struct
    const size_t expected_size = sizeof(DECISION_IN);

    // Check if the buffer size is sufficient
    // Note: In a real UDP scenario, you would need the actual received buffer size.
    // Assuming BUFSIZE is the max possible size and the sender sends exactly expected_size bytes.
    // A more robust solution would pass the actual received length to this function.
    // For now, we assume the buffer contains at least expected_size bytes.
    // A proper implementation should handle variable length messages or add a size prefix.
    if (BUFSIZE < expected_size)
    {
        fprintf(stderr, "Error: Received buffer is too small to contain DECISION_IN data.\n");
        return;
    }

    // Manually parse the buffer to avoid potential issues with memcpy and struct packing
    pthread_mutex_lock(&my_mutex);
    char *ptr = buffer;

    // Read bStart (int)
    decision_control_in.bStart = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read brake_bar (float)
    decision_control_in.brake_bar = *(reinterpret_cast<float *>(ptr));
    ptr += sizeof(float);

    // Read breaking_dis (double)
    decision_control_in.breaking_dis = *(reinterpret_cast<double *>(ptr));
    ptr += sizeof(double);

    // Read EPB_park (int)
    decision_control_in.EPB_park = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read gear (int)
    decision_control_in.gear = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read endSpeed (double)
    decision_control_in.endSpeed = *(reinterpret_cast<double *>(ptr));
    ptr += sizeof(double);

    // Read Emgy_brk_En (int)
    decision_control_in.Emgy_brk_En = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read Emgy_brk_ReqRmv (int)
    decision_control_in.Emgy_brk_ReqRmv = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read Emgy_FtCrashRemove (int)
    decision_control_in.Emgy_FtCrashRemove = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read Emgy_RrrCrashRemove (int)
    decision_control_in.Emgy_RrrCrashRemove = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read Emgy_LeftCrashRemove (int)
    decision_control_in.Emgy_LeftCrashRemove = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read Emgy_RightCrashRemove (int)
    decision_control_in.Emgy_RightCrashRemove = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_Hom (int)
    decision_control_in.ADU_Hom = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_BackLamp (int)
    decision_control_in.ADU_BackLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_TurnRLamp (int)
    decision_control_in.ADU_TurnRLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_TurnLLamp (int)
    decision_control_in.ADU_TurnLLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_DblFlashLamp (int)
    decision_control_in.ADU_DblFlashLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_LowBeamLamp (int)
    decision_control_in.ADU_LowBeamLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_WidthLamp (int)
    decision_control_in.ADU_WidthLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_HighBeamLamp (int)
    decision_control_in.ADU_HighBeamLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_FogLamp (int)
    decision_control_in.ADU_FogLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read ADU_BrkLamp (int)
    decision_control_in.ADU_BrkLamp = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);

    // Read map_name (string) - 读取字符串长度然后读取字符串内容
    int map_name_length = *(reinterpret_cast<int *>(ptr));
    ptr += sizeof(int);
    if (map_name_length > 0 && map_name_length < 256) { // 安全检查
        decision_control_in.map_name = std::string(ptr, map_name_length);
        ptr += map_name_length;
    } else {
        decision_control_in.map_name = "";
    }
 
    // 根据bStart值设置thread_mapRecord
    if (decision_control_in.bStart == 2) {
        thread_mapRecord = 1;
        printf("设置地图记录模式: thread_mapRecord = 1\n");
    } else if (decision_control_in.bStart == 1 || decision_control_in.bStart == 3) {
        thread_mapRecord = 0;
        printf("设置正常运行模式: thread_mapRecord = 0\n");
    }

    pthread_mutex_unlock(&my_mutex);

    _target_ind = 0;
    udp_monitor = 0;

    // Optional: Print received data for verification
    char clientIpStr[INET_ADDRSTRLEN];
    int clientPort;
    inet_ntop(AF_INET, &client_addr.sin_addr, clientIpStr, sizeof(clientIpStr));
    clientPort = ntohs(client_addr.sin_port);

    std::string timestamp_udp = getCurrentTimestamp();
    cout << "\n时间：" << timestamp_udp << endl;
    printf("=== decision_control_in (parsed from bytes) ===\n");
    printf("bStart = %d, brake_bar = %.2f, breaking_dis = %.2lf\n",
           decision_control_in.bStart, decision_control_in.brake_bar, decision_control_in.breaking_dis);
    printf("EPB_park = %d, gear = %d, endSpeed = %.2lf\n",
           decision_control_in.EPB_park, decision_control_in.gear, decision_control_in.endSpeed);
    printf("Emgy_brk_En = %d, Emgy_brk_ReqRmv = %d, Emgy_FtCrashRemove = %d\n",
           decision_control_in.Emgy_brk_En, decision_control_in.Emgy_brk_ReqRmv, decision_control_in.Emgy_FtCrashRemove);
    printf("Emgy_RrrCrashRemove = %d, Emgy_LeftCrashRemove = %d, Emgy_RightCrashRemove = %d\n",
           decision_control_in.Emgy_RrrCrashRemove, decision_control_in.Emgy_LeftCrashRemove, decision_control_in.Emgy_RightCrashRemove);
    printf("ADU_Hom = %d, ADU_BackLamp = %d, ADU_TurnRLamp = %d, ADU_TurnLLamp = %d\n",
           decision_control_in.ADU_Hom, decision_control_in.ADU_BackLamp, decision_control_in.ADU_TurnRLamp, decision_control_in.ADU_TurnLLamp);
    printf("ADU_DblFlashLamp = %d, ADU_LowBeamLamp = %d, ADU_WidthLamp = %d, ADU_HighBeamLamp = %d\n",
           decision_control_in.ADU_DblFlashLamp, decision_control_in.ADU_LowBeamLamp, decision_control_in.ADU_WidthLamp, decision_control_in.ADU_HighBeamLamp);
    printf("ADU_FogLamp = %d, ADU_BrkLamp = %d\n",
           decision_control_in.ADU_FogLamp, decision_control_in.ADU_BrkLamp);
    printf("map_name = %s\n", decision_control_in.map_name.c_str());
    printf("thread_mapRecord = %d\n", thread_mapRecord);
    printf("recvfrom %s at PORT %d\n", clientIpStr, clientPort);
}

/**
 * @brief 发送控制输出数据到客户端
 * @param sockfd 套接字描述符
 * @param client_addr 客户端地址信息
 * @param buffer 数据缓冲区
 * @param buffer_size 缓冲区大小
 * @return 发送结果，成功返回发送的字节数，失败返回-1
 */
int send_udp_response(int sockfd, struct sockaddr_in client_addr, char *buffer, size_t buffer_size)
{
    int ret;

    // 设置回复端口
    client_addr.sin_port = htons(8383);

    // 清空缓冲区
    memset(buffer, '\0', buffer_size);

    // 将控制输出结构体复制到缓冲区
    memcpy(buffer, &control_out, sizeof(CONTROL_OUT));
    /*
    // 打印即将发送的数据
    printf("[UDP发送] 准备发送数据到端口8383:\n");
    printf("[UDP发送] 数据大小: %zu bytes\n", sizeof(CONTROL_OUT));
    printf("[UDP发送] 目标地址: %s:%d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

    // 打印控制输出数据的关键字段
    printf("[UDP发送] 控制数据 - 档位: %d, 车速: %.2f km/h, 低压SOC: %.1f%%, 高压SOC: %.1f%%, 急刹状态: %d, 错误码: %d\n",
           control_out.controlOut_gear, control_out.vehicle_speed, control_out.Soc_LOW,
           control_out.Soc_HIGH, control_out.Emgy_brk_En, control_out.controlOut_ErrorCode);
    */

    // 发送数据
    ret = sendto(sockfd, buffer, sizeof(CONTROL_OUT), 0,
                 (struct sockaddr *)&client_addr, sizeof(client_addr));
    /*
    // 判断发送是否成功并打印结果
    if (ret > 0) {
        printf("[UDP发送] 成功发送 %d bytes 数据\n", ret);
    } else if (ret == 0) {
        printf("[UDP发送] 警告: 发送了0字节数据\n");
    } else {
        printf("[UDP发送] 发送失败: %s (错误码: %d)\n", strerror(errno), errno);
    }
    */

    // 返回发送结果
    return ret;
}

/**
 * @brief 解析RDC数据并赋值给rdc_in
 * @param buffer 接收到的数据缓冲区
 * @param size 数据大小
 * @return 解析是否成功
 */
bool parseRDCData(const char *buffer, size_t size)
{
    if (size != sizeof(RDC_IN))
    {
        printf("RDC data size mismatch: expected %zu, got %zu\n", sizeof(RDC_IN), size);
        return false;
    }

    try
    {
        // 直接拷贝结构体数据
        memcpy(&rdc_in, buffer, sizeof(RDC_IN));

        printf("RDC data parsed successfully:\n");
        // 按照1-18的顺序打印RDC_IN结构体成员
        printf("  1. Timestamp: %lld\n", rdc_in.RDC_timestamp);
        printf("  2. Brake: %.2f\n", rdc_in.RDC_brake);
        printf("  3. Steering: %.2f\n", rdc_in.RDC_steering_angle);
        printf("  4. Throttle: %.2f\n", rdc_in.RDC_throttle);
        printf("  5. Park: %d\n", rdc_in.RDC_Park);
        printf("  6. Gear: %d\n", rdc_in.RDC_gear);
        printf("  7. Emergency Brake Enable: %d\n", rdc_in.RDC_Emgy_brk_En);
        printf("  8. Emergency Brake Request Remove: %d\n", rdc_in.RDC_Emgy_brk_ReqRmv);
        printf("  9. Horn: %d\n", rdc_in.RDC_ADU_Hom);
        printf("  10. Back Lamp: %d\n", rdc_in.RDC_ADU_BackLamp);
        printf("  11. Turn Right Lamp: %d\n", rdc_in.RDC_ADU_TurnRLamp);
        printf("  12. Turn Left Lamp: %d\n", rdc_in.RDC_ADU_TurnLLamp);
        printf("  13. Double Flash Lamp: %d\n", rdc_in.RDC_ADU_DblFlashLamp);
        printf("  14. Low Beam Lamp: %d\n", rdc_in.RDC_ADU_LowBeamLamp);
        printf("  15. Width Lamp: %d\n", rdc_in.RDC_ADU_WidthLamp);
        printf("  16. High Beam Lamp: %d\n", rdc_in.RDC_ADU_HighBeamLamp);
        printf("  17. Fog Lamp: %d\n", rdc_in.RDC_ADU_FogLamp);
        printf("  18. Brake Lamp: %d\n", rdc_in.RDC_ADU_BrkLamp);

        return true;
    }
    catch (...)
    {
        printf("Error parsing RDC data\n");
        return false;
    }
}

/**
 * @brief UDP通信线程函数
 * @param arg 线程参数（未使用）
 * @return 线程返回值
 */
void *udp_thread(void *arg)
{

    printf("\nudp_Thread [%lu] is running\n", pthread_self());

    int sockfd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_len;
    int ret;
    char buffer[128];

    struct sockaddr_in server_addr_insert;
    struct sockaddr_in client_addr_insert;
    char buffer_insert[1024];
    MapToControl mapData;

    // 添加RDC socket相关变量
    int sockfd_rdc;
    struct sockaddr_in server_addr_rdc;
    struct sockaddr_in client_addr_rdc;
    char buffer_rdc[1024];
    socklen_t client_len_rdc;

    // 创建socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        perror("UDP socket creation failed");
        return NULL;
    }

    int sockfd_insert = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_insert < 0)
    {
        perror("UDP socket_insert creation failed");
        return NULL;
    }

    // 创建RDC socket (端口8002)
    sockfd_rdc = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_rdc < 0)
    {
        perror("UDP socket_rdc creation failed");
        return NULL;
    }

    int maxfd = std::max({sockfd, sockfd_insert, sockfd_rdc}) + 2;
    
    // 添加状态跟踪变量
    bool port8000_received = false;
    bool port8002_received = false;
    
    // 定义服务器地址
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(8000);

    bzero(&server_addr_insert, sizeof(server_addr_insert));
    server_addr_insert.sin_family = AF_INET;
    server_addr_insert.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr_insert.sin_port = htons(8001);

    // 配置RDC服务器地址 (端口8002)
    bzero(&server_addr_rdc, sizeof(server_addr_rdc));
    server_addr_rdc.sin_family = AF_INET;
    server_addr_rdc.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr_rdc.sin_port = htons(8002);

    // 绑定socket到地址
    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("UDP socket bind failed");
        close(sockfd);
        return NULL;
    }
    if (bind(sockfd_insert, (struct sockaddr *)&server_addr_insert, sizeof(server_addr_insert)) < 0)
    {
        perror("UDP socket_insert bind failed");
        close(sockfd_insert);
        return NULL;
    }
    if (bind(sockfd_rdc, (struct sockaddr *)&server_addr_rdc, sizeof(server_addr_rdc)) < 0)
    {
        perror("UDP socket_rdc bind failed");
        close(sockfd_rdc);
        return NULL;
    }
    client_len = sizeof(client_addr);
    socklen_t client_len_insert = sizeof(client_addr_insert);
    client_len_rdc = sizeof(client_addr_rdc);
    // 主循环：接收数据并响应
    while (1)
    {
        // 检查thread_mapRecord状态，如果为1则进入空执行模式
        if (thread_mapRecord == 1) {
            // printf("UDP线程进入等待模式...\n");
            sleep(1);
            continue;
        }
        
        // 清空接收缓冲区
        memset(buffer, '\0', sizeof(buffer));
        memset(buffer_insert, '\0', sizeof(buffer_insert));
        memset(buffer_rdc, '\0', sizeof(buffer_rdc));
        
        // 状态跟踪变量
        bool port8000_received = false;
        bool port8001_received = false;
        bool port8002_received = false;
        
        printf("等待UDP数据接收...\n");
        
        // 1. 首先等待端口8000（决策指令集）数据
        while (!port8000_received) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(sockfd, &readfds);
            FD_SET(sockfd_rdc, &readfds); // 同时监听RDC端口，因为RDC可以随时接收
            
            // 设置超时，避免无限阻塞
            struct timeval tv;
            tv.tv_sec = 1;  // 1秒超时
            tv.tv_usec = 0;
            
            int activity = select(maxfd, &readfds, nullptr, nullptr, &tv);
            if (activity < 0) {
                perror("select error");
                break;
            } else if (activity == 0) {
                // 超时，继续等待
                printf("等待端口8000数据...\n");
                continue;
            }
            
            // 检查端口8000
            if (FD_ISSET(sockfd, &readfds)) {
                printf("Port 8000 socket is ready for reading\n");
                // 接收数据
                ret = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                               (struct sockaddr *)&client_addr, &client_len);
                
                if (ret > 0) {
                    port8000_received = true;
                    printf("Port 8000 data received successfully, size: %d bytes\n", ret);
                    
                    // 处理接收到的数据
                    process_udp_receive(buffer, client_addr);
                    
                    // 发送响应数据
                    ret = send_udp_response(sockfd, client_addr, buffer, sizeof(buffer));
                    
                    printf("端口8000数据接收成功，等待端口8001数据...\n");
                } else {
                    // 接收失败处理
                    perror("UDP port 8000 receive failed");
                }
            }
            
            // 检查端口8002 (RDC数据)
            if (FD_ISSET(sockfd_rdc, &readfds)) {
                printf("Port 8002 socket is ready for reading\n");
                ret = recvfrom(sockfd_rdc, buffer_rdc, sizeof(buffer_rdc), 0,
                               (struct sockaddr *)&client_addr_rdc, &client_len_rdc);
                if (ret > 0) {
                    port8002_received = true;
                    printf("Received RDC data from port 8002, size: %d bytes\n", ret);
                    
                    // 更新RDC数据接收时间戳
                    {
                        std::lock_guard<std::mutex> lock(control_state_mutex);
                        last_rdc_time = std::chrono::steady_clock::now();
                        rdc_ever_received = true;
                        printf("Updated RDC timestamp\n");
                    }
                    
                    // 解析RDC数据
                    if (parseRDCData(buffer_rdc, ret)) {
                        printf("Port 8002 RDC data parsed successfully\n");
                    } else {
                        printf("Failed to parse RDC data from port 8002\n");
                    }
                } else {
                    // 接收失败处理
                    perror("UDP RDC receive failed");
                }
            }
        }
        
        // 2. 接收到端口8000数据后，等待端口8001（地图数据），同时允许8000端口数据更新
        while (!port8001_received) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(sockfd, &readfds);       // 监听8000端口，允许更新
            FD_SET(sockfd_insert, &readfds); // 监听8001端口
            FD_SET(sockfd_rdc, &readfds);    // 监听8002端口
            
            // 设置超时，避免无限阻塞
            struct timeval tv;
            tv.tv_sec = 1;  // 1秒超时
            tv.tv_usec = 0;
            
            int activity = select(maxfd, &readfds, nullptr, nullptr, &tv);
            if (activity < 0) {
                perror("select error");
                break;
            } else if (activity == 0) {
                // 超时，继续等待
                printf("等待端口8001数据...\n");
                continue;
            }
            
            // 检查端口8000，允许更新决策指令集数据
            if (FD_ISSET(sockfd, &readfds)) {
                printf("Port 8000 socket is ready for reading (update)\n");
                // 接收数据
                ret = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                               (struct sockaddr *)&client_addr, &client_len);
                
                if (ret > 0) {
                    printf("Port 8000 data updated successfully, size: %d bytes\n", ret);
                    
                    // 处理接收到的数据
                    process_udp_receive(buffer, client_addr);
                    
                    // 发送响应数据
                    ret = send_udp_response(sockfd, client_addr, buffer, sizeof(buffer));
                    
                    printf("端口8000数据更新成功，继续等待端口8001数据...\n");
                } else {
                    // 接收失败处理
                    perror("UDP port 8000 update failed");
                }
            }
            
            // 检查端口8001
            if (FD_ISSET(sockfd_insert, &readfds)) {
                printf("Port 8001 socket is ready for reading\n");
                ret = recvfrom(sockfd_insert, buffer_insert, sizeof(buffer_insert), 0,
                               (struct sockaddr *)&client_addr_insert, &client_len_insert);
                if (ret > 0) {
                    port8001_received = true;
                    printf("Port 8001 data received successfully, size: %d bytes\n", ret);
                    printf("--------------------------------\n");
                    
                    cout << "buffer_insert size =" << sizeof(buffer_insert) << ",sizeof(MapToControl)=" << sizeof(MapToControl) << endl;
                    memcpy(&mapData, buffer_insert, sizeof(MapToControl));
                    
                    // 打印接收到的数据
                    printf("接收到的地图数据:\n");
                    printf("版本号/时间戳: %ld\n", mapData.version);
                    printf("帧总数: %d\n", mapData.frameNum);
                    printf("当前帧号: %d\n", mapData.frameId);
                    printf("本帧有效数据数量: %d\n", mapData.validNumInFrame);
                    printf("路径点数据:\n");
                    for (int i = 0; i < mapData.validNumInFrame; i++) {
                        printf("点[%d]: x=%.6f, y=%.6f\n", i, mapData.info[i].x, mapData.info[i].y);
                    }
                    printf("--------------------------------\n");
                    
                    // 准备调用insertTargetPoints函数的参数
                    std::vector<CPosition> points;
                    for (int i = 0; i < mapData.validNumInFrame; i++) {
                        points.push_back(mapData.info[i]);
                    }
                    
                    // 清空之前的数据
                    map_latitude_v.clear();
                    map_longitude_v.clear();
                    
                    UDP_map = insertTargetPoints(points);
                    
                    // 将数据存入map_latitude_v和map_longitude_v
                    for (const auto &road : UDP_map) {
                        map_latitude_v.push_back(road.latitude);
                        map_longitude_v.push_back(road.longitude);
                    }
                    
                    cout << "Successfully read " << map_latitude_v.size() << " GIS_map points." << endl;
                    printf("端口8001数据接收成功，地图数据已处理\n");
                } else {
                    // 接收失败处理
                    perror("UDP port 8001 receive failed");
                }
            }
            
            // 检查端口8002 (RDC数据)
            if (FD_ISSET(sockfd_rdc, &readfds)) {
                printf("Port 8002 socket is ready for reading\n");
                ret = recvfrom(sockfd_rdc, buffer_rdc, sizeof(buffer_rdc), 0,
                               (struct sockaddr *)&client_addr_rdc, &client_len_rdc);
                if (ret > 0) {
                    port8002_received = true;
                    printf("Received RDC data from port 8002, size: %d bytes\n", ret);
                    
                    // 更新RDC数据接收时间戳
                    {
                        std::lock_guard<std::mutex> lock(control_state_mutex);
                        last_rdc_time = std::chrono::steady_clock::now();
                        rdc_ever_received = true;
                        printf("Updated RDC timestamp\n");
                    }
                    
                    // 解析RDC数据
                    if (parseRDCData(buffer_rdc, ret)) {
                        printf("Port 8002 RDC data parsed successfully\n");
                    } else {
                        printf("Failed to parse RDC data from port 8002\n");
                    }
                } else {
                    // 接收失败处理
                    perror("UDP RDC receive failed");
                }
            }
        }
        
        // 3. 当端口8000和8001都接收到数据后，设置begin_run标志，允许主线程继续执行
        // 主线程会检查GPS数据是否正常，然后才会真正启动程序
        if (port8000_received && port8001_received) {
            begin_run = 1;
            printf("端口8000和8001数据均已接收，设置begin_run=1，等待主线程检查GPS数据...\n");
        }
        
        // 4. 智能ControlState管理逻辑
        {
            std::lock_guard<std::mutex> lock(control_state_mutex);
            auto current_time = std::chrono::steady_clock::now();
            
            if (port8002_received) {
                // 接收到RDC数据，立即切换到远程控制模式
                ControlState = 2;
                printf("Port 8002 RDC data received, ControlState set to 2 (Remote Control)\n");
            } else if (rdc_ever_received) {
                // 检查RDC数据超时
                auto time_since_last_rdc = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_rdc_time).count();
                
                if (time_since_last_rdc > RDC_TIMEOUT_MS) {
                    // RDC数据超时，切换到自动驾驶模式
                    if (ControlState == 2) {
                        ControlState = 1;
                        printf("RDC timeout (%ld ms), switching ControlState to 1 (Auto Control)\n", time_since_last_rdc);
                    }
                } else {
                    // RDC数据未超时，保持远程控制模式
                    if (ControlState != 2) {
                        ControlState = 2;
                        printf("RDC data still valid (%ld ms ago), maintaining ControlState 2 (Remote Control)\n", time_since_last_rdc);
                    }
                }
            } else if (port8000_received) {
                // 只接收到自动驾驶数据，且从未接收过RDC数据
                ControlState = 1;
                printf("Port 8000 data received, ControlState set to 1 (Auto Control)\n");
            }
        }
    }

    // 关闭socket（实际上永远不会执行到这里，除非while循环被打断）
    close(sockfd);
    return NULL;
}

/*
date:20241224
author:rxl
增加解析大巴车指令报文程序
*/

// 组装 发送CAN 报文（决策接入更换参数）
CanFrame createCanFrame(int handshake, int gear, int park, double brake, double angle, double steer_speed, double vehicle_speed)
{
    CanFrame frame = {};

    // Byte 0: 握手（0为不握手、5为握手）、档位（1N，2R，3D）、驻车（1释放，2驻车）
    frame.data[0] = (handshake & 0b00000111) | ((gear & 0b00000111) << 3) | ((park & 0b00000011) << 6);

    // Byte 1-2: 刹车减速度控制（低8位在前，高8位在后） brake 最大为10
    int16_t brake_raw = static_cast<int16_t>((15 - brake) * 2048);
    frame.data[1] = brake_raw & 0xFF;
    frame.data[2] = (brake_raw >> 8) & 0xFF;

    // Byte 3-4: 方向盘转角（低8位在前，高8位在后）左负右正
    int16_t angle_raw = static_cast<int16_t>((1575 + angle));
    frame.data[3] = angle_raw & 0xFF;
    frame.data[4] = (angle_raw >> 8) & 0xFF;

    // Byte 5: 方向盘角速度
    frame.data[5] = static_cast<uint8_t>(steer_speed / 10);

    // Byte 6-7: 车速（高4位在 byte7，低8位在 byte6）
    uint16_t vehicle_speed_raw = static_cast<uint16_t>(vehicle_speed / 0.05);
    frame.data[6] = vehicle_speed_raw & 0xFF;        // 低8位
    frame.data[7] = (vehicle_speed_raw >> 8) & 0x0F; // 高4位

    // 打印报文内容
    std::cout << "Created CAN Frame: [ ";
    for (int i = 0; i < 8; ++i)
    {
        std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << "]" << std::endl;

    return frame;
}

void sendCanFrame(const CanFrame &frame)
{
    // 模拟发送 CAN 报文
    std::cout << "Sending CAN  : [ ";
    for (int i = 0; i < 8; ++i)
    {
        std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << "]" << std::endl;
}

// 工具：将速度值转换为 0.05 的倍数，并打印转换过程
double convert_and_print_speed(double &speed)
{ // 使用引用传递，可以直接修改speed的值
    // std::cout << "原始速度：" << speed;
    double multiple = speed / 0.05;
    double rounded_multiple = std::round(multiple);
    speed = rounded_multiple * 0.05;
    return speed;
}
// 工具：减速度
double convert_and_print_brake_bar(double &brake_bar)
{
    // std::cout << "原始减速度：" << brake_bar;

    // 用 0.00048828125 作为精度
    double multiple = brake_bar / 0.01;             // 计算出多少个0.01
    double rounded_multiple = std::round(multiple); // 四舍五入
    brake_bar = rounded_multiple * 0.01;            // 计算转换后的减速度

    // std::cout << std::fixed << std::setprecision(4); // 设置输出精度
    // std::cout << "转换后的减速度为：" << brake_bar << std::endl;

    return brake_bar;
}
// 工具：补全8位十六进制报文
std::string formatCanData(const CanFrame &frame)
{
    std::ostringstream oss;
    for (int i = 0; i < 8; ++i)
    { // 假设 CAN 数据帧长度为 8
        oss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(frame.data[i]) << " ";
    }
    return oss.str();
}

// 计算异或校验和函数（XOR）
uint8_t calculateChecksum(uint8_t *data, size_t length)
{
    uint8_t xor_sum = 0;
    for (size_t i = 0; i < length; i++)
    {
        xor_sum ^= data[i];
    }
    return xor_sum;
}

/*
brake_bar：对应公式输出值，实际刹车压力0-100，公式：800(输入)*0.01（精度）=80 bar（输出）
EPB_Park：驻车状态 0释放 1拉起;需要先拉起再释放
steer_angle：对应公式输出值，方向盘转角，左正右负，最大值540deg对应车轮倾角33°，公式；10800（输入）*0.1（精度）=540 deg（输出）
steer_speed：对应公式输出值，方向盘角速度，最大520 deg/s，公式；1000（输入）*0.2（精度）=200 deg/s（输出）
endspeed：对应公式输出值，最大40 km/h，公式：800（输入）*0.05（精度）=40 km/h（输出）
Emgy_brk_En：急刹使能，0移除，1急刹
Emgy_brk_Ft：前触边移除，0不移除，1移除
Emgy_brk_Rr：后触边移除，0不移除，1移除
Emgy_brk_Left：左触边移除，0不移除，1移除
Emgy_brk_Right：右触边移除，0不移除，1移除
ADU_Hom ：喇叭控制
ADU_BackLamp：倒车灯控制
ADU_TurnRLamp：右转灯控制
ADU_TurnLLamp：左转灯控制
ADU_DblFlashLamp：双闪灯控制
ADU_LowBeamLamp：近光灯控制
ADU_HighBeamLamp：远光灯控制
ADU_WidthLamp：示宽灯控制
ADU_FogLamp：雾灯控制
ADU_BrkLamp：制动灯控制
ADU_DoorUnlcking：车门解锁
ADU_DoorLocking：车门上锁
ADU_TricolourRedLamp：三色灯控制-红
ADU_TricolourYellowLamp：三色灯控制-黄
ADU_TricolourGreenLamp：三色灯控制-绿
ADU_FANRunCtrl：风扇控制

默认参数：
brake_bar：0    无刹车压力
EPB_Park：0     释放
steer_angle：0  正方向0
steer_speed：200    200 deg/s
endspeed：0     无速度
gear：5         N档
Emgy_brk_En：0  无急刹
Emgy_brk_Ft：0  无前触边移除
Emgy_brk_Rr：0  无后触边移除
Emgy_brk_Left：0  无左触边移除
Emgy_brk_Right：0  无右触边移除
ADU_Hom ：0  喇叭关闭
ADU_BackLamp：0  倒车灯关闭
ADU_TurnRLamp：0  右转灯关闭
ADU_TurnLLamp：0  左转灯关闭
ADU_DblFlashLamp：0  双闪灯关闭
ADU_LowBeamLamp：0  近光灯关闭
ADU_HighBeamLamp：0  远光灯关闭
ADU_WidthLamp：0  示宽灯关闭
ADU_FogLamp：0  雾灯关闭
ADU_BrkLamp：0  制动灯关闭
ADU_DoorUnlcking：0  车门解锁
ADU_DoorLocking：0  车门上锁
ADU_TricolourRedLamp：0  三色灯控制-红
ADU_TricolourYellowLamp：0  三色灯控制-黄
ADU_TricolourGreenLamp：0  三色灯控制-绿
ADU_FANRunCtrl：0  风扇控制
*/

void createEightCanFrames(std::vector<can_frame> &eightFrames, int step,
                          float brake_bar = 0,
                          int EPB_Park = 1,
                          double steer_angle = 0,
                          double steer_speed = 200,
                          double endspeed = 0,
                          int gear = 5,
                          double PedalposReq = 0, // 0-100油门请求 远程使用
                          int Emgy_brk_En = 0,
                          int Emgy_brk_ReqRmv = 0,
                          int Emgy_brk_Ft = 0,
                          int Emgy_brk_Rr = 0,
                          int Emgy_brk_Left = 0,
                          int Emgy_brk_Right = 0,
                          int ADU_Hom = 0,
                          int ADU_BackLamp = 0,
                          int ADU_TurnRLamp = 0,
                          int ADU_TurnLLamp = 0,
                          int ADU_DblFlashLamp = 0,
                          int ADU_LowBeamLamp = 0,
                          int ADU_WidthLamp = 0,
                          int ADU_HighBeamLamp = 0,
                          int ADU_FogLamp = 0,
                          int ADU_BrkLamp = 0,
                          int ADU_DoorUnlcking = 0,
                          int ADU_DoorLocking = 0,
                          int ADU_TricolourRedLamp = 0,
                          int ADU_TricolourYellowLamp = 0,
                          int ADU_TricolourGreenLamp = 0,
                          int ADU_FANRunCtrl = 0)
{
    std::lock_guard<std::mutex> lock(framesMutex);
    eightFrames.clear();

    static uint8_t frameCounter = 0;

    auto push_frame = [&](uint32_t id, const std::array<uint8_t, 8> &data)
    {
        can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = 8;
        std::copy(data.begin(), data.end(), frame.data);
        eightFrames.push_back(frame);
    };

    // 210
    std::array<uint8_t, 8> data210{};
    if (step == 0)
    {
        data210[0] = 0x80;
    }
    else if (step == 1 || step == 2)
    {
        data210[0] = 0xA0;
        data210[2] = 0x07;
        data210[3] = 0xD0;
    }
    else if (step == 3)
    {
        data210[0] = 0x80;
    }
    else
    {
        data210[0] = 0xA0;
        int16_t brake_barValue = static_cast<int16_t>(brake_bar / 0.01);
        data210[2] = (brake_barValue >> 8) & 0xFF;
        data210[3] = brake_barValue & 0xFF;
    }
    data210[6] = frameCounter & 0x0F;
    data210[7] = calculateChecksum(data210.data(), 7);
    push_frame(0x210, data210);

    // 220
    std::array<uint8_t, 8> data220{};
    data220[0] = 0x80 | ((EPB_Park & 0b01) << 6);
    data220[6] = frameCounter & 0x0F;
    data220[7] = calculateChecksum(data220.data(), 7);
    push_frame(0x220, data220);

    // 230
    std::array<uint8_t, 8> data230{};
    if (step == 0)
    {
        data230[0] = 0x80;
        data230[6] = frameCounter & 0x0F;
    }
    else
    {
        data230[0] = 0xA0;
        int16_t angleValue = static_cast<int16_t>(steer_angle / 0.05);
        data230[1] = (angleValue >> 8) & 0xFF;
        data230[2] = angleValue & 0xFF;
        int16_t steer_speedValue = static_cast<int16_t>(steer_speed / 0.2);
        data230[5] = (steer_speedValue >> 4) & 0xFF;
        data230[6] = ((steer_speedValue & 0x0F) << 4) | (frameCounter & 0x0F);
    }
    data230[7] = calculateChecksum(data230.data(), 7);
    push_frame(0x230, data230);

    // 240
    std::array<uint8_t, 8> data240{};
    if (step == 0 || step == 1 || step == 3)
    {
        data240[0] = 0x80;
        data240[1] = 0x07;
        data240[2] = 0xD0;
        data240[3] = 0x27;
        data240[4] = 0x10;
    }
    else if (ControlState == 1) // 决策模式：速度环控制
    {
        data240[0] = 0xA0 | (gear & 0x0F);
        double clamped_speed = std::clamp(endspeed, -100.0, 104.75);
        uint16_t speed_raw = static_cast<uint16_t>((clamped_speed + 100) / 0.05);
        speed_raw = std::clamp(speed_raw, static_cast<uint16_t>(0), static_cast<uint16_t>(4095));
        data240[1] = (speed_raw >> 8) & 0x0F;
        data240[2] = speed_raw & 0xFF;
        data240[3] = 0x27;
        data240[4] = 0x10;
    }
    else if (ControlState == 2) // 远程模式：油门踏板控制
    {
        data240[0] = 0xE0 | (gear & 0x0F);

        // 油门踏板请求控制 (第5字节，40bit位起占8bit)
        uint8_t pedal_raw = static_cast<uint8_t>(PedalposReq / 0.5); // 转换百分比
        data240[5] = pedal_raw;
    }
    data240[6] = frameCounter & 0x0F;
    data240[7] = calculateChecksum(data240.data(), 7);
    push_frame(0x240, data240);

    // 251
    std::array<uint8_t, 8> data251{};
    data251[0] = ((Emgy_brk_Right & 1) << 5) | ((Emgy_brk_Left & 1) << 4) | ((Emgy_brk_Rr & 1) << 3) |
                 ((Emgy_brk_Ft & 1) << 2) | ((Emgy_brk_ReqRmv & 1) << 1) | (Emgy_brk_En & 1);
    data251[6] = frameCounter & 0x0F;
    data251[7] = calculateChecksum(data251.data(), 7);
    push_frame(0x251, data251);

    // 260
    std::array<uint8_t, 8> data260{};
    data260[1] = ((ADU_DblFlashLamp & 1) << 6) | ((ADU_TurnLLamp & 1) << 5) | ((ADU_TurnRLamp & 1) << 4) |
                 ((ADU_BackLamp & 1) << 3) | ((ADU_Hom & 1) << 1);
    data260[2] = ((ADU_BrkLamp & 1) << 7) | ((ADU_FogLamp & 1) << 6) | ((ADU_WidthLamp & 1) << 5) |
                 ((ADU_HighBeamLamp & 1) << 4) | ((ADU_LowBeamLamp & 1) << 3);
    data260[6] = frameCounter & 0x0F;
    data260[7] = calculateChecksum(data260.data(), 7);
    push_frame(0x260, data260);

    // 262
    std::array<uint8_t, 8> data262{};
    data262[6] = frameCounter & 0x0F;
    data262[7] = calculateChecksum(data262.data(), 7);
    push_frame(0x262, data262);

    // 272
    std::array<uint8_t, 8> data272{};
    data272[6] = frameCounter & 0x0F;
    data272[7] = calculateChecksum(data272.data(), 7);
    push_frame(0x272, data272);

    frameCounter = (frameCounter + 1) & 0x0F;
}

/*
program：发送can报文控制车辆
steer_angle:循迹计算后的方向盘转角
CONTROL_OUT c_out：控制输出
DECISION_IN &din：决策输入
recvData revin：解析310得到的结构体，包含当前车速等信息
unsigned char *orders：报文指针
int &canFrameCount：can初始化以及控制功能计数器
int Map_ind：预瞄点地图下标
*/
void send_candata(double steer_angle, CONTROL_OUT &c_out, DECISION_IN &din, VCU_VehDynStatus revin_4a2, VCU_DriveStatus revin_441, int &canFrameCount)
{
    // 当前步骤计数
    static int currentStep = 0;

    // Step Emy: 急停控制帧 判断是否需要急停
    if (din.Emgy_brk_En == 1 || begin_run == 0) // 急停
    {
        std::cout << "Step Emy: 急停" << ",begin_run=" << begin_run << std::endl;
        // 创建紧急控制报文
        createEightCanFrames(eightFrames, 10, 100, 0, 0, 200, 0, 5, 1, 0, 0, 0, 0, 0, 1);
        canFrameCount = -1;
        currentStep = -1;
    }
    else if (din.Emgy_brk_ReqRmv == 1) // 移除急停
    {
        std::cout << "Step Emy——Rmv: 移除急停" << std::endl;
        // 移除急停请求
        createEightCanFrames(eightFrames, 10, 100, 0, 0, 200, 0, 5, 0, 1, 1, 1, 1, 1);
        canFrameCount = 0;
        currentStep = 0;
    }
    // Step 0: 初始化握手
    if (canFrameCount >= 0 && canFrameCount < 30 && begin_run == 1)
    {
        std::cout << "Step 0: 初始化握手阶段" << std::endl;
        // 创建握手报文（210、220、230、240需要握手）
        createEightCanFrames(eightFrames, 0);
        ++canFrameCount;
        currentStep = 0;
    }
    // Step 1: 建压阶段
    else if ((currentStep == 0 || currentStep == 1) && canFrameCount < 40)
    {
        std::cout << "Step 1: 建压阶段" << std::endl;
        createEightCanFrames(eightFrames, 1);
        ++canFrameCount;
        currentStep = 1;
    }
    // Step 2: 换挡阶段
    else if ((currentStep == 1 || currentStep == 2) && canFrameCount < 60)
    {
        std::cout << "Step 2: 换挡阶段" << std::endl;
        createEightCanFrames(eightFrames, 2);
        ++canFrameCount;
        currentStep = 2;
    }
    // Step 3: 解除建压阶段
    else if ((currentStep == 2 || currentStep == 3) && canFrameCount < 80)
    {
        std::cout << "Step 3: 解除建压阶段" << std::endl;
        createEightCanFrames(eightFrames, 3);
        ++canFrameCount;
        currentStep = 3;
    }
    // Step 4: 实际控制帧
    else if (currentStep >= 3 && canFrameCount >= 80 && begin_run == 1)
    {
        std::cout << "Step 4: 控制变速阶段" << std::endl;
        static double ctrl_speed = 0;       // 实际下发末速度
        static bool flag = false;           // 加速false，减速true
        static int gear_count = 0;          // 档位计数器
        static float tmp_din_brake_bar = 0; // 指令刹车压力
  
        /* 指令策略判断 */
        // 1. 刹车以及油门冲突
        if (din.endSpeed != 0 && din.brake_bar != 0) // 同时按下刹车和油门指令冲突报错
        {
            cout << "刹车和油门指令冲突，无法下发" << endl;
            c_out.controlOut_ErrorCode = 1;
            return;
        }
        // 2. 驻车档以及车速冲突
        if (din.gear == 1 && c_out.vehicle_speed != 0) // 同时按下刹车和油门指令冲突报错
        {
            cout << "驻车档以及车速冲突，无法下发" << endl;
            c_out.controlOut_ErrorCode = 2;
            return;
        }
        //========================== 1.先判断档位 ==========================*/
        if (din.gear != revin_441.Drv_GearAct && din.brake_bar == 0)
        {
            // 0.存储原先指令压力
            tmp_din_brake_bar = din.brake_bar;
            // 1.再建压赋值
            din.brake_bar = 50;
            cout << "----档位切换:din.gear=" << din.gear << ",revin_441.Drv_GearAct=" << revin_441.Drv_GearAct << "----" << endl;
            gear_count += 1;
        }
        else if ((tmp_din_brake_bar != din.brake_bar) && (gear_count != 0))
        {
            // 2. 解除建压赋值 还原原先指令0压力
            din.brake_bar = 0;
            gear_count = 0;
        }
        //========================== 2.再控制速度 ==========================*/

        // 中途停车情况重新赋值 - 优化条件判断
        if (fabs(ctrl_speed - c_out.vehicle_speed) >= 6 && ctrl_speed > c_out.vehicle_speed)
        {
            ctrl_speed = c_out.vehicle_speed;
            std::cout << "检测到车速异常下降，重置控制速度为: " << ctrl_speed << std::endl;
        }

        // 根据目标速度和当前速度关系，确定加减速模式
        if (din.endSpeed > c_out.vehicle_speed) // 需要加速
        {
            flag = false;
            std::cout << "目标速度大于当前速度，进入加速模式" << std::endl;
        }
        else if (din.endSpeed < c_out.vehicle_speed) // 需要减速
        {
            flag = true;
            std::cout << "目标速度小于当前速度，进入减速模式" << std::endl;
        }
        else{
            std::cout << "目标速度等于当前速度，保持不变" << std::endl;
        }
        // 加速过程
        if (!flag) // 加速模式
        {
            if (c_out.vehicle_speed < din.endSpeed)
            {
                // 自适应加速：根据当前速度和目标速度的差值调整加速量
                double speed_diff = din.endSpeed - ctrl_speed;
                double speed_unit;

                if (speed_diff > 10) // 差距较大时，使用较大加速度
                {
                    speed_unit = 0.05 * 4; // 4个最小单位
                }
                else if (speed_diff > 5) // 中等差距
                {
                    speed_unit = 0.05 * 2; // 2个最小单位
                }
                else // 接近目标速度
                {
                    speed_unit = 0.05; // 1个最小单位，平滑过渡
                }

                ctrl_speed += speed_unit;

                // 确保不会加速过度
                if (ctrl_speed > din.endSpeed)
                {
                    ctrl_speed = din.endSpeed;
                }

                std::cout << "自适应加速: " << speed_unit << " 单位/周期, 目标速度: "
                          << din.endSpeed << ", 当前控制速度: " << ctrl_speed << std::endl;
            }
            else
            {
                ctrl_speed = din.endSpeed; // 达到目标速度或差距过小，保持不变
                std::cout << "已达到目标速度: " << din.endSpeed << ",下发速度ctrl:" << ctrl_speed << std::endl;
            }
        }
        // 减速过程
        else // 减速模式
        {
            // 计算需要的减速度
            double speed_diff = c_out.vehicle_speed - din.endSpeed;

            if (speed_diff > 8) // 需要较大减速
            {
                // PID计算刹车压力，使用前馈量进行
                din.brake_bar = static_cast<float>(calculateBrakePressure(revin_4a2.vehicle_speed, din.endSpeed));
                std::cout << "大幅减速 - PID计算刹车压力: " << din.brake_bar << std::endl;
                if (ctrl_speed < din.endSpeed)
                {
                    ctrl_speed = din.endSpeed;
                }
            }
            else if (speed_diff > 3) // 中等减速
            {
                // 中等刹车压力
                din.brake_bar = static_cast<float>(calculateBrakePressure(revin_4a2.vehicle_speed, din.endSpeed) * 0.7);
                std::cout << "中等减速 - PID计算刹车压力: " << din.brake_bar << std::endl;

                if (ctrl_speed < din.endSpeed)
                {
                    ctrl_speed = din.endSpeed;
                }
            }
            else // 微调减速
            {
                // 轻微刹车压力
                din.brake_bar = static_cast<float>(calculateBrakePressure(revin_4a2.vehicle_speed, din.endSpeed) * 0.5);
                std::cout << "微调减速 - PID计算刹车压力: " << din.brake_bar << std::endl;

                // 缓慢调整到目标速度
                ctrl_speed = din.endSpeed;
            }

            // 检查减速效果
            if (revin_4a2.vehicle_speed <= din.endSpeed + 2)
            {
                // 已经接近目标速度，取消刹车
                din.brake_bar = 0;
                ctrl_speed = din.endSpeed;
                std::cout << "减速完成，保持目标速度: " << din.endSpeed << std::endl;
            }
        }
        // 确保控制速度是0.05的整数倍
        ctrl_speed = convert_and_print_speed(ctrl_speed);
        // 确保控制压力是0.01的整数倍
        double tmp_brake_bar = din.brake_bar;
        din.brake_bar = convert_and_print_brake_bar(tmp_brake_bar);

        // 确保所有参数在有效范围内
        if (ctrl_speed < 0)
            ctrl_speed = 0;
        if (din.brake_bar < 0)
            din.brake_bar = 0;

        // 使用memset清空eightFrames数组，避免使用未初始化的内存
        // memset(eightFrames, 0, sizeof(unsigned char) * 8 * 8);
        eightFrames.clear();

        createEightCanFrames(eightFrames, 4, din.brake_bar, din.EPB_park, steer_angle, 200, ctrl_speed, din.gear,
                             din.Emgy_brk_En, din.Emgy_brk_ReqRmv, din.Emgy_FtCrashRemove, din.Emgy_RrrCrashRemove, din.Emgy_LeftCrashRemove, din.Emgy_RightCrashRemove,
                             din.ADU_Hom, din.ADU_BackLamp, din.ADU_TurnRLamp, din.ADU_TurnLLamp, din.ADU_DblFlashLamp, din.ADU_LowBeamLamp, din.ADU_WidthLamp, din.ADU_HighBeamLamp, din.ADU_FogLamp, din.ADU_BrkLamp);
        ++canFrameCount;
    }
    // Step 5: 结束帧
    else if (currentStep >= 4 && canFrameCount > 0 && c_out.vehicle_speed == 0 && din.EPB_park == 1 && din.endSpeed == 0)
    {
        std::cout << "Step 5: 结束帧" << std::endl;
        currentStep = -2;

        // 在调用createEightCanFrames前清空内存
        // memset(eightFrames, 0, sizeof(unsigned char) * 8 * 8);
        eightFrames.clear();

        createEightCanFrames(eightFrames, 1); // 建压

        // 再次清空内存，避免使用可能已被释放的内存
        // memset(eightFrames, 0, sizeof(unsigned char) * 8 * 8);
        eightFrames.clear();

        createEightCanFrames(eightFrames, 5, 20, 1, 0, 50, 0, 1); // 换P挡

        canFrameCount = -2;
    }
    else if (currentStep < 0)
    {
        if (currentStep == -1)
        {
            std::cout << "急停未移除" << std::endl;
        }
        if (currentStep == -2)
        {
            std::cout << "结束状态需重启" << std::endl;
        }
    }
    else
    {
        std::cout << "未知状态：currentStep=" << currentStep
                  << ", canFrameCount=" << canFrameCount
                  << ", begin_run=" << begin_run
                  << ", vehicle_speed=" << revin_4a2.vehicle_speed
                  << ", EPB_park=" << din.EPB_park
                  << ", endSpeed=" << din.endSpeed
                  << std::endl;
    }
}

/*
program：发送can报文控制车辆（RDC重载版本）
steer_angle:循迹计算后的方向盘转角
CONTROL_OUT c_out：控制输出
RDC_IN rdc_in：远程驾驶控制输入
recvData revin：解析310得到的结构体，包含当前车速等信息
unsigned char *orders：报文指针
int &canFrameCount：can初始化以及控制功能计数器
*/
void send_candata(RDC_IN rdc_in, CONTROL_OUT &c_out, VCU_VehDynStatus revin_4a2, VCU_DriveStatus revin_441, int &canFrameCount)
{
    // 当前步骤计数
    static int currentStep = 0;

    printf("RDC控制模式 - 刹车: %.2f, 油门: %.2f, 转向: %.2f, 档位: %d\n",
           rdc_in.RDC_brake, rdc_in.RDC_throttle, rdc_in.RDC_steering_angle, rdc_in.RDC_gear);

    // Step Emy: 急停控制帧 判断是否需要急停
    if (rdc_in.RDC_Emgy_brk_En == 1 || begin_run == 0) // 急停
    {
        std::cout << "Step Emy: 急停" << ",begin_run=" << begin_run << std::endl;
        // 创建紧急控制报文
        createEightCanFrames(eightFrames, 10, 100, 0, 0, 200, 0, 5, 1, 0, 0, 0, 0, 0, 1);
        canFrameCount = -1;
        currentStep = -1;
    }
    else if (rdc_in.RDC_Emgy_brk_ReqRmv == 1) // 移除急停
    {
        std::cout << "Step Emy——Rmv: 移除急停" << std::endl;
        // 移除急停请求
        createEightCanFrames(eightFrames, 10, 100, 0, 0, 200, 0, 5, 0, 1, 1, 1, 1, 1);
        canFrameCount = 0;
        currentStep = 0;
    }
    // Step 0: 初始化握手
    if (canFrameCount >= 0 && canFrameCount < 30 && begin_run == 1)
    {
        std::cout << "Step 0: 初始化握手阶段" << std::endl;
        // 创建握手报文（210、220、230、240需要握手）
        createEightCanFrames(eightFrames, 0);
        ++canFrameCount;
        currentStep = 1;
    }
    // Step 1: RDC直接控制模式（跳过建压、换挡、解除建压阶段）
    else if (currentStep > 0 && canFrameCount >= 30 && begin_run == 1)
    {
        std::cout << "Step 4: RDC直接控制模式" << std::endl;

        // RDC模式下直接使用远程指令进行油门开合度控制

        float current_throttle = rdc_in.RDC_throttle;
        float current_brake = rdc_in.RDC_brake;

        // 1. 刹车以及油门冲突检查
        if (current_throttle != 0 && current_brake != 0)
        {
            cout << "RDC模式：刹车和油门指令冲突，优先执行刹车" << endl;
            current_throttle = 0; // 清除油门指令
            c_out.controlOut_ErrorCode = 1;
        }

        // 2. 驻车档以及车速冲突检查
        if (rdc_in.RDC_gear == 1 && c_out.vehicle_speed != 0)
        {
            cout << "RDC模式：驻车档以及车速冲突，强制刹车" << endl;
            current_brake = 50; // 强制刹车
            c_out.controlOut_ErrorCode = 2;
        }

        // 确保控制参数在有效范围内
        if (current_throttle < 0)
            current_throttle = 0;
        if (current_throttle > 100)
            current_throttle = 100;
        if (current_brake < 0)
            current_brake = 0;
        if (current_brake > 100)
            current_brake = 100;

        // 转换刹车压力为0.01的整数倍
        double tmp_brake_bar = current_brake;
        current_brake = convert_and_print_brake_bar(tmp_brake_bar);

        // 清空并创建CAN帧
        eightFrames.clear();

        // 直接使用RDC结构体成员进行控制
        createEightCanFrames(eightFrames, 4, current_brake, rdc_in.RDC_Park, rdc_in.RDC_steering_angle, 200, 0, rdc_in.RDC_gear, current_throttle,
                             rdc_in.RDC_Emgy_brk_En, rdc_in.RDC_Emgy_brk_ReqRmv, 0, 0, 0, 0, rdc_in.RDC_ADU_Hom, rdc_in.RDC_ADU_BackLamp,
                             rdc_in.RDC_ADU_TurnRLamp, rdc_in.RDC_ADU_TurnLLamp, rdc_in.RDC_ADU_DblFlashLamp,
                             rdc_in.RDC_ADU_LowBeamLamp, rdc_in.RDC_ADU_WidthLamp, rdc_in.RDC_ADU_HighBeamLamp, rdc_in.RDC_ADU_FogLamp);

        ++canFrameCount;
    }
    // Step 5: 结束帧
    else if (currentStep >= 4 && canFrameCount > 0 && c_out.vehicle_speed == 0 && rdc_in.RDC_Park == 1)
    {
        std::cout << "Step 5: RDC结束帧" << std::endl;
        currentStep = -2;

        eightFrames.clear();
        createEightCanFrames(eightFrames, 1); // 建压

        eightFrames.clear();
        createEightCanFrames(eightFrames, 5, 20, 1, 0, 50, 0, 1); // 换P挡

        canFrameCount = -2;
    }
    else if (currentStep < 0)
    {
        if (currentStep == -1)
        {
            std::cout << "RDC模式：急停未移除" << std::endl;
        }
        if (currentStep == -2)
        {
            std::cout << "RDC模式：结束状态需重启" << std::endl;
        }
    }
    else
    {
        std::cout << "RDC模式未知状态：currentStep=" << currentStep
                  << ", canFrameCount=" << canFrameCount
                  << ", begin_run=" << begin_run
                  << ", vehicle_speed=" << revin_4a2.vehicle_speed
                  << ", EPB_park=" << rdc_in.RDC_Park
                  << std::endl;
    }
}

// 初始化日志文件
void initLogFile()
{
    // 获取当前时间戳，生成文件名
    std::string timestamp = getCurrentTimestamp();
    std::string logFilePath = "/home/ztl/log/log_" + timestamp.substr(0, 19) + ".txt";

    // 如果已经有打开的文件，先关闭
    if (logFile.is_open())
    {
        logFile.close();
    }

    // 打开新文件
    logFile.open(logFilePath, std::ios::out);
    if (!logFile.is_open())
    {
        std::cerr << "Error opening log file: " << logFilePath << std::endl;
        exit(EXIT_FAILURE);
    }

    // 写入表头
    logFile << "时间戳, 纬度, 经度, 航向角, 当前车速 , 方向盘角度, 地图纬度, 地图经度, 启动标签, 刹车压力, 刹车距离, EPB电子驻车, 档位, 目标末速度, 急刹使能, 急刹移除请求, 前触边移除请求, 后触边移除请求, 左触边移除请求, 右触边移除请求, 喇叭, 倒车灯, 右转灯, 左转灯, 双闪, 近光灯, 示廓灯, 远光灯, 雾灯, 刹车灯, 预瞄距离,横向误差, alpha";
    
    // 添加EA雷达数据表头
    for (int i = 0; i < NUM_PROBES; i++)
    {
        logFile << ", EA雷达" << i + 1;
    }
    
    // 添加E8雷达数据表头
    for (int i = 0; i < NUM_PROBES; i++)
    {
        logFile << ", E8雷达" << i + 1;
    }

    // 为CAN帧数据添加表头
    for (int i = 0; i < 8; i++)
    {
        logFile << ", CAN帧" << i + 1 << "_ID, CAN帧" << i + 1 << "_数据";
    }

    // 表头结束，换行
    logFile << std::endl;

    std::cout << "Created new log file: " << logFilePath << std::endl;
    recordCounter = 0;                               // 重置记录计数
    current_log_timestamp = timestamp.substr(0, 19); // 保存当前日志文件的时间戳
}

// 保存日志到文件
void logToFile(DECISION_IN decision_control_in, VPARAMS cur_params, GIS GIS_map, double Ld, double alpha, std::vector<can_frame> can_frame_log,double Puresuit_lateral_error)
{
    // 获取当前时间戳
    std::string timestamp = getCurrentTimestamp();

    // 如果日志文件未打开或者记录数已达到上限，初始化新文件
    if (!logFile.is_open() || recordCounter >= MAX_LOG_ENTRIES)
    {
        initLogFile();
    }

    // 写入日志内容
    logFile << timestamp << ", "
            /**** 当前车辆状态 ****/
            << std::fixed << std::setprecision(8) << cur_params.latitude << ", "
            << std::fixed << std::setprecision(8) << cur_params.longitude << ", "
            << cur_params.heading_angle << ", "
            << cur_params.vehicle_speed << ", "
            << cur_params.steer_angle << ", "
            /**** 地图目标点 ****/
            << GIS_map.latitude << ", "
            << GIS_map.longitude << ", "
            /**** 决策输入（全部22个参数） ****/
            << decision_control_in.bStart << ", "
            /* 制动控制 210 */
            << decision_control_in.brake_bar << ", "
            << decision_control_in.breaking_dis << ", "
            /* 驻车控制 220 */
            << decision_control_in.EPB_park << ", "
            /* 驱动控制 240 */
            << decision_control_in.gear << ", "
            << decision_control_in.endSpeed << ", "
            /* 紧急控制 251 */
            << decision_control_in.Emgy_brk_En << ", "
            << decision_control_in.Emgy_brk_ReqRmv << ", "
            << decision_control_in.Emgy_FtCrashRemove << ", "
            << decision_control_in.Emgy_RrrCrashRemove << ", "
            << decision_control_in.Emgy_LeftCrashRemove << ", "
            << decision_control_in.Emgy_RightCrashRemove << ", "
            /* 附件控制 260 */
            << decision_control_in.ADU_Hom << ", "
            << decision_control_in.ADU_BackLamp << ", "
            << decision_control_in.ADU_TurnRLamp << ", "
            << decision_control_in.ADU_TurnLLamp << ", "
            << decision_control_in.ADU_DblFlashLamp << ", "
            << decision_control_in.ADU_LowBeamLamp << ", "
            << decision_control_in.ADU_WidthLamp << ", "
            << decision_control_in.ADU_HighBeamLamp << ", "
            << decision_control_in.ADU_FogLamp << ", "
            << decision_control_in.ADU_BrkLamp << ", "
            /**** 控制循迹 */
            << Ld << ", "
            << Puresuit_lateral_error << ", "
            << alpha;
            
    // 记录EA雷达数据
    for (int i = 0; i < NUM_PROBES; i++)
    {
        logFile << std::dec << ", " << radarDataEA[i];
    }
    
    // 记录E8雷达数据
    for (int i = 0; i < NUM_PROBES; i++)
    {
        logFile << std::dec << ", " << radarDataE8[i];
    }

    // 记录所有CAN帧
    for (size_t i = 0; i < can_frame_log.size(); i++)
    {
        logFile << ", " << can_frame_log[i].can_id << ", ";

        // 记录当前CAN帧的8字节数据
        for (int j = 0; j < 8; ++j)
        {
            logFile << std::setw(2) << std::setfill('0') << std::hex << (int)can_frame_log[i].data[j];
            if (j < 7)
                logFile << " ";
        }
    }

    // 换行
    logFile << std::endl;
    // 增加记录计数
    recordCounter++;
    // 每100条记录刷新一次缓冲区，确保数据写入磁盘
    if (recordCounter % FLUSH_INTERVAL == 0)
    {
        logFile.flush();
    }
}

/*地图记录线程函数*/
void *map_recorder_thread(void *arg)
{
    printf("地图记录线程已启动\n");
    
    pid_t child_pid = -1;
    bool process_running = false;
    
    while (1)
    {
        // 检查thread_mapRecord状态
        if (thread_mapRecord == 1) {
            // 如果进程还没有启动，启动3000P程序
            if (!process_running) {
                std::string filename;
                
                // 使用map_name创建文件名，如果map_name为空则使用默认名称
                if (!decision_control_in.map_name.empty()) {
                    filename = decision_control_in.map_name;
                } else {
                    // 使用时间戳作为默认文件名
                    time_t now = time(0);
                    struct tm *ltm = localtime(&now);
                    char timestamp[32];
                    sprintf(timestamp, "gps_path%04d%02d%02d%02d%02d%02d.txt",
                           1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday,
                           ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
                    filename = std::string(timestamp);
                }
                
                printf("启动3000P程序，文件名: %s\n", filename.c_str());
                
                // 创建子进程
                child_pid = fork();
                if (child_pid == 0) {
                    // 子进程：执行3000P程序
                    chdir("/home/ztl");
                    execl("./3000P_dynmaic", "3000P_dynmaic", filename.c_str(), (char*)NULL);
                    // 如果execl失败
                    printf("启动3000P程序失败\n");
                    exit(1);
                } else if (child_pid > 0) {
                    // 父进程：记录进程状态
                    process_running = true;
                    printf("3000P_dynmaic程序已启动，PID: %d\n", child_pid);
                } else {
                    // fork失败
                    printf("创建子进程失败\n");
                }
            }
            
            // 检查子进程是否还在运行
            if (process_running && child_pid > 0) {
                int status;
                pid_t result = waitpid(child_pid, &status, WNOHANG);
                if (result == child_pid) {
                    // 子进程已结束
                    printf("3000P程序已结束\n");
                    process_running = false;
                    child_pid = -1;
                }
            }
            
            // 每秒检查一次
            sleep(1);
        } else {
            // 如果thread_mapRecord不为1，终止3000P进程
            if (process_running && child_pid > 0) {
                printf("停止3000P程序\n");
                kill(child_pid, SIGTERM);
                
                // 等待子进程结束
                int status;
                waitpid(child_pid, &status, 0);
                
                printf("3000P程序已停止\n");
                process_running = false;
                child_pid = -1;
            }
            
            // 等待模式，每秒检查一次
            sleep(1);
        }
    }
    
    // 清理资源
    if (process_running && child_pid > 0) {
        kill(child_pid, SIGTERM);
        waitpid(child_pid, NULL, 0);
    }
    
    return NULL;
}

/**
 * @brief 雷达决策融合函数
 * @details 整合雷达数据处理和停车决策逻辑
 * @param serialFd2 雷达串口文件描述符
 * @return 无返回值，结果存储在全局radar_params中
 */
void processRadarDecision(int serialFd2)
{
    // 检查雷达功能是否启用
    if (!radar_params.enable_radar) {
        // 雷达功能关闭时，清除所有停车标志
        radar_params.should_stop_EA = false;
        radar_params.should_stop_E8 = false;
        radar_params.should_stop = false;
        radar_params.counter_EA = 0;
        radar_params.counter_E8 = 0;
        printf("雷达功能已关闭，跳过雷达决策\n");
        return;
    }

    printf("雷达功能已开启，执行雷达决策\n");
    
    /****************************
     * 雷达数据采集部分
     ****************************/
    //  发送固定数据（不计算校验和）
    std::vector<uint8_t> sendData1 = {0xEA, 0x2C, 0x01, 0xC7}; // 12个探头的数据
    sendSerialData(serialFd2, sendData1); 

    // 接收雷达数据
    std::vector<uint8_t> receivedData_EA = receiveSerialData(serialFd2, 26); // 接收26字节数据
    if (receivedData_EA.size() == 26) {
        parseReceivedData(receivedData_EA);
    } else {
        std::cerr << "Failed to read EA radar data" << std::endl;
        memset(radarDataEA, 0, sizeof(radarDataEA));  // 清空数据防止误判
    }
    
    // 休眠60毫秒
    usleep(60000);

    std::vector<uint8_t> sendData2 = {0xE8, 0x2C, 0x01, 0xC5}; // 8个探头的数据
    sendSerialData(serialFd2, sendData2);

    // 接收雷达数据
    std::vector<uint8_t> receivedData_E8 = receiveSerialData(serialFd2, 26); // 接收26字节数据
    if (receivedData_E8.size() == 26) {
        parseReceivedData(receivedData_E8);
    } else {
        std::cerr << "Failed to read E8 radar data" << std::endl;
        memset(radarDataE8, 0, sizeof(radarDataE8));  // 清空数据防止误判
    } 

    /****************************
     * 雷达决策逻辑部分
     ****************************/
    // 检查EA雷达各组探头
    bool obstacle_front_group = false; // 2,1,7,8探头（正前方）
    bool obstacle_other_group = false; // 其余探头

    // 检查正前方组（2,1,7,8探头，索引为1,0,6,7）
    int front_probes[] = {1, 0, 6, 7}; // 2,1,7,8探头对应的索引
    for (int i = 0; i < 4; ++i)
    {
        if (radarDataEA[front_probes[i]] < (radar_params.safe_dis + 200))
        {
            obstacle_front_group = true;
            break;
        }
    }

    // 检查其余探头（3,4,5,6,9,10,11,12探头，索引为2,3,4,5,8,9,10,11）
    int other_probes[] = {2, 3, 4, 5, 8, 9, 10, 11}; // 其余探头对应的索引
    for (int i = 0; i < 8; ++i)
    {
        if (radarDataEA[other_probes[i]] < radar_params.safe_dis)
        {
            obstacle_other_group = true;
            break;
        }
    }

    // 根据各组状态统一设置停车标志
    radar_params.should_stop_EA = obstacle_front_group || obstacle_other_group;

    // 检查 E8 数组中的数据是否全部成员都小于safe_dis安全距离
    bool all_e8_below_safe = false;
    for (int i = 0; i < NUM_PROBES; ++i) // E8
    {
        if (radarDataE8[i] < radar_params.safe_dis)
        {
            all_e8_below_safe = true;
            break;
        }
    }
    radar_params.should_stop_E8 = all_e8_below_safe;
    
    // 检查两个标识是否有一个为true
    if (radar_params.should_stop_EA || radar_params.should_stop_E8){
        radar_params.should_stop = true;  // 至少有一个为true，触发停车
    }
    else
    {
        radar_params.should_stop = false;  // 两个都为false，不触发停车
    }

    // 打印雷达决策状态
    printf("雷达决策状态: EA停车=%s, E8停车=%s, 总停车=%s\n", 
           radar_params.should_stop_EA ? "是" : "否",
           radar_params.should_stop_E8 ? "是" : "否", 
           radar_params.should_stop ? "是" : "否");
}

void *main_thread(void *arg)
{
    /****************************
     * 第一部分：初始化
     ****************************/
    printf("\nMain_Thread [%lu] is running\n", pthread_self());
    
    // 等待UDP接收完成（begin_run被设置为1）
    printf("主线程等待UDP数据接收完成 (begin_run = %d)...\n", begin_run);
    while (begin_run == 0) {
        // 短暂休眠，避免CPU占用过高
        usleep(100000); // 100ms
    }
    printf("UDP数据接收完成，主线程继续执行...\n");
    
    // 初始化优化预瞄距离计算器
    initializeLookaheadOptimizer();

    // 初始化变量
    double Ld = 0.0, alpha = 0.0;
    double steer_error = 0.0;  // 计算出跟踪车身所需角度
    double steer_angle = 0.0;  // 实际方向盘角度
    float steer_ratio = 16.36; // 转向比赋值：540 deg（方向盘）/33°（车身）=16.36
    double wheelbase = 2.0;    // 轴距

    if (decision_control_in.bStart == 1) // 自动驾驶启动时,默认启用GPS
    {
        enable_gps = true;
    }

    // 初始化串口（仅在GPS启用时）
    int serialFd = -1;
    if (enable_gps)
    {
        serialFd = configureSerialPort(SERIAL_PORT);
        if (serialFd == -1)
        {
            printf("\nSerialPort configure error!\n");
            return NULL;
        }
        std::cout << "Serial port " << SERIAL_PORT << " configured successfully. Listening for data..." << std::endl;
    }
    else
    {
        std::cout << "GPS功能已禁用，将使用模拟数据进行调试" << std::endl;
    }
    // 初始化雷达串口设备路径
    int serialFd2 = configureSerialPort2(RADAR_SERIAL_PORT);  
    if (serialFd2 == -1) {
        std::cerr << "Serial port configuration failed!" << std::endl;
        return NULL;
    }
    std::cout << "Radar thread: Serial port configured successfully. Listening for data..." << std::endl;




    cout_can = 0; // CAN初始化计数器

    // 初始化GPS数据处理相关变量
    static auto lastProcessTime = std::chrono::steady_clock::now();
    static int emptyDataCount = 0;
    static int totalDataCount = 0;

    // 主循环
    while (1)
    {
        /****************************
         * 第二部分：GPS数据获取
         ****************************/
        // 从CAN帧中获取车速，用于发送给决策结构体control_out
        control_out.vehicle_speed = revin_4a2.vehicle_speed;
        // 从CAN帧中获取SOC，用于发送给决策结构体control_out
        control_out.Soc_HIGH = revin_473.Bat_BatSOC;
        control_out.Soc_LOW = revin_471.LVBat_Volt;
        control_out.Emgy_brk_En = revin_451.Emrg_Sw_St;

        // 车速获取，用于更新车辆状态结构体cur_params
        cur_params.vehicle_speed = control_out.vehicle_speed;

        std::vector<double> real_gps;

        // 计算距离上次处理的时间间隔
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastProcessTime).count();

        // 检查GPS数据是否正常的标志
        static bool gps_initialized = false;
        static int gps_retry_count = 0;
        const int MAX_GPS_RETRY = 10; // 最大重试次数
        
        if (enable_gps)
        {
            // 每次读取前记录时间，确保处理间隔合理（GPS数据通常是10Hz，即100ms一次）
            if (elapsedMs >= 95)
            { // 稍微提前一点，确保不会错过数据
                lastProcessTime = currentTime;

                // 从串口读取实际GPS数据
                real_gps = readSerialData(serialFd);
                totalDataCount++;

                if (real_gps.empty())
                {
                    emptyDataCount++;
                    std::cerr << "无GPS数据或解析错误：No valid data received. 空数据率: "
                              << (double)emptyDataCount / totalDataCount * 100 << "%" << std::endl;
                    // 如果无法获取GPS数据，可以使用上一次的有效数据或默认值
                    
                    // 如果GPS尚未初始化，增加重试计数
                    if (!gps_initialized) {
                        gps_retry_count++;
                        printf("GPS数据初始化重试 %d/%d...\n", gps_retry_count, MAX_GPS_RETRY);
                        
                        if (gps_retry_count >= MAX_GPS_RETRY) {
                            printf("警告：GPS数据初始化失败，将使用模拟数据继续执行\n");
                            gps_initialized = true; // 强制继续执行
                        }
                    }
                }
                else
                {
                    // 更新车辆参数
                    cur_params.latitude = real_gps[0];
                    cur_params.longitude = real_gps[1];
                    cur_params.heading_angle = real_gps[2];
                    std::cout << "GPS数据处理间隔: " << elapsedMs << "ms" << std::endl;
                    printf("\n当前GPS三元组信息：latitude = %f, longitude = %f, heading_angle = %f，",
                           cur_params.latitude, cur_params.longitude, cur_params.heading_angle);
                    printf("当前车速：vehicle_speed = %f\n", cur_params.vehicle_speed);
                    
                    // 检查GPS数据是否有效
                    bool gps_valid = (cur_params.latitude >= 1.0 && cur_params.latitude <= 50.0 && 
                                     cur_params.longitude >= 1.0 && cur_params.longitude <= 120.0);
                    
                    if (gps_valid && !gps_initialized) {
                        printf("GPS数据初始化成功，程序正式启动\n");
                        gps_initialized = true;
                    }
                }
                
                // 如果GPS尚未初始化，暂停主循环的其他处理
                if (!gps_initialized) {
                    usleep(100000); // 100ms
                    continue; // 跳过本次循环的其余部分
                }
            }
        }
        else
        {
            // GPS功能禁用时，直接标记为已初始化
            gps_initialized = true;
            // 使用模拟数据进行调试
            // 这里可以设置固定的模拟值或者根据需要生成动态模拟数据
            cur_params.latitude = 39.20842796;   // 模拟纬度
            cur_params.longitude = 117.22737372; // 模拟经度
            cur_params.heading_angle = 160.0;    // 模拟航向角
            // 从CAN帧中获取车速
            control_out.vehicle_speed = revin_4a2.vehicle_speed;
            cur_params.vehicle_speed = control_out.vehicle_speed;

            printf("\n使用模拟GPS数据：latitude = %f, longitude = %f, heading_angle = %f，",
                   cur_params.latitude, cur_params.longitude, cur_params.heading_angle);
            printf("当前车速：vehicle_speed = %f\n", cur_params.vehicle_speed);
        }

        /****************************
         * 第三部分：雷达决策融合处理
         ****************************/
        // 调用雷达决策融合函数，根据enable_radar开关决定是否执行雷达逻辑
        processRadarDecision(serialFd2);

        /****************************
         * 第三部分：路径规划计算
         ***************************/
        pthread_mutex_lock(&my_mutex);

        // 检查GPS数据是否有效（纬度在1-50之间，经度在1-120之间）
        bool gps_valid = (cur_params.latitude >= 1.0 && cur_params.latitude <= 50.0 && 
                         cur_params.longitude >= 1.0 && cur_params.longitude <= 120.0);
        
        double Ld = 3.5; // 默认预瞄距离
        
        if (gps_valid) {
            // GPS数据有效，进行预瞄点计算
            // double Ld = obtainMapData();    //原有算法
            Ld = obtainMapData_optimized();
            
            // 模拟或低速时不进行转向
            if (cur_params.vehicle_speed < 0.1 || enable_gps == false)
            {
                steer_error = 0;
            }
            else
            {
                // 计算转向角度
                steer_error = GPS2Steer(cur_params.vehicle_speed, cur_params.latitude, cur_params.longitude,
                                        cur_params.heading_angle, GIS_map.latitude, GIS_map.longitude,
                                        wheelbase, Ld, alpha);
                // 计算横误差
                Puresuit_lateral_error = g_lookahead_optimizer.calculateLateralError(cur_params.latitude, cur_params.longitude,
                                                                                    GIS_map.latitude, GIS_map.longitude,
                                                                                    cur_params.heading_angle);
            }
        } else {
            // GPS数据无效，不进行预瞄点计算和转向
            std::cout << "警告：GPS数据无效，不进行预瞄点计算和转向控制" << std::endl;
            steer_error = 0;
        }

        // 计算实际方向盘角度
        steer_angle = steer_error * steer_ratio; // 实际角度=计算角度*转向比
        /* test */
        // steer_angle = 0;

        cur_params.steer_angle = steer_angle;

        pthread_mutex_unlock(&my_mutex);

        /****************************
         * 第四部分：决策控制赋值 test
         ****************************/
        printf("\n\nStart Can \n\n");
        int test = 0;
        if (test == 1)
        {
            // 为决策结构体的所有成员赋值 - 完整测试用例
            // 1. 启动标签
            decision_control_in.bStart = 1; // 启动标签：1表示自动驾驶启动 2表示地图录制启动

            // 2. 制动控制 (210)
            // decision_control_in.brake_bar = 0;        // 设置刹车压力为0bar
            decision_control_in.breaking_dis = 200.0; // 设置刹车距离为200m

            // 3. 驻车控制 (220)
            decision_control_in.EPB_park = 0; // 不启用驻车

            // 4. 驱动控制 (240)
            decision_control_in.gear = 9;       // 设置档位为1P 3R 5N 9D
            // decision_control_in.endSpeed = 0;
            
            
            // 5. 紧急控制 (251)
            decision_control_in.Emgy_brk_En = 0;           // 不启用急刹
            decision_control_in.Emgy_brk_ReqRmv = 0;       // 不移除急刹
            decision_control_in.Emgy_FtCrashRemove = 0;    // 不移除前触边
            decision_control_in.Emgy_RrrCrashRemove = 0;   // 不移除后触边
            decision_control_in.Emgy_LeftCrashRemove = 0;  // 不移除左触边
            decision_control_in.Emgy_RightCrashRemove = 0; // 不移除右触边

            // 6. 附件控制 (260)
            decision_control_in.ADU_Hom = 0;          // 不启用喇叭
            decision_control_in.ADU_BackLamp = 0;     // 不启用倒车灯
            decision_control_in.ADU_TurnRLamp = 0;    // 不启用右转灯
            decision_control_in.ADU_TurnLLamp = 0;    // 启用左转灯
            // decision_control_in.ADU_DblFlashLamp = 0; // 不启用双闪灯
            decision_control_in.ADU_LowBeamLamp = 0;  // 启用近光灯
            decision_control_in.ADU_WidthLamp = 1;    // 启用示宽灯
            decision_control_in.ADU_HighBeamLamp = 0; // 不启用远光灯////
            decision_control_in.ADU_FogLamp = 1;      // 不启用雾灯
            decision_control_in.ADU_BrkLamp = 0;      // 不启用制动灯

            //7. 地图信息
            decision_control_in.map_name = "path.txt";

            // 根据雷达停车逻辑设置速度和制动
            if (radar_params.should_stop)
            {
                decision_control_in.brake_bar = 100;
                decision_control_in.endSpeed = 0;
                // decision_control_in.Emgy_brk_En = 1;
                printf("\nSTATUS:Emergency Stop !\n");
            }
            else
            {
                decision_control_in.brake_bar = 0;
                decision_control_in.endSpeed = 3.5;
                decision_control_in.ADU_DblFlashLamp = 1;
                // decision_control_in.Emgy_brk_En = 0;
                // decision_control_in.Emgy_brk_ReqRmv=1;
                printf("\nSTATUS:Automatic !\n");
            }
            cout << "雷达决策：EndSpeed:" << decision_control_in.endSpeed << ", Emgy_brk_En:" << decision_control_in.Emgy_brk_En << ", Emgy_brk_ReqRmv=" << decision_control_in.Emgy_brk_ReqRmv << endl;

            // 打印测试参数
            printf("\n测试参数:\n");
            printf("转向角度: %.2f deg\n", steer_angle);
            printf("启动标签: %d\n", decision_control_in.bStart);
            printf("刹车压力: %.2f bar\n", decision_control_in.brake_bar);
            printf("刹车距离: %.2f m\n", decision_control_in.breaking_dis);
            printf("驻车使能: %d\n", decision_control_in.EPB_park);
            printf("档位设置: %d\n", decision_control_in.gear);
            printf("目标速度: %.2f km/h\n", decision_control_in.endSpeed);
            printf("急刹使能: %d\n", decision_control_in.Emgy_brk_En);
            printf("急刹请求移除: %d\n", decision_control_in.Emgy_brk_ReqRmv);
            printf("前触边移除: %d\n", decision_control_in.Emgy_FtCrashRemove);
            printf("后触边移除: %d\n", decision_control_in.Emgy_RrrCrashRemove);
            printf("左触边移除: %d\n", decision_control_in.Emgy_LeftCrashRemove);
            printf("右触边移除: %d\n", decision_control_in.Emgy_RightCrashRemove);
            printf("喇叭: %d\n", decision_control_in.ADU_Hom);
            printf("倒车灯: %d\n", decision_control_in.ADU_BackLamp);
            printf("左转灯: %d\n", decision_control_in.ADU_TurnLLamp);
            printf("右转灯: %d\n", decision_control_in.ADU_TurnRLamp);
            printf("双闪灯: %d\n", decision_control_in.ADU_DblFlashLamp);
            printf("近光灯: %d\n", decision_control_in.ADU_LowBeamLamp);
            printf("远光灯: %d\n", decision_control_in.ADU_HighBeamLamp);
            printf("雾灯: %d\n", decision_control_in.ADU_FogLamp);
            printf("制动灯: %d\n", decision_control_in.ADU_BrkLamp);
            printf("示宽灯: %d\n", decision_control_in.ADU_WidthLamp);
        } 
        convert_and_print_speed(decision_control_in.endSpeed); // 转为速度最小精度的倍数

        /****************************
         * 第五部分：CAN通信
         ****************************/
        // 重启标签处理
        if (cout_can == -2 && decision_control_in.bStart == 1)
        {
            std::cout << "-------------------RESETTING--------------------" << std::endl;
            cout_can = 0;
        }

        // 发送CAN数据 - 根据ControlState选择不同的控制模式
        if (ControlState == 2)
        {
            // RDC远程控制模式
            // send_candata(rdc_in, control_out, revin_4a2, revin_441, cout_can);
            printf("RDC模式 - can计数器：cout_can= %d\n", cout_can);
        }
        else
        {
            // 决策控制模式
            // send_candata(steer_angle, control_out, decision_control_in, revin_4a2, revin_441, cout_can);
            printf("决策模式 - can计数器：cout_can= %d\n", cout_can);
        }

        // 记录日志
        logToFile(decision_control_in, cur_params, GIS_map, Ld, alpha, can_frame_log,Puresuit_lateral_error);

        // 休眠20毫秒
        usleep(20000);
    }

    return NULL;
}

// CAN套接字健康检查函数
bool check_can_socket_health(int sockfd)
{
    if (sockfd < 0)
        return false;

    // 尝试获取接口状态，检查套接字是否有效
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, "can0");

    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl in health check");
        return false;
    }

    return true;
}

// CAN套接字监控线程函数
void *can_monitor_thread(void *arg)
{
    printf("CAN监控线程已启动\n");

    while (1)
    {
        // 检查thread_mapRecord状态，如果为1则进入空执行模式
        if (thread_mapRecord == 1) {
            // printf("CAN监控线程进入等待模式...\n");
            sleep(1);
            continue;
        }

        // 每5秒检查一次CAN套接字健康状态
        sleep(5);

        // 使用互斥锁保护全局套接字访问
        pthread_mutex_lock(&my_mutex);

        if (!check_can_socket_health(can_sockfd))
        {
            fprintf(stderr, "CAN套接字健康检查失败，尝试重新初始化...\n");

            // 关闭旧套接字
            if (can_sockfd >= 0)
            {
                close(can_sockfd);
            }

            // 重新初始化
            can_sockfd = init_can_socket("can0");
            if (can_sockfd < 0)
            {
                fprintf(stderr, "CAN套接字重新初始化失败!\n");
            }
            else
            {
                fprintf(stderr, "CAN套接字重新初始化成功\n");
            }
        }

        pthread_mutex_unlock(&my_mutex);
    }

    return NULL;
}

/*can发送线程创建*/
// 发送线程函数
void canSendThreadFunc(int socket_fd)
{
    while (running)
    {
        // 检查thread_mapRecord状态，如果为1则进入空执行模式
        if (thread_mapRecord == 1) {
            // printf("CAN发送线程进入等待模式...\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        std::vector<can_frame> framesToSend;
        {
            std::lock_guard<std::mutex> lock(framesMutex);
            framesToSend = eightFrames;
        }
        can_frame_log = framesToSend; // can结构体：日志记录使用
        if (!framesToSend.empty())
        {
            if (flag_canSend)
            {
                batch_send_frames(socket_fd, framesToSend.data(), framesToSend.size());
                flag_canSend = false;
            }
            else
            {
                batch_send_frames(socket_fd, framesToSend.data(), framesToSend.size());
            }
        }
    }
}

// 线程兼容函数封装给 pthread_create 调用
void *can_send_thread(void *arg)
{
    int socket_fd = *reinterpret_cast<int *>(arg);
    canSendThreadFunc(socket_fd);
    return nullptr;
}

int main()
{
    pthread_t threads[NUM_THREADS]; // 增加一个线程用于CAN监控
    int rc;
    long t;

    rev_ok = 0;
    rev_save_ok = 1;
    rev_first = 1;
    begin_run = 0;

    rev_count = 0;
    frame_count = 0;

    _target_ind = 0;

    // 初始化全局CAN socket
    can_sockfd = init_can_socket("can0");
    if (can_sockfd < 0)
    {
        fprintf(stderr, "初始CAN套接字初始化失败，将在5秒后重试...\n");
        sleep(5); // 等待5秒后重试

        can_sockfd = init_can_socket("can0");
        if (can_sockfd < 0)
        {
            fprintf(stderr, "重试CAN套接字初始化仍然失败，请检查硬件连接!\n");
            exit(EXIT_FAILURE);
        }
    }

    // 设置套接字为非阻塞模式
    int flags = fcntl(can_sockfd, F_GETFL, 0);
    fcntl(can_sockfd, F_SETFL, flags | O_NONBLOCK);

    printf("CAN套接字初始化成功，套接字描述符: %d\n", can_sockfd);

    // 启动CAN监控线程
    rc = pthread_create(&threads[0], NULL, can_monitor_thread, (void *)t);
    if (rc)
    {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }
    // 启动UDP线程
    rc = pthread_create(&threads[1], NULL, udp_thread, (void *)t);
    if(rc){
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }


    /*
    地图文件读取版本使用决策进行字符串传输
    // 控制自测test 关闭udp
    //test 通过传输字符串进行地图读取录制
    decision_control_in.map_name = "path.txt";
    decision_control_in.bStart = 1;
    thread_mapRecord = 0;

    // 构建地图文件路径
    string filePath = "/home/ztl/gps_path/" + decision_control_in.map_name;

    // 调试信息：显示地图文件路径
    cout << "尝试读取地图文件: " << filePath << endl;
    cout << "map_name: '" << decision_control_in.map_name << "'" << endl;

    // 检查map_name是否为空
    // 检查map_name是否为空或文件是否存在
    bool need_wait_for_udp = false;

    if (decision_control_in.map_name.empty())
    {
        cout << "map_name为空，程序将等待UDP数据传输地图信息" << endl;
        need_wait_for_udp = true;
    }
   else
   {
       // 检查文件是否存在
       ifstream file_check(filePath);
       if (!file_check.good())
       {
           cout << "警告：地图文件不存在或无法访问: " << filePath << endl;
           cout << "程序将等待UDP传输地图数据" << endl;
           need_wait_for_udp = true;
       }
       else
       {
           file_check.close();
           // 从文件中读取数据并存入map_latitude_v和map_longitude_v
           readMapDataFromFile(filePath);
       }
   }

   // 如果需要等待UDP数据，进入等待循环
   if (need_wait_for_udp == true && decision_control_in.bStart != 2)
   {
       cout << "自动驾驶模式：进入等待模式，等待UDP传输地图数据..." << endl;

       // 等待地图数据通过UDP接收
       while (map_latitude_v.empty() || map_longitude_v.empty())
       {
           // 短暂休眠，避免CPU占用过高
           usleep(100000); // 100ms

           // 可选：添加超时机制或其他退出条件
           // 例如：检查是否收到退出信号等
       }

       cout << "已接收到UDP地图数据，地图点数量: " << map_latitude_v.size() << endl;
   }
   else if (need_wait_for_udp == true && decision_control_in.bStart == 2)
   {
       cout << "地图录制模式" << endl;
   }
    */
    if (map_latitude_v.size() > 0 && map_longitude_v.size() > 0)
    {
        cout << "UDP map read successful!" << endl;
    }
    

    for (size_t i = 0; i < map_latitude_v.size(); ++i)
    {
        // 使用fixed和setprecision来确保输出8位小数
        cout << fixed << setprecision(8)
             << "Latitude: " << map_latitude_v[i]
             << ", Longitude: " << map_longitude_v[i] << endl;
    }
    // begin_run = 1; // 控制自测 test
    cout << "等待UDP数据接收完成: begin_run =" << begin_run << endl;
    
    // 等待begin_run变为1（UDP接收完成）
    printf("主程序等待UDP数据接收完成...\n");
    while (begin_run == 0) {
        // 短暂休眠，避免CPU占用过高
        usleep(100000); // 100ms
    }
    printf("UDP数据接收完成，开始创建其他线程...\n");

    // 创建其他线程
    printf("Creating CAN receiving thread\n");
    rc = pthread_create(&threads[2], NULL, can_receiving_thread, (void *)2);
    if (rc)
    {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }

    printf("Creating main control thread\n");
    rc = pthread_create(&threads[3], NULL, main_thread, (void *)3);
    if (rc)
    {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }

    printf("Creating CAN send thread\n");
    rc = pthread_create(&threads[4], NULL, can_send_thread, &can_sockfd);
    if (rc)
    {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }

    printf("Creating map recorder thread\n");
    rc = pthread_create(&threads[5], NULL, map_recorder_thread, (void *)5);
    if (rc)
    {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
        exit(-1);
    }

    printf("所有线程已创建，系统启动中...\n");

    // 打印线程状态
    printf("线程状态:\n");
    printf("- 线程0: CAN监控线程\n");
    printf("- 线程1: UDP通信线程\n");
    printf("- 线程2: CAN接收线程\n");
    printf("- 线程3: 主控制线程\n");
    printf("- 线程4: CAN发送线程\n");
    printf("- 线程5: 地图记录线程\n");

    struct itimerval tick;
    signal(SIGALRM, handle_sigalrm); // 设置信号处理函数

    // 初始化定时器
    tick.it_value.tv_sec = 0;
    tick.it_value.tv_usec = 20 * 1000; // 20毫秒
    tick.it_interval = tick.it_value;  // 设置重复周期

    // 启动定时器
    if (setitimer(ITIMER_REAL, &tick, NULL) < 0)
    {
        perror("setitimer");
        return 1;
    }

    // 等待所有线程完成
    // 注意：现在有NUM_THREADS+1个线程（增加了CAN监控线程）
    for (t = 0; t < NUM_THREADS; t++)
    {
        printf("等待线程 %ld 结束...\n", t);
        pthread_join(threads[t], NULL);
        printf("线程 %ld 已结束\n", t);
    }

    // 程序结束前清理资源
    // 1. 关闭CAN socket
    if (can_sockfd >= 0)
    {
        close(can_sockfd);
        can_sockfd = -1;
    }

    // 2. 关闭日志文件
    if (logFile.is_open())
    {
        logFile.close();
    }

    // 3. 清空全局数组，避免退出时的内存问题
    eightFrames.clear();

    // 4. 清空容器
    can_frame_log.clear();
    map_latitude_v.clear();
    map_longitude_v.clear();

    std::cout << "程序正常退出，资源已清理" << std::endl;
    return 0;
}
