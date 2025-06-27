# cmake_minimum_required(VERSION 3.10)

# # 项目信息
# project(MyProject VERSION 1.0 LANGUAGES CXX)

# # 设置 C++ 标准
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# # 添加包含路径
# include_directories(/usr/local/include/GeographicLib)
# include_directories(/usr/include/eigen3) 
# include_directories(/usr/include/coin)
# # 添加库搜索路径
# link_directories(/usr/local/lib)

# # 添加源文件
# #file(GLOB SOURCES main.cpp)

# # 可执行文件目标
# add_executable(MainControl control.cpp)

# # 链接库
# target_link_libraries(MainControl PRIVATE GeographicLib pthread ipopt)


# # 可选：设置 CMAKE_INCLUDE_PATH 来确保找到标准库头文件
# set(CMAKE_INCLUDE_PATH "/usr/include:/usr/local/include")

cmake_minimum_required(VERSION 3.10)
project(MainControl VERSION 1.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)  # 或17，取决于您的需求
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必要的包
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)
pkg_check_modules(CPPAD REQUIRED cppad)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib CONFIG REQUIRED)

# 添加包含路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${IPOPT_INCLUDE_DIRS})
include_directories(${CPPAD_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
# include_directories(${GeographicLib_INCLUDE_DIRS})
include_directories(/usr/local/include/GeographicLib)

# 添加源文件
set(SOURCES
    src/control.cpp
    src/optimal_lookahead_calculator.cpp
)

# 可执行文件目标
add_executable(MainControl ${SOURCES})

# 另一个可执行文件目标（MPC版本）
# add_executable(MPCControl src/control_HY_0529_MPC.cpp src/optimal_lookahead_calculator.cpp)

# 测试预瞄距离计算的可执行文件
# add_executable(test_Ld src/test_lookahead.cpp src/optimal_lookahead_calculator.cpp)

# 链接库
target_link_libraries(MainControl
    ${IPOPT_LIBRARIES}
    ${CPPAD_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${GeographicLib_LIBRARIES}
    pthread
)

# target_link_libraries(MPCControl
#     ${IPOPT_LIBRARIES}
#     ${CPPAD_LIBRARIES}
#     ${EIGEN3_LIBRARIES}
#     ${GeographicLib_LIBRARIES}
#     pthread
# )

#target_link_libraries(test_Ld
#    ${IPOPT_LIBRARIES}
#    ${CPPAD_LIBRARIES}
#    ${EIGEN3_LIBRARIES}
#    ${GeographicLib_LIBRARIES}
#    pthread
#)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)