cmake_minimum_required(VERSION 3.10)
project(MainControl VERSION 1.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================
# 编译优化选项：性能最大化
# ============================

# 默认使用Release构建
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 添加Release模式的编译器优化参数
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -DNDEBUG -flto")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -flto")

# ============================
# 包/依赖查找
# ============================

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)
pkg_check_modules(CPPAD REQUIRED cppad)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib CONFIG REQUIRED)

# ============================
# 包含目录
# ============================

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${IPOPT_INCLUDE_DIRS})
include_directories(${CPPAD_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/usr/local/include/GeographicLib)

# ============================
# 源文件
# ============================

set(SOURCES
    src/control.cpp
    src/optimal_lookahead_calculator.cpp
)

# ============================
# 可执行文件目标
# ============================

add_executable(MainControl ${SOURCES})

# ============================
# 链接库
# ============================

target_link_libraries(MainControl
    ${IPOPT_LIBRARIES}
    ${CPPAD_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${GeographicLib_LIBRARIES}
    pthread
)

# ============================
# 输出路径设置
# ============================

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)
