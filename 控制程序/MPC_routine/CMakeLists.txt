cmake_minimum_required(VERSION 3.10)

# 项目信息
project(MPCAlgorithm VERSION 1.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加包含路径
include_directories(/usr/local/include/GeographicLib)
include_directories(/usr/include/eigen3) 
include_directories(/usr/include/coin)
include_directories(/usr/include/cppad)

# 添加库搜索路径
link_directories(/usr/local/lib)

# 添加源文件
set(SOURCES 
    MPC.cpp
    MPC_demo.cpp
)

# 可执行文件目标
add_executable(MPCDemo ${SOURCES})

# 链接库
target_link_libraries(MPCDemo PRIVATE 
    GeographicLib 
    ipopt 
    pthread
)

# 编译选项
target_compile_options(MPCDemo PRIVATE -O2 -Wall -Wno-sign-compare)

# 可选：设置 CMAKE_INCLUDE_PATH 来确保找到标准库头文件
set(CMAKE_INCLUDE_PATH "/usr/include:/usr/local/include")

# 添加安装目标
install(TARGETS MPCDemo DESTINATION bin)
install(FILES MPC.h DESTINATION include)