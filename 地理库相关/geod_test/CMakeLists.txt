cmake_minimum_required(VERSION 3.10)

# 项目信息
project(MyProject VERSION 1.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加包含路径
include_directories(/usr/local/include/GeographicLib)

# 添加库搜索路径
link_directories(/usr/local/lib)

# 添加源文件
#file(GLOB SOURCES main.cpp)

# 可执行文件目标
add_executable(example main.cpp)

# 链接库
target_link_libraries(example PRIVATE GeographicLib)
