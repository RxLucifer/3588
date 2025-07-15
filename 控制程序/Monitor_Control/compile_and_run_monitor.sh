#!/bin/bash

# 编译监控程序
g++ -o monitor_maincontrol monitor_maincontrol.cpp -std=c++11

# 检查编译是否成功
if [ $? -ne 0 ]; then
    echo "编译失败！"
    exit 1
fi

echo "编译成功，正在启动监控程序..."

# 运行监控程序
./monitor_maincontrol