#!/usr/bin/expect

# 定义 root 密码
set password "123456"

# 切换到 root 模式
spawn su
expect "Password:"
send "$password\r"

# 打开串口ttyS0
send "chmod 777 /dev/ttyS0\r"

# 执行关闭 can0 设备
send "ip link set can0 down\r"
send "echo '关闭 can0 设备...'\r"

# 设置 can0 比特率为 500Kbps
send "ip link set can0 type can bitrate 500000\r"
send "echo '设置 can0 比特率为 500Kbps...'\r"

# 打开 can0 设备
send "ip link set can0 up\r"
send "echo '打开 can0 设备...'\r"

# 退出 root 模式
send "exit\r"
expect eof
