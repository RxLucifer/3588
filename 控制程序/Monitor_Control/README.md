# MainControl程序监控系统

本系统包含以下组件：

1. `run_maincontrol.sh` - 用于直接运行MainControl程序的简单脚本
2. `monitor_maincontrol.cpp` - 监控程序，用于监控MainControl进程并在需要时重启它
3. `compile_and_run_monitor.sh` - 编译并运行监控程序的脚本

## 修改说明

已对`control_HY_0711_test.cpp`文件进行了修改：

- 删除了原有的自启动代码
- 添加了发送重启信号的代码，当`cout_can > 600`且满足其他条件时，会向监控程序发送重启信号

## 使用方法

### 1. 将文件复制到Ubuntu系统

将以下文件复制到Ubuntu系统中：

- `run_maincontrol.sh`
- `monitor_maincontrol.cpp`
- `compile_and_run_monitor.sh`
- 修改后的`control_HY_XXXX.cpp`

### 2. 编译修改后的MainControl程序

```bash
# 进入MainControl源码目录
cd /home/ztl/mainControl_Test

# 编译程序
make
```

### 3. 设置脚本权限

```bash
chmod +x run_maincontrol.sh
chmod +x compile_and_run_monitor.sh
```

### 4. 启动监控程序

```bash
./compile_and_run_monitor.sh
```

## 工作原理

1. 监控程序启动后，会立即启动MainControl程序
2. 监控程序会定期检查MainControl进程是否仍在运行，如果不在运行则自动重启
3. 当MainControl程序中的条件触发时（`cout_can > 600`等），会向`/tmp/maincontrol_restart_signal`文件写入"restart"信号
4. 监控程序检测到信号后，会终止当前的MainControl进程并启动一个新的实例

## 注意事项

1. 监控程序会创建`/tmp/maincontrol_restart_signal`文件用于信号传递
2. 确保运行监控程序的用户有权限访问和修改该文件
3. 监控程序会捕获SIGTERM和SIGINT信号，在收到这些信号时会先终止MainControl进程再退出