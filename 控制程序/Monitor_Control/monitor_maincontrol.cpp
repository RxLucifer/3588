#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>
#include <csignal>
#include <sys/wait.h>

// 定义信号文件路径
const char* SIGNAL_FILE = "/tmp/maincontrol_restart_signal";

// 全局变量，用于存储MainControl进程的PID
pid_t maincontrol_pid = -1;

// 信号处理函数
void signal_handler(int sig) {
    if (sig == SIGTERM || sig == SIGINT) {
        std::cout << "监控程序收到终止信号，正在清理..." << std::endl;
        
        // 如果MainControl进程正在运行，则终止它
        if (maincontrol_pid > 0) {
            std::cout << "终止MainControl进程 (PID: " << maincontrol_pid << ")" << std::endl;
            // 使用pkill命令终止所有MainControl进程
            std::cout << "执行pkill MainControl命令..." << std::endl;
            system("pkill MainControl");
            
            // 等待进程终止
            int status;
            waitpid(maincontrol_pid, &status, 0);
        }
        
        // 删除信号文件
        unlink(SIGNAL_FILE);
        
        // 退出监控程序
        exit(0);
    }
}

// 启动MainControl进程
pid_t start_maincontrol() {
    // 先杀死所有可能存在的MainControl进程
    std::cout << "执行pkill MainControl命令，确保没有旧进程运行..." << std::endl;
    system("pkill MainControl");
    
    // 等待一小段时间确保进程已经终止
    std::cout << "等待2秒确保所有旧进程已终止..." << std::endl;
    
    pid_t pid = fork();
    
    if (pid < 0) {
        // fork失败
        std::cerr << "创建子进程失败: " << strerror(errno) << std::endl;
        return -1;
    } else if (pid == 0) {
        // 子进程
        // 切换到MainControl目录
        if (chdir("/home/ztl/mainControl_Test/build") == -1) {
            std::cerr << "切换目录失败: " << strerror(errno) << std::endl;
            exit(EXIT_FAILURE);
        }
        
        // 执行MainControl程序
        execl("./MainControl", "MainControl", NULL);
        
        // 如果execl返回，说明出错了
        std::cerr << "启动MainControl失败: " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    } else {
        // 父进程
        std::cout << "MainControl进程已启动，PID: " << pid << std::endl;
        return pid;
    }
}

// 检查进程是否存在
bool is_process_running(pid_t pid) {
    // 使用/proc目录检查进程是否存在
    char proc_path[64];
    snprintf(proc_path, sizeof(proc_path), "/proc/%d", pid);
    
    // 如果/proc/[pid]目录存在，则进程存在
    struct stat st;
    return stat(proc_path, &st) == 0;
}

// 主函数
int main() {
    // 设置信号处理
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    
    std::cout << "MainControl监控程序启动" << std::endl;
    
    // 创建信号文件（如果不存在）
    int fd = open(SIGNAL_FILE, O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
        std::cerr << "无法创建信号文件: " << strerror(errno) << std::endl;
        return 1;
    }
    close(fd);
    
    // 启动MainControl进程
    maincontrol_pid = start_maincontrol();
    if (maincontrol_pid < 0) {
        std::cerr << "启动MainControl失败" << std::endl;
        return 1;
    }
    
    // 监控循环
    while (true) {
        // 检查信号文件是否被修改
        struct stat file_stat;
        if (stat(SIGNAL_FILE, &file_stat) == 0) {
            // 读取信号文件内容
            std::ifstream signal_file(SIGNAL_FILE);
            std::string signal_content;
            if (signal_file.is_open()) {
                std::getline(signal_file, signal_content);
                signal_file.close();
                
                // 如果文件内容为"restart"，则重启MainControl
                if (signal_content == "restart") {
                    std::cout << "收到重启信号，正在重启MainControl..." << std::endl;
                    
                    // 终止当前MainControl进程
                    if (maincontrol_pid > 0 && is_process_running(maincontrol_pid)) {
                        std::cout << "终止当前MainControl进程 (PID: " << maincontrol_pid << ")" << std::endl;
                        // 使用pkill命令终止所有MainControl进程
                        std::cout << "执行pkill MainControl命令..." << std::endl;
                        system("pkill MainControl");
                        
                        // 等待进程终止
                        int status;
                        waitpid(maincontrol_pid, &status, 0);
                    }
                    
                    // 等待10秒，确保资源完全释放
                    std::cout << "等待10秒后启动新进程..." << std::endl;
                    sleep(10);
                    
                    // 启动新的MainControl进程
                    maincontrol_pid = start_maincontrol();
                    if (maincontrol_pid < 0) {
                        std::cerr << "重启MainControl失败" << std::endl;
                        return 1;
                    }
                    
                    // 清空信号文件
                    std::ofstream clear_file(SIGNAL_FILE, std::ios::trunc);
                    clear_file.close();
                }
            }
        }
        
        // 检查MainControl进程是否仍在运行
        if (maincontrol_pid > 0 && !is_process_running(maincontrol_pid)) {
            std::cout << "MainControl进程已终止，正在重启..." << std::endl;
            
            // 等待10秒，确保资源完全释放
            std::cout << "等待10秒后启动新进程..." << std::endl;
            sleep(10);
            
            // 启动新的MainControl进程
            maincontrol_pid = start_maincontrol();
            if (maincontrol_pid < 0) {
                std::cerr << "重启MainControl失败" << std::endl;
                return 1;
            }
        }
        
        // 休眠1秒
        sleep(1);
    }
    
    return 0;
}