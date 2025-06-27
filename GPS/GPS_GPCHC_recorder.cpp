#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <iomanip> // 用于设置输出精度

#define SERIAL_PORT "/dev/ttyS7" // 串口设备路径
#define BAUD_RATE B115200         // 波特率设置为 115200

// 解析 $GPCHC 报文并提取所需字段
bool parseGPCHC(const std::string& sentence, double& latitude, double& longitude, double& heading, int& status) {
    if (sentence.find("$GPCHC") != 0) {
        return false;
    }

    std::vector<std::string> fields;
    std::stringstream ss(sentence);
    std::string field;

    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

    if (fields.size() < 24) {
        std::cerr << "Incomplete $GPCHC sentence: " << sentence << std::endl;
        return false;
    }

    try {
        heading = std::stod(fields[3]);    // 偏航角
        latitude = std::stod(fields[12]); // 纬度
        longitude = std::stod(fields[13]);// 经度
        status = std::stoi(fields[21]);   // 状态码
    } catch (const std::exception& e) {
        std::cerr << "Error parsing $GPCHC sentence: " << e.what() << std::endl;
        return false;
    }

    return true;
}

// 获取当前时间字符串用于文件命名
std::string getCurrentTimeString() {
    time_t now = time(0);
    tm* localtm = localtime(&now);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "gps_path%Y%m%d%H%M%S.txt", localtm);
    return std::string(buffer);
}

// 配置串口
int configureSerialPort(const std::string& port) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);  // 去掉 O_NDELAY，使用阻塞模式
    if (fd == -1) {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error getting serial attributes: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 数据位：8
    tty.c_cflag &= ~PARENB;  // 无校验位
    tty.c_cflag &= ~CSTOPB;  // 停止位：1
    tty.c_cflag |= CREAD | CLOCAL;  // 启用接收器，忽略调制解调器线路状态

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 禁用规范模式
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // 禁用软件流控制
    tty.c_oflag &= ~OPOST;  // 输出原始模式

    tty.c_cc[VMIN] = 1;  // 最小读取字符数
    tty.c_cc[VTIME] = 1; // 超时：0.1秒

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

// 从串口读取数据并保存到文件
void readSerialData(int fd) {
    char buffer[256];
    std::string sentence;
    std::vector<uint8_t> dataBuffer;  // 用来积累接收到的数据
    
    double latitude = 0.0, longitude = 0.0, heading = 0.0, vehicleSpeed = 0.0;
    int insFlag = 0;

    // 创建存储路径
    std::string directory = "/home/ztl/gps_path/";
    if (mkdir(directory.c_str(), 0777) && errno != EEXIST) {
        std::cerr << "Failed to create directory: " << directory << std::endl;
        return;
    }

    // 创建文件
    std::string filePath = directory + getCurrentTimeString();
    std::ofstream outFile(filePath);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return;
    }

    std::cout << "Saving data to file: " << filePath << std::endl;

    // 设置输出精度：纬度和经度保留小数点后 8 位
    outFile << std::fixed << std::setprecision(8);

    // 设置屏幕输出精度：纬度和经度保留小数点后 8 位
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
        } else if (bytesRead > 0) {
            // 将读取到的字节数据添加到缓冲区
            for (int i = 0; i < bytesRead; ++i) {
                dataBuffer.push_back(static_cast<uint8_t>(buffer[i]));
            }

            // 检查缓冲区中的数据是否达到93字节
            while (dataBuffer.size() >= 93) {
                // 截取前93字节并进行处理
                std::vector<uint8_t> completeData(dataBuffer.begin(), dataBuffer.begin() + 93);
                
                // 从缓冲区中移除已处理的数据
                dataBuffer.erase(dataBuffer.begin(), dataBuffer.begin() + 93);
                
                // 解析数据
                std::vector<double> parsedData = processHexData(completeData);

                // 检查解析结果并写入文件
                if (!parsedData.empty()) {
                    outFile << "Latitude: " << parsedData[0] << ", Longitude: " << parsedData[1]
                            << ", Heading: " << parsedData[2] << ", insFlag: " << parsedData[3]
                            << ", VehicleSpeed: " << parsedData[4] << std::endl;
                }

                // 输出到控制台
                std::cout << "Latitude: " << parsedData[0] << ", Longitude: " << parsedData[1]
                          << ", Heading: " << parsedData[2] << ", insFlag: " << parsedData[3]
                          << ", VehicleSpeed: " << parsedData[4] << std::endl;
            }
        }
    }

    outFile.close();
    std::cout << "Data saved to file: " << filePath << std::endl;
}

int main() {
    int serialFd = configureSerialPort(SERIAL_PORT);
    if (serialFd == -1) {
        return EXIT_FAILURE;
    }

    std::cout << "Serial port " << SERIAL_PORT << " configured successfully. Listening for data..." << std::endl;

    readSerialData(serialFd);

    close(serialFd);
    return EXIT_SUCCESS;
}
