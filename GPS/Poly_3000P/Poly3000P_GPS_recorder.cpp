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
    catch (const std::invalid_argument& e) {
        std::cerr << "转换失败：字段格式不正确：" << e.what() << std::endl;
        return false;
    }
    catch (const std::out_of_range& e) {
        std::cerr << "转换失败：值超出范围：" << e.what() << std::endl;
        return false;
    }
    return true;
}

void readSerialData(int fd)
{
    char buffer[256];
    std::string sentence_acc;
    double latitude = 0.0, longitude = 0.0, heading = 0.0;
    int status = 0;
    std::vector<double> res;

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
            // 检查文件是否打开
            if (outFile.is_open())
            {
                outFile << "Latitude: " << latitude << ", Longitude: " << longitude
                        << ", Heading: " << heading << ", Status: " << status << std::endl;
            }
            else
            {
                std::cerr << "无法写入文件：outFile 未打开。" << std::endl;
            }
            res.push_back(latitude);
            res.push_back(longitude);
            res.push_back(heading);
            res.push_back(static_cast<double>(status));
            // return res;
        }
        else
        {
            std::cerr << "Invalid GPNAV sentence: " << fullSentence << std::endl;
            // 如果解析失败，可返回一组特殊值或继续循环
            res = {0, 0, 0, 0};
            // return res;
        }
    }

    // 若退出循环未成功，返回空 vector
    // return res;
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
