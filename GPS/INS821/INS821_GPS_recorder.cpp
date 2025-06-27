#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <iomanip>
#include <cstring>
#include <ctime>
#include <chrono>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/types.h>

volatile sig_atomic_t keepRunning = 1;

void intHandler(int) {
    keepRunning = 0;
}

// Parse a raw byte frame into {latitude, longitude, heading, status}
std::vector<double> parseData(std::vector<uint8_t>& data) {
    double headingAngle = 0.0, longitudeData = 0.0, latitudeData = 0.0;
    int status = -1;
    std::vector<double> res(4, 0.0);

    // Find frame header BD DB 0B
    size_t frameStartIndex = 0;
    bool frameFound = false;
    for (size_t i = 0; i + 2 < data.size(); ++i) {
        if (data[i] == 0xBD && data[i+1] == 0xDB && data[i+2] == 0x0B) {
            frameStartIndex = i;
            frameFound = true;
            break;
        }
    }

    if (frameFound && frameStartIndex + 92 <= data.size()) {
        std::vector<uint8_t> frame(data.begin() + frameStartIndex, data.begin() + frameStartIndex + 92);
        uint16_t headingRaw = (frame[8] << 8) | frame[7];
        headingAngle = headingRaw * 360.0 / 32768.0;
        uint64_t latRaw = 0;
        for (int j = 0; j < 8; ++j) latRaw |= static_cast<uint64_t>(frame[21 + j]) << (8 * j);
        latitudeData = latRaw * 1e-8;
        uint64_t lonRaw = 0;
        for (int j = 0; j < 8; ++j) lonRaw |= static_cast<uint64_t>(frame[29 + j]) << (8 * j);
        longitudeData = lonRaw * 1e-8;
        status = frame[47];

        res[0] = latitudeData;
        res[1] = longitudeData;
        res[2] = headingAngle;
        res[3] = static_cast<double>(status);
    }
    return res;
}

// Read from serial and return parsed values; returns {0,0,0,0} if no new frame
std::vector<double> readSerialData(int serialPort) {
    static std::vector<uint8_t> buffer;
    char tmp[1024];
    ssize_t n = read(serialPort, tmp, sizeof(tmp));
    if (n > 0) {
        buffer.insert(buffer.end(), tmp, tmp + n);
        if (buffer.size() > 2000) buffer.erase(buffer.begin(), buffer.begin() + (buffer.size() - 2000));
    }
    for (size_t i = 0; i + 92 <= buffer.size(); ++i) {
        if (buffer[i] == 0xBD && buffer[i+1] == 0xDB && buffer[i+2] == 0x0B) {
            std::vector<uint8_t> frame(buffer.begin() + i, buffer.begin() + i + 92);
            auto res = parseData(frame);
            buffer.erase(buffer.begin(), buffer.begin() + i + 92);
            return res;
        }
    }
    return std::vector<double>(4, 0.0);
}

int main() {
    signal(SIGINT, intHandler);

    const char* device = "/dev/ttyS0";
    int serialPort = open(device, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serialPort < 0) {
        std::cerr << "Error opening serial port " << device << std::endl;
        return 1;
    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serialPort, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        close(serialPort);
        return 1;
    }
    cfsetispeed(&tty, B460800);
    cfsetospeed(&tty, B460800);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tcsetattr(serialPort, TCSANOW, &tty);

    // Prepare output directory
    const char* outDir = "/home/ztl/gps_path";
    mkdir(outDir, 0777);

    // Timestamped filename
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);
    char fname[64];
    std::strftime(fname, sizeof(fname), "gps_path%Y%m%d%H%M%S.txt", &tm);
    char filepath[128];
    std::snprintf(filepath, sizeof(filepath), "%s/%s", outDir, fname);

    std::ofstream outFile(filepath);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open output file: " << filepath << std::endl;
        close(serialPort);
        return 1;
    }
    std::cout << "Logging GPS data to " << filepath << std::endl;

    while (keepRunning) {
        auto data = readSerialData(serialPort);
        if (data[0] == 0.0 && data[1] == 0.0 && data[2] == 0.0 && data[3] == 0.0) {
            usleep(100000);
            continue;
        }
        // Write to file
        outFile << "Latitude: " << std::fixed << std::setprecision(8) << data[0]
                << ", Longitude: " << data[1]
                << ", Heading: " << data[2]
                << ", Status: " << static_cast<int>(data[3])
                << std::endl;
        outFile.flush();
        // Print current recorded point
        std::cout << "Recorded Point -> Latitude: " << std::fixed << std::setprecision(8) << data[0]
                  << ", Longitude: " << data[1]
                  << ", Heading: " << data[2]
                  << ", Status: " << static_cast<int>(data[3])
                  << std::endl;
        usleep(100000);
    }

    std::cout << "Shutting down, closing file." << std::endl;
    outFile.close();
    close(serialPort);
    return 0;
}
