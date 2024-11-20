#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <vector>
#include <cstring>

class serialPort {
public:
    // 생성자: 포트 이름을 설정
    explicit serialPort(const std::string &portName)
        : portName_(portName), fd_(-1), baudRate_(B19200) {}

    // 소멸자: 포트 닫기
    ~serialPort() {
        closePort();
    }

    // 보드레이트를 설정하는 메서드
    void setBaudRate(speed_t baudRate) {
        baudRate_ = baudRate;
    }

    // 포트를 열고 설정
    bool openPort() {
        fd_ = open(portName_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            std::cerr << "Failed to open serial port: " << portName_ << std::endl;
            return false;
        }

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "Error getting terminal attributes." << std::endl;
            close(fd_);
            return false;
        }

        // 설정된 보드레이트 사용
        cfsetospeed(&tty, baudRate_);
        cfsetispeed(&tty, baudRate_);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8비트 데이터
        tty.c_iflag &= ~IGNBRK; // 브레이크 처리 비활성화
        tty.c_lflag = 0; // Canonical 모드 비활성화
        tty.c_oflag = 0; // 출력 프로세싱 비활성화
        tty.c_cc[VMIN] = 1; // 최소 읽기 크기
        tty.c_cc[VTIME] = 1; // 읽기 타임아웃 (단위: 1/10초)

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 소프트웨어 흐름 제어 비활성화
        tty.c_cflag |= (CLOCAL | CREAD); // 로컬 연결 및 읽기 활성화
        tty.c_cflag &= ~(PARENB | PARODD); // 패리티 비활성화
        tty.c_cflag &= ~CSTOPB; // 1 스톱 비트
        tty.c_cflag &= ~CRTSCTS; // 하드웨어 흐름 제어 비활성화

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting terminal attributes." << std::endl;
            close(fd_);
            return false;
        }

        return true;
    }

    // 포트 닫기
    void closePort() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    // 데이터 쓰기 (std::string 버전)
    bool writeData(const std::string &data) {
        if (fd_ < 0) {
            std::cerr << "Serial port not open." << std::endl;
            return false;
        }

        ssize_t written = write(fd_, data.c_str(), data.size());
        if (written != static_cast<ssize_t>(data.size())) {
            std::cerr << "Error writing to serial port." << std::endl;
            return false;
        }

        return true;
    }

    // 데이터 쓰기 (std::vector<uint8_t> 버전)
    bool writeData_bytearray(const std::vector<uint8_t>& data) {
        if (fd_ < 0) {
            std::cerr << "Serial port not open." << std::endl;
            return false;
        }

        ssize_t bytes_written = write(fd_, data.data(), data.size());
        if (bytes_written != static_cast<ssize_t>(data.size())) {
            std::cerr << "Error writing to serial port." << std::endl;
            return false;
        }

        return true;
    }

    // 데이터 읽기
    std::string readData() {
        if (fd_ < 0) {
            std::cerr << "Serial port not open." << std::endl;
            return "";
        }

        char buffer[256];  // 큰 버퍼로 데이터를 수신
        int bytesRead = 0;
        int packetLength = 20;  // 패킷 길이 20바이트

        // 수신된 데이터를 계속 읽어서 20바이트 패킷이 올 때까지 기다린다.
        while (true) {
            bytesRead = read(fd_, buffer, sizeof(buffer));

            if (bytesRead < 0) {
                std::cerr << "Error reading from serial port." << std::endl;
                return "";
            }

            // 수신된 데이터가 20바이트 이상의 패킷을 포함한 경우
            if (bytesRead == packetLength) {
                int8_t *packet = reinterpret_cast<int8_t *>(buffer);

                // 20바이트 패킷을 그대로 반환
                return std::string(reinterpret_cast<char *>(packet), packetLength);
            }
        }
    }




private:
    std::string portName_;  // 포트 이름
    int fd_;                // 파일 디스크립터
    speed_t baudRate_;      // 보드레이트 (기본값: 19200)
};

#endif // SERIAL_PORT_HPP
