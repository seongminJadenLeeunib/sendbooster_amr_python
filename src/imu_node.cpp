#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "sendbooster_bringup/serial_port.hpp" // 사용자 정의 SerialPort 클래스 헤더 포함

using namespace std::chrono_literals;

class IMUPublisher : public rclcpp::Node {
public:
    IMUPublisher()
        : Node("imu_node"), serialPort_("/dev/ttyUSB0") {
        
        serialPort_.setBaudRate(B115200);
        
        // 시리얼 포트 열기
        if (!serialPort_.openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
            rclcpp::shutdown();
            return;
        }

        // IMU 데이터 요청 명령 전송
        std::string dataToSend = "ss=4\n"; // 센서 초기화 명령
        if (!serialPort_.writeData(dataToSend)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data to the sensor!");
            rclcpp::shutdown();
            return;
        }

        // IMU 데이터 게시자 설정
        imuPublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/sendbooster_imu", 10);

        // 타이머로 데이터 읽기 및 게시 반복
        timer_ = this->create_wall_timer(100ms, std::bind(&IMUPublisher::readAndPublishIMUData, this));
    }

private:
    void readAndPublishIMUData() {
        // 센서 데이터 읽기
        std::string receivedData = serialPort_.readData();
        if (receivedData.empty()) {
            RCLCPP_WARN(this->get_logger(), "No data received from IMU.");
            return;
        }

        // 원시 데이터 출력 (디버깅용)
        RCLCPP_INFO(this->get_logger(), "Raw IMU data: %s", receivedData.c_str());

        // 데이터 파싱: 한 줄씩 처리하여 롤, 피치, 요를 추출
        std::istringstream iss(receivedData);
        float roll, pitch, yaw;
        std::string line;

        while (std::getline(iss, line)) {
            // 공백만 있는 줄은 건너뛰기
            if (line.empty() || line.find_first_not_of(" \t\n") == std::string::npos) {
                continue;
            }

            std::istringstream lineStream(line);
            if (lineStream >> roll >> pitch >> yaw) {
                // Float32MultiArray 메시지 생성
                auto imuMsg = std_msgs::msg::Float32MultiArray();
                imuMsg.data = {roll, pitch, yaw}; // Roll, Pitch, Yaw 데이터 배열 추가

                // 메시지 게시
                imuPublisher_->publish(imuMsg);

                RCLCPP_INFO(this->get_logger(), "IMU data published: Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
                            roll, pitch, yaw);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse IMU data line: %s", line.c_str());
            }
        }
    }

    serialPort serialPort_; // 시리얼 포트 객체
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr imuPublisher_; // IMU 데이터 게시자
    rclcpp::TimerBase::SharedPtr timer_; // 타이머
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}
