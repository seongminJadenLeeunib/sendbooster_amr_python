#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sendbooster_bringup/serial_port.hpp"
#include <cmath>
#include <vector>
#include <chrono>
#include <sstream>

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode()
        : Node("motor_control_node"), serialPort_("/dev/ttyUSB0")
    {
        serialPort_.setBaudRate(19200);

        // 시리얼 포트 열기
        if (!serialPort_.openPort())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open motor serial port");
            rclcpp::shutdown();
        }

        // cmd_vel 구독 설정
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MotorControlNode::motor_command_from_ros2, this, std::placeholders::_1));

        // motor_rpm 토픽 발행
        motor_rpm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_rpm", 10);

        // 초기화 패킷 전송
        motor_driver_data_request_command({183, 184, 1, 10, 1, 10});

        // 0.5초 주기 타이머로 패킷 전송 및 데이터 읽기
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MotorControlNode::receive_hall_sensor_data, this));
    }

    ~MotorControlNode()
    {
        serialPort_.closePort();
    }

private:
    void motor_command_from_ros2(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // cmd_vel 메시지를 통해 모터 RPM 계산
        float linear_x = msg->linear.x;
        float angular_z = msg->angular.z;
        const float wheel_base = 0.745;   // 바퀴 간 거리
        const float wheel_radius = 0.095; // 바퀴 반지름
        const int gear_ratio = 10;        // 기어비 1:10

        // 기어비를 고려하여 RPM 계산
        int rpm1 = static_cast<int>(((linear_x - (angular_z * wheel_base / 2)) / (2 * M_PI * wheel_radius)) * -60 * gear_ratio);
        int rpm2 = static_cast<int>(((linear_x + (angular_z * wheel_base / 2)) / (2 * M_PI * wheel_radius)) * 60 * gear_ratio);

        motor_movement_command(rpm1, rpm2);

        // motor_rpm 토픽에 왼쪽과 오른쪽 RPM 값 발행
        std_msgs::msg::Float32MultiArray rpm_msg;
        rpm_msg.data.push_back(rpm1);
        rpm_msg.data.push_back(rpm2);
        motor_rpm_pub_->publish(rpm_msg);

        RCLCPP_INFO(this->get_logger(), "Published motor RPMs - Left: %d, Right: %d", rpm1, rpm2);
    }

    void receive_hall_sensor_data()
    {
        // 데이터 읽기
        motor_driver_data_request_command({183, 184, 1, 4, 1, 216});
        read_data();
    }

    void motor_movement_command(int rpm1, int rpm2)
    {
        uint8_t rpm1_low = rpm1 & 0xFF;
        uint8_t rpm1_high = (rpm1 >> 8) & 0xFF;
        uint8_t rpm2_low = rpm2 & 0xFF;
        uint8_t rpm2_high = (rpm2 >> 8) & 0xFF;

        std::vector<int8_t> packet = {183, 184, 1, 207, 7, 1, rpm1_low, rpm1_high, 1, rpm2_low, rpm2_high, 0};
        uint8_t checksum = calculate_checksum(packet);
        packet.push_back(checksum);

        // 전송되는 바이트 배열 출력 (디버깅용)
        std::ostringstream oss;
        oss << "Sending packet: ";
        for (size_t i = 0; i < packet.size() - 1; ++i)
        {
            oss << "0x" << std::hex << static_cast<int>(packet[i]) << " ";
        }
        RCLCPP_DEBUG(this->get_logger(), oss.str().c_str());

        // 시리얼 포트를 통해 bytearray 전송
        serialPort_.writeData_bytearray(packet);
    }

    void motor_driver_data_request_command(const std::vector<int8_t>& packet_data)
    {
        // 입력된 패킷 데이터로 체크섬 계산 후 패킷 전송
        std::vector<int8_t> packet = packet_data;
        uint8_t checksum = calculate_checksum(packet);
        packet.push_back(checksum);

        // 디버깅용 패킷 출력
        std::ostringstream oss;
        oss << "Sending packet: ";
        for (auto byte : packet)
            oss << "0x" << std::hex << static_cast<int>(byte) << " ";
        RCLCPP_DEBUG(this->get_logger(), oss.str().c_str());

        // 패킷 전송
        serialPort_.writeData_bytearray(packet);
    }

    uint8_t calculate_checksum(const std::vector<int8_t>& packet)
    {
        uint8_t checksum_value = 0;
        for (const auto& byte : packet)
            checksum_value += byte;

        return (~checksum_value + 1) & 0xFF;
    }

    void read_data()
    {
        auto data = serialPort_.readData();
        RCLCPP_INFO(this->get_logger(), "Received data size: %ld", data.size());

        // 원시 데이터 출력
        std::stringstream raw_data_stream;
        RCLCPP_DEBUG(this->get_logger(), "Raw data: %s", raw_data_stream.str().c_str());

        if (data.size() == 20) // 20바이트의 패킷이 제대로 수신된 경우
        {
            // 모터1 속도 (D5, D6) -> int16_t (부호 있는 16비트 값)
            int16_t motor1_speed = static_cast<int16_t>(data[5]) | (static_cast<uint8_t>(data[6]) << 8);

            // 모터1 상태변수 (D7) -> uint8_t (상태 값)
            uint8_t motor1_status = data[7];

            // 모터1 위치 (D8~D11) -> int32_t (부호 있는 32비트 값)
            int32_t motor1_position = static_cast<int8_t>(data[8]) |
                                       (static_cast<uint8_t>(data[9]) << 8) |
                                       (static_cast<uint8_t>(data[10]) << 16) |
                                       (static_cast<uint8_t>(data[11]) << 24);

            // 모터2 속도 (D12, D13) -> int16_t (부호 있는 16비트 값)
            int16_t motor2_speed = static_cast<int16_t>(data[12]) | (static_cast<uint8_t>(data[13]) << 8);

            // 모터2 상태변수 (D14) -> uint8_t (상태 값)
            uint8_t motor2_status = data[14];

            // 모터2 위치 (D15~D18) -> int32_t (부호 있는 32비트 값)
            int32_t motor2_position = static_cast<int8_t>(data[15]) |
                                       (static_cast<uint8_t>(data[16]) << 8) |
                                       (static_cast<uint8_t>(data[17]) << 16) |
                                       (static_cast<uint8_t>(data[18]) << 24);

            // 출력
            RCLCPP_INFO(this->get_logger(),
                        "Motor1 - Speed: %d, Status: %d, Position: %d | Motor2 - Speed: %d, Status: %d, Position: %d",
                        motor1_speed, motor1_status, motor1_position, motor2_speed, motor2_status, motor2_position);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Incomplete or invalid data received");
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_rpm_pub_;
    serialPort serialPort_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
