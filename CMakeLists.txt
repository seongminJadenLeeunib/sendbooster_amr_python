cmake_minimum_required(VERSION 3.5)
project(sendbooster_bringup)

# C++ 표준 설정
add_compile_options(-std=c++17)
include_directories(
  include
)
# 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 실행 파일 추가
add_executable(motor_driver src/motor_driver.cpp)
add_executable(imu_node src/imu_node.cpp)

# 타겟에 의존성 연결
ament_target_dependencies(motor_driver rclcpp std_msgs geometry_msgs)
ament_target_dependencies(imu_node rclcpp sensor_msgs)

# 실행 파일 설치
install(TARGETS
  motor_driver
  imu_node
  DESTINATION lib/${PROJECT_NAME})

# 패키지 설정
ament_package()
