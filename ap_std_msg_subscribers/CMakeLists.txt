cmake_minimum_required(VERSION 3.5)
project(ap_std_msg_subscribers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bool_listener src/APBoolSubscriber.cpp)
ament_target_dependencies(bool_listener rclcpp std_msgs)

add_executable(byte_listener src/APByteSubscriber.cpp)
ament_target_dependencies(byte_listener rclcpp std_msgs)

add_executable(char_listener src/APCharSubscriber.cpp)
ament_target_dependencies(char_listener rclcpp std_msgs)

add_executable(int8_listener src/APInt8Subscriber.cpp)
ament_target_dependencies(int8_listener rclcpp std_msgs)

add_executable(uint8_listener src/APUInt8Subscriber.cpp)
ament_target_dependencies(uint8_listener rclcpp std_msgs)

add_executable(int16_listener src/APInt16Subscriber.cpp)
ament_target_dependencies(int16_listener rclcpp std_msgs)

add_executable(uint16_listener src/APUInt16Subscriber.cpp)
ament_target_dependencies(uint16_listener rclcpp std_msgs)

add_executable(int32_listener src/APInt32Subscriber.cpp)
ament_target_dependencies(int32_listener rclcpp std_msgs)

add_executable(uint32_listener src/APUInt32Subscriber.cpp)
ament_target_dependencies(uint32_listener rclcpp std_msgs)

add_executable(int64_listener src/APInt64Subscriber.cpp)
ament_target_dependencies(int64_listener rclcpp std_msgs)

add_executable(uint64_listener src/APUInt64Subscriber.cpp)
ament_target_dependencies(uint64_listener rclcpp std_msgs)
                                    
add_executable(float32_listener src/APFloat32Subscriber.cpp)
ament_target_dependencies(float32_listener rclcpp std_msgs)

add_executable(float64_listener src/APFloat64Subscriber.cpp)
ament_target_dependencies(float64_listener rclcpp std_msgs)

add_executable(string_listener src/APStringSubscriber.cpp)
ament_target_dependencies(string_listener rclcpp std_msgs)

add_executable(time_listener src/APTimeSubscriber.cpp)
ament_target_dependencies(time_listener rclcpp std_msgs)

install(TARGETS
  bool_listener
  byte_listener
  int8_listener
  uint8_listener
  int16_listener
  uint16_listener
  int32_listener
  uint32_listener
  int64_listener
  uint64_listener
  float32_listener
  float64_listener
  char_listener
  string_listener
  time_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
