cmake_minimum_required(VERSION 3.5)
project(ap_custom_msg_subscribers)

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
find_package(ap_custom_interfaces REQUIRED)

add_executable(num_listener src/APNumSubscriber.cpp)
ament_target_dependencies(num_listener rclcpp ap_custom_interfaces)

add_executable(baro_listener src/APBaroSubscriber.cpp)
ament_target_dependencies(baro_listener rclcpp ap_custom_interfaces)

add_executable(gps_listener src/APGPSSubscriber.cpp)
ament_target_dependencies(gps_listener rclcpp ap_custom_interfaces)

add_executable(compass_listener src/APCompassSubscriber.cpp)
ament_target_dependencies(compass_listener rclcpp ap_custom_interfaces)

add_executable(ins_listener src/APINSSubscriber.cpp)
ament_target_dependencies(ins_listener rclcpp ap_custom_interfaces)

install(TARGETS
  num_listener
  baro_listener
  gps_listener
  compass_listener
  ins_listener
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
