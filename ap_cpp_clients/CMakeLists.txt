cmake_minimum_required(VERSION 3.8)
project(ap_cpp_clients)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ap_custom_services REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(arm_motors_client src/arming_motor_client.cpp)
ament_target_dependencies(arm_motors_client
  rclcpp ap_custom_services) 

add_executable(mode_select_client src/mode_select_client.cpp)
ament_target_dependencies(mode_select_client
    rclcpp ap_custom_services) 

install(TARGETS
  arm_motors_client
  mode_select_client
  DESTINATION lib/${PROJECT_NAME})

  ament_package()
