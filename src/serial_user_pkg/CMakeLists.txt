cmake_minimum_required(VERSION 3.8)
project(serial_user_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(serial_manager_pkg REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(serial_listener 
src/listen.cpp
)

ament_target_dependencies(serial_listener
rclcpp
rclcpp_components
serial_manager_pkg
)

install(TARGETS
  serial_listener
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(serial_talker 
src/talk.cpp
)

ament_target_dependencies(serial_talker
rclcpp
rclcpp_components
serial_manager_pkg
sensor_msgs
)

install(TARGETS
  serial_talker
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
