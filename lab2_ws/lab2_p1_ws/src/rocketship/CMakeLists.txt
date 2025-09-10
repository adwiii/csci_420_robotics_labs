cmake_minimum_required(VERSION 3.14.4)
project(rocketship)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin and any catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(src)

add_executable(countdown src/countdown_node.cpp)
target_include_directories(countdown PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(countdown PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS})
install(TARGETS countdown
  DESTINATION lib/${PROJECT_NAME})

add_executable(main_controller src/main_controller_node.cpp)
target_include_directories(main_controller PUBLIC
 "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
 "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(main_controller PUBLIC
rclcpp::rclcpp
${std_msgs_TARGETS})
install(TARGETS main_controller
DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  src/rocket_engine.py
  DESTINATION lib/rocketship )

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()