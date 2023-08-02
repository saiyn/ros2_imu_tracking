cmake_minimum_required(VERSION 3.5)
project(imu_integrator)

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
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(Imu_Integrator_node src/Imu_Integrator.cpp)
ament_target_dependencies(Imu_Integrator_node rclcpp std_msgs visualization_msgs)

install(TARGETS
  Imu_Integrator_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()