cmake_minimum_required(VERSION 3.5)
project(rosbag2_moriyama_converter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(dataset_converter dataset_converter.cpp)

autoware_set_compile_options(dataset_converter)
ament_target_dependencies(dataset_converter rclcpp nmea_msgs sensor_msgs)

if(BUILD_TESTING)
    # run linters
    autoware_static_code_analysis()
endif()

install(TARGETS
  dataset_converter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
