cmake_minimum_required(VERSION 3.5)
project(latlon_muxer)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/node.cpp)

autoware_set_compile_options(${PROJECT_NAME})
# TODO(latlon_muxer) : RCLCPP_ERROR_THROTTLE() has built-in old-style casts.
# TODO(latlon_muxer) : Temporary workaround until this is fixed.
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "tmp::LatLonMuxer" EXECUTABLE ${PROJECT_NAME}_exe)


ament_auto_package()
