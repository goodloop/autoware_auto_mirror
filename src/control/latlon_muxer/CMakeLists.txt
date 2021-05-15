cmake_minimum_required(VERSION 3.5)
project(latlon_muxer)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/latlon_muxer/node.hpp
  src/node.cpp
)

autoware_set_compile_options(${PROJECT_NAME})
# TODO(latlon_muxer) : RCLCPP_ERROR_THROTTLE() has built-in old-style casts.
# TODO(latlon_muxer) : Temporary workaround until this is fixed.
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "autoware::motion::control::latlon_muxer::LatLonMuxer" EXECUTABLE ${PROJECT_NAME}_exe)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Unit tests
  set(LATLON_MUXER_TEST test_latlon_muxer_node)
  # TODO(latlon_muxer) : Fails to link the node library.
  # TODO(latlon_muxer) : Temporary workaround: compile the node src with the tests
  ament_add_gtest(${LATLON_MUXER_TEST} test/test_latlon_muxer_node.cpp src/node.cpp)
  target_compile_options(${LATLON_MUXER_TEST} PRIVATE -Wno-error=old-style-cast)
  autoware_set_compile_options(${LATLON_MUXER_TEST})
  target_link_libraries(${LATLON_MUXER_TEST} ${PROJECT_NAME})
endif()

ament_auto_package()
