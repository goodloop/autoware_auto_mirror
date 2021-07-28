cmake_minimum_required(VERSION 3.5)
project(autoware_state_monitor)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build
set(STATE_MONITOR_SRC
  src/autoware_state_monitor_node/autoware_state_monitor_node.cpp
  src/autoware_state_monitor_node/state_machine.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${STATE_MONITOR_SRC})
autoware_set_compile_options(${PROJECT_NAME})

ament_auto_add_executable(${PROJECT_NAME}_exe
  src/autoware_state_monitor_node/main.cpp
)
autoware_set_compile_options(${PROJECT_NAME}_exe)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)