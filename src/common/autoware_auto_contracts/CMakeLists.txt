cmake_minimum_required(VERSION 3.5)
project(autoware_auto_contracts)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(contracts_lite_vendor REQUIRED)
find_package(autoware_auto_common REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  "rclcpp"
  "contracts_lite_vendor"
  "autoware_auto_common")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  set(BUILD_DEFINITIONS
    -DCONTRACT_VIOLATION_CONTINUATION_MODE_ON # Options: _OFF, _ON
    -DCONTRACT_BUILD_LEVEL_DEFAULT # Options: _OFF, _DEFAULT, _AUDIT
  )

  # Unit tests
  set(TEST_AUTOWARE_AUTO_CONTRACTS_EXE test_autoware_auto_contracts)
  ament_add_gtest(${TEST_AUTOWARE_AUTO_CONTRACTS_EXE}
    test/test_scalar_flicker.cpp
    test/test_acute_degree.cpp
    test/test_acute_radian.cpp
    test/test_nonnegative_real.cpp
    test/test_real.cpp
    test/test_size_bound.cpp
    test/test_strictly_eps_positive_real.cpp
    test/test_strictly_positive_real.cpp
    test/test_dimensional_analysis.cpp
    test/gtest_main.cpp)
  target_include_directories(${TEST_AUTOWARE_AUTO_CONTRACTS_EXE} PRIVATE include)
  target_compile_definitions(${TEST_AUTOWARE_AUTO_CONTRACTS_EXE} PRIVATE ${BUILD_DEFINITIONS})
  ament_target_dependencies(${TEST_AUTOWARE_AUTO_CONTRACTS_EXE}
    "rclcpp"
    "contracts_lite_vendor"
    "autoware_auto_common")
endif()

ament_package()
