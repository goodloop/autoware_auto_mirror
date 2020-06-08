cmake_minimum_required(VERSION 3.5.1)
project(lanelet2_examples)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)
  set(MRT_PKG_VERSION 4.0.0)
  # Modify only if you know what you are doing!
  ###################
  ## Find packages ##
  ###################
  find_package(mrt_cmake_modules REQUIRED)
  include(UseMrtStdCompilerFlags)
  include(GatherDeps)

  # You can add a custom.cmake in order to add special handling for this package. E.g. you can do:
  # list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
  # To remove libs which cannot be found automatically. You can also "find_package" other, custom dependencies there.
  # You can also set PROJECT_INSTALL_FILES to install files that are not installed by default.
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
      include("${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
  endif()

  find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})

  mrt_parse_package_xml()

  ########################
  ## Add python modules ##
  ########################
  # This adds a python module if located under src/{PROJECT_NAME)
  mrt_python_module_setup()

  ###########
  ## Build ##
  ###########
  glob_folders(SRC_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/src")
  if (SRC_DIRECTORIES)
      # Found subfolders, add executable for each subfolder
      foreach(SRC_DIR ${SRC_DIRECTORIES})
          mrt_add_executable(${SRC_DIR} FOLDER "src/${SRC_DIR}")
      endforeach()
  else()
      # No subfolder found, add executable and python modules for src folder
      mrt_add_executable(${PROJECT_NAME} FOLDER "src")
  endif()

  #############
  ## Install ##
  #############
  # Install all targets, headers by default and scripts and other files if specified (folders or files).
  # This command also exports libraries and config files for dependent packages and this supersedes catkin_package.
  mrt_install(PROGRAMS scripts FILES res data ${PROJECT_INSTALL_FILES})

  #############
  ## Testing ##
  #############
  # Add test targets for cpp and python tests
  if (CATKIN_ENABLE_TESTING)
      mrt_add_tests(test)
      mrt_add_nosetests(test)
  endif()
elseif(${ROS_VERSION} EQUAL 2)
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  foreach(SRC_DIR ${SRC_DIRECTORIES})
    ament_auto_add_executable(${SRC_DOR} FOLDER "src/${SRC_DIR}")
  endforeach()

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # Add gtest tests
    file(GLOB TEST_FILES_SRC "test/*.cpp")

    foreach(TEST_FILE ${TEST_FILES_SRC})
      get_filename_component(TEST_FILENAME ${TEST_FILE} NAME_WE)
      ament_add_gtest(${TEST_FILENAME}_test ${TEST_FILE})
      target_link_libraries(${TEST_FILENAME}_test
        ${${PROJECT_NAME}_LIBRARIES}
        Eigen3::Eigen
      )
      add_dependencies(${TEST_FILENAME}_test ${PROJECT_NAME})
    endforeach()

    # Add pytest tests
    file(GLOB TEST_FILES_PY "test/*.py")

    foreach(TEST_FILE ${TEST_FILES_PY})
      get_filename_component(TEST_FILENAME ${TEST_FILE} NAME_WE)
      ament_add_pytest_test(${TEST_FILENAME}_test ${TEST_FILE} PYTHON_EXECUTABLE "/usr/bin/python")
    endforeach()
  endif()

  ament_auto_package()
endif()
