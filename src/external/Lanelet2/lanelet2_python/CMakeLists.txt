cmake_minimum_required(VERSION 3.5.1)
project(lanelet2_python)

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

  mrt_glob_files(PROJECT_PYTHON_SOURCE_FILES_SRC "python_api/*.cpp")
  if (PROJECT_PYTHON_SOURCE_FILES_SRC)
      # Add a cpp-python api library. Make sure there are no name collisions with python modules in this project
      mrt_add_python_api( lanelet2
          FILES ${PROJECT_PYTHON_SOURCE_FILES_SRC}
          )
  endif()

  ############################
  ## Read source code files ##
  ############################
  mrt_glob_files_recurse(PROJECT_HEADER_FILES_INC "include/*.h" "include/*.hpp" "include/*.cuh")
  mrt_glob_files(PROJECT_SOURCE_FILES_INC "src/*.h" "src/*.hpp" "src/*.cuh")
  mrt_glob_files(PROJECT_SOURCE_FILES_SRC "src/*.cpp" "src/*.cu")

  ###########
  ## Build ##
  ###########
  # Declare a cpp library
  mrt_add_library(${PROJECT_NAME}
      INCLUDES ${PROJECT_HEADER_FILES_INC} ${PROJECT_SOURCE_FILES_INC}
      SOURCES ${PROJECT_SOURCE_FILES_SRC}
      )

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
  find_package(Boost REQUIRED COMPONENTS python3)
  ament_auto_find_build_dependencies()

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # You can add a custom.cmake in order to add special handling for this package. E.g. you can do:
  # list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
  # To remove libs which cannot be found automatically. You can also "find_package" other, custom dependencies there.
  # You can also set PROJECT_INSTALL_FILES to install files that are not installed by default.
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
  endif()

  # Build and install a python module for each file
  file(GLOB PROJECT_PYTHON_SOURCE_FILES_SRC "python_api/*.cpp")
  set(PYTHON_API_MODULE_NAME "lanelet2")
  set(PYTHON_MODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${PYTHON_API_MODULE_NAME}")
  set(TEMPLATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/templates")

  foreach(API_FILE ${PROJECT_PYTHON_SOURCE_FILES_SRC})
    get_filename_component(SUBMODULE_NAME ${API_FILE} NAME_WE)
    set(TARGET_NAME "${PROJECT_NAME}-${PYTHON_API_MODULE_NAME}-${SUBMODULE_NAME}-pyapi")
    set(LIBRARY_NAME ${SUBMODULE_NAME})

    ament_auto_add_library(${TARGET_NAME} SHARED ${API_FILE})
    ament_target_dependencies(${TARGET_NAME} Boost)
    target_include_directories(${TARGET_NAME} PUBLIC "/usr/include/python3.6")
    target_link_libraries(${TARGET_NAME} Eigen3::Eigen)
    set_target_properties(${TARGET_NAME} PROPERTIES OUTPUT_NAME ${LIBRARY_NAME} PREFIX "")
    set_target_properties(
      ${TARGET_NAME}
      PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PYTHON_MODULE_DIR}"
                 LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PYTHON_MODULE_DIR}"
                 LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PYTHON_MODULE_DIR}"
                 LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${PYTHON_MODULE_DIR}"
                 LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PYTHON_MODULE_DIR}"
    )
    target_compile_definitions(${TARGET_NAME} PRIVATE PYTHON_API_MODULE_NAME=${LIBRARY_NAME})
  endforeach()

  configure_file(${TEMPLATE_DIR}/__init__.py.in ${PYTHON_MODULE_DIR}/__init__.py)
  configure_file(${TEMPLATE_DIR}/setup.py.in ${PYTHON_MODULE_DIR}/setup.py)
  message("Directory: ${PYTHON_MODULE_DIR}")
  ament_python_install_package(${PYTHON_API_MODULE_NAME})

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
      ament_add_pytest_test(${TEST_FILENAME}_test ${TEST_FILE})
    endforeach()
  endif()

  ament_auto_package()
endif()
