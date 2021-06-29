cmake_minimum_required(VERSION 3.5)
project(lidar_centerpoint)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if (CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if (CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif ()
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif (CUDA_FOUND)




# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVONNXPARSER nvonnxparser)
find_library(NVCAFFE_PARSER NAMES nvcaffe_parser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVPARSERS AND NVCAFFE_PARSER AND NVINFER_PLUGIN)
  if (CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVPARSERS: ${NVPARSERS}")
    message("NVCAFFE_PARSER: ${NVCAFFE_PARSER}")
  endif ()
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

option(TORCH_AVAIL "Torch available" OFF)
set(Torch_DIR /usr/local/libtorch/share/cmake/Torch)
find_package(Torch)
if (TORCH_FOUND)
  if (VERBOSE)
    message(STATUS "TORCH_INCLUDE_DIRS: ${TORCH_INCLUDE_DIRS}")
    message(STATUS "TORCH_LIBRARIES: ${TORCH_LIBRARIES}")
  endif ()
  set(TORCH_AVAIL ON)
else()
  message("Torch NOT FOUND")
  set(TORCH_AVAIL OFF)
endif (TORCH_FOUND)

if(TRT_AVAIL AND CUDA_AVAIL AND TORCH_AVAIL)
# Download trained models
    find_program(GDOWN_AVAIL "gdown")
    if (NOT GDOWN_AVAIL)
        message("gdown: command not found. External files could not be downloaded.")
    endif ()

    set(DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data)
    execute_process(COMMAND mkdir -p ${DATA_PATH})

    function(download FILE_NAME GFILE_ID FILE_HASH)
        message(STATUS "Checking and downloading ${FILE_NAME}")
        set(FILE_PATH ${DATA_PATH}/${FILE_NAME})
        if (EXISTS ${FILE_PATH})
            file(MD5 ${FILE_PATH} EXISTING_FILE_HASH)
            if (NOT ${FILE_HASH} EQUAL ${EXISTING_FILE_HASH})
                message(STATUS "... file hash changes. Downloading now ...")
                execute_process(COMMAND gdown --quiet https://drive.google.com//uc?id=${GFILE_ID} -O ${FILE_PATH})
            endif()
        else()
            message(STATUS "... file doesn't exists. Downloading now ...")
            execute_process(COMMAND gdown --quiet https://drive.google.com//uc?id=${GFILE_ID} -O ${FILE_PATH})
        endif ()
    endfunction()
    download(centerpoint_vfe.pt 1NUOn3GILXO2uTVjONBHRZYLKLiLirBpK 825dc48c799d05b3785ac314afe49c0b)
    download(centerpoint_head.pt 1WxXtxKYWkhwW90MRHnHxqa1QkHu_bjFk 3164deaa33f3ec456379b53137808384)
    download(centerpoint_vfe.onnx 1TsNwIYVPat-rZxRRVhW7DpNWu2W7srox 14cc03fab1670267f1872edfa1b83a48)
    download(centerpoint_head.onnx 1nmdNpmnhHa15VsoCaHoh0oreL6pDux_k d27881e2c72363e2cd765358f9f479bc)

    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    include_directories(
        lib/include
        ${CUDA_INCLUDE_DIRS}
        ${TORCH_INCLUDE_DIRS}
    )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_MWAITXINTRIN_H_INCLUDED")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORCE_INLINES")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__STRICT_ANSI__")

    ### centerpoint ###
    ament_auto_add_library(
            centerpoint SHARED
            lib/src/pointcloud_densification.cpp
            lib/src/voxel_generator.cpp
            lib/src/centerpoint_trt.cpp
            lib/src/tensorrt_wrapper.cpp
            lib/src/network_trt.cpp)

    target_link_libraries(
            centerpoint
            ${NVINFER}
            ${NVCAFFE_PARSER}
            ${NVONNXPARSER}
            ${NVINFER_PLUGIN}
            ${CUDA_LIBRARIES}
            ${CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${TORCH_LIBRARIES}
    )

    ## node ##
    ament_auto_add_library(
            lidar_centerpoint_component SHARED
            # src/main.cpp
            src/node.cpp
    )

    target_link_libraries(
            lidar_centerpoint_component
            centerpoint
    )

    ament_auto_add_executable(
        lidar_centerpoint_node_main
        src/main.cpp
    )

    target_link_libraries(
        lidar_centerpoint_node_main
        lidar_centerpoint_component
    )
    rclcpp_components_register_node(lidar_centerpoint_component
        PLUGIN "centerpoint::LidarCenterPointNode"
        EXECUTABLE lidar_centerpoint_node
    )
    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()

    ament_auto_package(INSTALL_TO_SHARE
        launch
        data
    )
else()
    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()
    ament_auto_package(INSTALL_TO_SHARE
        launch
    )
endif()
