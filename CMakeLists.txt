cmake_minimum_required(VERSION 2.8)
project(realsense_yolo)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS "-Wall -Wno-unused-result -Wno-unknown-pragmas -Wno-unused-variable -Wfatal-errors -fPIC ${CMAKE_C_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  message_generation
  tf
  yolov4_trt_ros
  message_filters
  image_transport
)

message("Use spencer")
add_definitions(-DWITH_SPENCER)

# set(USE_GPU OFF CACHE BOOL "Build Tensorflow with GPU support")
set(USE_SPENCER ON CACHE BOOL "Publising spencer message")
if(USE_SPENCER)
  message("Use spencer")
  add_definitions(-DWITH_SPENCER)
  find_package(catkin REQUIRED COMPONENTS spencer_tracking_msgs)
endif(USE_SPENCER)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/libopencv3")
find_package(OpenCV 3.4.11 REQUIRED)
message("--!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--")
message("-- OpenCV version: ${OpenCV_VERSION}")
message("-- OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message("-- catkin include dirs: ${catkin_INCLUDE_DIRS}")
message("--!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--")

find_package(CUDA REQUIRED)
if (CUDA_FOUND)
  find_package(CUDA)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies from realsense_yolo: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
  )
  add_definitions(-DGPU)
else()
  list(APPEND LIBRARIES "m")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES debug_yolo.msg BoundingBoxes3d.msg BoundingBox3d.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realsense_yolo
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime message_filters image_transport cv_bridge
   LIBRARIES
    ${PROJECT_NAME}_lib
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## add_dependencies -> add_executable -> set_target_properties
if (CUDA_FOUND)

  link_directories(
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
  )

  if(FALSE)
  cuda_add_library(${PROJECT_NAME}_lib
    ${PROJECT_LIB_FILES} ${DARKNET_CORE_FILES}
    ${DARKNET_CUDA_FILES}
  )

  target_link_libraries(${PROJECT_NAME}_lib
    cuda
    cudart
    cublas
    curand
  )
  endif()

  cuda_add_executable(realsense_yolo src/realsense_yolo_node.cpp src/realsense_yolo.cpp)
  target_link_libraries(realsense_yolo ${catkin_LIBRARIES} ${OpenCV_LIBS} /opt/ros/noetic/lib/libmessage_filters.so /opt/ros/$ENV{ROS_DISTRO}/lib/libcv_bridge.so)

  cuda_add_executable(mobilessd_adaptor src/mobilessd_adaptor.cpp src/realsense_yolo.cpp)
  target_link_libraries(mobilessd_adaptor ${catkin_LIBRARIES} ${OpenCV_LIBS})

  cuda_add_executable(mobilessd_adaptor_tensorRT src/mobilenetssd_tensorRT_adaptor.cpp src/realsense_yolo.cpp)
  target_link_libraries(mobilessd_adaptor_tensorRT ${catkin_LIBRARIES} ${OpenCV_LIBS})

  cuda_add_executable(yolov4_adaptor_tensorRT src/yolov4_tensorRT_adaptor.cpp src/realsense_yolo.cpp)
  target_link_libraries(yolov4_adaptor_tensorRT ${catkin_LIBRARIES} ${OpenCV_LIBS})

else()
  add_executable(realsense_yolo src/realsense_yolo_node.cpp src/realsense_yolo.cpp)
  target_link_libraries(realsense_yolo ${catkin_LIBRARIES} ${OpenCV_LIBS} /opt/ros/noetic/lib/libmessage_filters.so /opt/ros/$ENV{ROS_DISTRO}/lib/libcv_bridge.so)

  add_executable(mobilessd_adaptor src/mobilessd_adaptor.cpp src/realsense_yolo.cpp)
  target_link_libraries(mobilessd_adaptor ${catkin_LIBRARIES} ${OpenCV_LIBS})

  add_executable(mobilessd_adaptor_tensorRT src/mobilenetssd_tensorRT_adaptor.cpp src/realsense_yolo.cpp)
  target_link_libraries(mobilessd_adaptor_tensorRT ${catkin_LIBRARIES} ${OpenCV_LIBS})

  add_executable(yolov4_adaptor_tensorRT src/yolov4_tensorRT_adaptor.cpp src/realsense_yolo.cpp)
  target_link_libraries(yolov4_adaptor_tensorRT ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()
