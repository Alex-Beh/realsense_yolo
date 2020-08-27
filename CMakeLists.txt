cmake_minimum_required(VERSION 3.0.2)
project(realsense_yolo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(OpenCV 2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge2
  roscpp
  rospy
  std_msgs
  actionlib
  darknet_ros_msgs
  image_transport
  spencer_tracking_msgs
  message_generation
  pcl_ros
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES debug_yolo.msg BoundingBoxes3d.msg BoundingBox3d.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realsense_yolo
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
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

  cuda_add_executable(realsense_yolo src/realsense_yolo_node.cpp src/realsense_yolo.cpp)
  target_link_libraries(realsense_yolo ${catkin_LIBRARIES} ${OpenCV_LIBS})

else()
  add_executable(realsense_yolo src/realsense_yolo_node.cpp src/realsense_yolo.cpp)
  target_link_libraries(realsense_yolo ${catkin_LIBRARIES} ${OpenCV_LIBS})

endif()
