cmake_minimum_required(VERSION 2.8.3)
project(realsense_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  dynamic_reconfigure
  image_transport
  message_generation
  roscpp
  roslib
  std_msgs
  tf
)

# V4L2, udev and libusb
find_package(PkgConfig REQUIRED)
pkg_check_modules(V4L REQUIRED libv4l2)
pkg_check_modules(UDEV REQUIRED libudev)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# OpenCV
find_package(OpenCV REQUIRED)
if (OPENCV_FOUND)
  message("-- Found OpenCV version ${OPENCV_VERSION}: ${OPENCV_INCLUDE_DIRS}")
endif ()


# PCL
find_package(PCL 1.7 REQUIRED COMPONENTS
    common
    io
)


generate_dynamic_reconfigure_options(
  config/RealsenseCamera.cfg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


add_message_files(
  FILES
  realsenseConfig.msg
)

add_service_files(
  FILES
  get_rgb_uv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realsense_camera
  CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${V4L_INCLUDE_DIRS}
  ${UDEV_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)



add_definitions(
  ${OpenCV_DEFINITIONS}
  ${PCL_DEFINITIONS}
)


## Declare a cpp library
# add_library(realsense_camera
#   src/${PROJECT_NAME}/realsense_camera.cpp
# )

## Declare a cpp executable
# add_executable(realsense_camera_node src/realsense_camera_node.cpp)

add_executable(realsense_camera_node 
  src/realsense_camera.cpp
  src/capturer_mmap.cpp
  src/v4l_unit.cpp
  src/udev_unit.cpp
)

add_executable(realsense_camera_config_node 
  src/realsense_camera_config.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(realsense_camera_node realsense_camera_generate_messages_cpp)


add_dependencies(realsense_camera_node realsense_camera_generate_messages_cpp)
add_dependencies(realsense_camera_config_node realsense_camera_generate_messages_cpp)
add_dependencies(realsense_camera_node ${PROJECT_NAME}_gencfg)


## Specify libraries to link a library or executable target against
# target_link_libraries(realsense_camera_node
#   ${catkin_LIBRARIES}
# )


target_link_libraries(realsense_camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${V4L_LIBRARIES}
  ${UDEV_LIBRARIES}
  ${LIBUSB_LIBRARIES}
)


target_link_libraries(realsense_camera_config_node
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(
  TARGETS
    realsense_camera_node
    realsense_camera_config_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    config/80-realsense-usb.rules
  DESTINATION /etc/udev/rules.d
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
    launch/realsense_camera.launch
    launch/realsense_camera_config.launch
    launch/realsense_frames.launch
    launch/realsense_r200_camera.launch 
    launch/realsense_r200_rviz.launch
    launch/realsense_rviz.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
    config/realsense.rviz
    config/RealsenseCamera.cfg
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_realsense_camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
