cmake_minimum_required(VERSION 3.0.2)
project(insta360_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lib
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} CameraSDK ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
    "${PROJECT_SOURCE_DIR}/include"
)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS 
  scripts/compression.py
  scripts/undistortion.py
  scripts/live_processing.py
  scripts/get_images.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

