cmake_minimum_required(VERSION 3.5)
project(insta360_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(imu_tools REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
set(INSTA360_MEDIA_SDK_ROOT "/usr" CACHE PATH "Insta360 MediaSDK installation prefix")
find_path(MEDIA_SDK_INCLUDE_DIR ins_realtime_stitcher.h
  PATHS "${INSTA360_MEDIA_SDK_ROOT}/include" /usr/include REQUIRED)
find_library(MEDIA_SDK_LIBRARY MediaSDK
  PATHS "${INSTA360_MEDIA_SDK_ROOT}/lib" /usr/lib /usr/lib/x86_64-linux-gnu REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "msg/CameraPreviewInfo.msg"
  DEPENDENCIES std_msgs
)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_search_module(SWSCALE REQUIRED libswscale)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/include
  ${GSTREAMER_LIBRARY_DIRS}
)

install(
  DIRECTORY config launch lib
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(${PROJECT_NAME} src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rosidl_default_runtime
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
target_link_libraries(${PROJECT_NAME} CameraSDK  ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})
rosidl_target_interfaces(${PROJECT_NAME} ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp")

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(ffmpeg_decoder src/decoder.cpp)
ament_target_dependencies(ffmpeg_decoder
  rclcpp
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
target_link_libraries(ffmpeg_decoder CameraSDK  ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})

install(TARGETS ffmpeg_decoder
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(gstreamer_encoder src/gstreamer_encoder.cpp src/gstreamer_ros_sink.cpp)
ament_target_dependencies(gstreamer_encoder rclcpp cv_bridge sensor_msgs std_msgs OpenCV)
target_link_libraries(gstreamer_encoder ${OpenCV_LIBRARIES} ${GSTREAMER_LIBRARIES})

add_executable(gstreamer_decoder src/gstreamer_decoder.cpp)
ament_target_dependencies(gstreamer_decoder rclcpp cv_bridge sensor_msgs std_msgs OpenCV)
target_link_libraries(gstreamer_decoder ${GSTREAMER_LIBRARIES})

add_executable(media_sdk_stitcher src/media_sdk_stitcher.cpp)
ament_target_dependencies(media_sdk_stitcher rclcpp sensor_msgs cv_bridge rosidl_default_runtime)
target_include_directories(media_sdk_stitcher PRIVATE ${MEDIA_SDK_INCLUDE_DIR})
target_link_libraries(media_sdk_stitcher ${MEDIA_SDK_LIBRARY} pthread)
rosidl_target_interfaces(media_sdk_stitcher ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp")

add_executable(webcam_equirectangular src/webcam_equirectangular.cpp)
ament_target_dependencies(webcam_equirectangular rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(webcam_equirectangular ${OpenCV_LIBRARIES})

# Add C++ image conversion nodes
add_executable(dual_fisheye2equirectangular_cpp src/dual_fisheye2equirectangular.cpp)
ament_target_dependencies(dual_fisheye2equirectangular_cpp
  rclcpp
  rcl_interfaces
  cv_bridge
  sensor_msgs
  std_msgs
  OpenCV
)
add_executable(dual_fisheye2perspective_cpp src/dual_fisheye2perspective.cpp)
ament_target_dependencies(dual_fisheye2perspective_cpp
  rclcpp
  rcl_interfaces
  cv_bridge
  sensor_msgs
  std_msgs
  OpenCV
  tf2_geometry_msgs
  geometry_msgs
)
add_executable(equirectangular2perspective_cpp src/equirectangular2perspective.cpp)
ament_target_dependencies(equirectangular2perspective_cpp
  rclcpp
  rcl_interfaces
  cv_bridge
  sensor_msgs
  std_msgs
  OpenCV
  tf2_geometry_msgs
  geometry_msgs
)
target_link_libraries(dual_fisheye2equirectangular_cpp ${OpenCV_LIBRARIES})
target_link_libraries(dual_fisheye2perspective_cpp ${OpenCV_LIBRARIES} Eigen3::Eigen)
target_link_libraries(equirectangular2perspective_cpp ${OpenCV_LIBRARIES} Eigen3::Eigen)

install(TARGETS
  dual_fisheye2equirectangular_cpp
  dual_fisheye2perspective_cpp
  equirectangular2perspective_cpp
  gstreamer_encoder
  gstreamer_decoder
  media_sdk_stitcher
  webcam_equirectangular
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/equirectangular.py
  scripts/calibrate.py
  scripts/calibrate_cpp.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
