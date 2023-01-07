cmake_minimum_required(VERSION 2.8.3)
project(sensor_ros_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_GLIBCXX_USE_C99=1)

## imu: Wit
find_package(catkin REQUIRED COMPONENTS
  ecl_build
  ecl_devices
  ecl_sigslots
  ecl_time
  ecl_geometry
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nodelet
  pluginlib
  message_generation
  eigen_conversions
  cv_bridge
  image_transport
)


## Generate messages in the 'msg' folder
add_message_files(
 FILES
 ImuGpsRaw.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wit_nodelet
  CATKIN_DEPENDS
  ecl_build
  ecl_devices
  ecl_geometry
  ecl_sigslots
  ecl_time
  nodelet
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

## camera: Mindvision
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
add_library(mindvision SHARED IMPORTED)
set_target_properties(mindvision PROPERTIES 
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/include/sdk/mindvision/lib/x64/libMVSDK.so
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include/sdk/mindvision/include)
add_executable(camera_ros src/camera_ros.cpp)
add_executable(save_demo src/save_demo.cpp)
target_link_libraries(camera_ros ${catkin_LIBRARIES}
                             ${OpenCV_LIBRARIES}
                             mindvision)
target_compile_definitions(camera_ros PRIVATE MINDVISION)
target_link_libraries(save_demo ${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
mindvision)
target_compile_definitions(save_demo PRIVATE MINDVISION) 




















