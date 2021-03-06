
cmake_minimum_required(VERSION 3.5)
project(orbslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Pangolin REQUIRED)

set(ORB_SLAM2_DIR ${HOME}/ORB_SLAM2)

include_directories(${ORB_SLAM2_DIR}/include)
include_directories(${ORB_SLAM2_DIR})
include_directories(${Pangolin_INCLUDE_DIRS})

link_directories(${ORB_SLAM2_DIR}/lib/)


add_executable(mono 
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)

target_link_libraries(mono
  ORB_SLAM2
)

ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM2 Pangolin)


add_executable(rgbd 
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)

target_link_libraries(rgbd
  ORB_SLAM2
)

ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM2 Pangolin)



add_executable(stereo 
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)

target_link_libraries(stereo
  ORB_SLAM2
)

ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM2 Pangolin)



install(TARGETS mono rgbd stereo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
