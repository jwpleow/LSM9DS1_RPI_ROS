cmake_minimum_required(VERSION 3.0.0)
project(LSM9DS1_RPI_GRPC)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(Threads REQUIRED)

add_subdirectory(LSM9DS1)

# Find ROS
set(ROS_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
  set(FOUND_ROS_DISTRO $ENV{ROS_DISTRO})
  set(ROS_FOUND TRUE)
else()
  message("ROS distro variable not set. Trying to figure it out...")
  set(AVAILABLE_ROS_VERSIONS "noetic;melodic;lunar;kinetic;jade;indigo")
  set(ROS_FOUND FALSE)
  foreach(version ${AVAILABLE_ROS_VERSIONS})
    if(NOT ROS_FOUND)
      find_path(ROS_H ros.h PATHS /opt/ros/${version}/include/ros)
      if(ROS_H)
        message("Found ros version ${version}")
        set(FOUND_ROS_DISTRO ${version})
        set(ROS_FOUND TRUE)
      endif()
    endif()
  endforeach()
endif()

if(ROS_FOUND)
  include_directories(/opt/ros/$ENV{ROS_DISTRO}/include/ /opt/ros/$ENV{ROS_DISTRO}/lib/)

  #imu ros client
  add_executable(imu_ros main_ros.cpp)
  target_link_libraries(imu_ros LSM9DS1
  -I/opt/ros/$ENV{ROS_DISTRO}/include -L/opt/ros/$ENV{ROS_DISTRO}/lib
  -lroscpp -lrostime -lrosconsole -lroscpp_serialization)
else()
  message("ROS not found, skipping...")
endif()


