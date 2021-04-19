cmake_minimum_required(VERSION 3.0.0)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(LSM9DS1 LANGUAGES CXX)
include(GNUInstallDirs)
# add_subdirectory(example)

# add_compile_options(-Wall -Wconversion -Wextra -pedantic)

include_directories(include/)
set(LIBSRC src/LSM9DS1.cpp)
set(LIBINCLUDE include/LSM9DS1.h include/LSM9DS1_Registers.h include/LSM9DS1_Types.h include/wiringPiI2C.h)

add_library(LSM9DS1
  SHARED
  ${LIBSRC}
  )
target_include_directories(LSM9DS1 PUBLIC include/ ${CMAKE_CURRENT_SOURCE_DIR})




