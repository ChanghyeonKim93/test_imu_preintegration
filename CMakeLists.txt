cmake_minimum_required(VERSION 3.9)
project(imu_preintegrator)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread")
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

set(INCLUDE_DIR ${INCLUDE_DIR} 
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIR})

include_directories(${INCLUDE_DIR})

set(SRC ${SRC}
  core/imu_preintegrator.cc
)

add_executable(
  test_imu_preintegration_residual_only_optimization 
  test/test_imu_preintegration_residual_only_optimization.cc ${SRC})
target_include_directories(
  test_imu_preintegration_residual_only_optimization
  PUBLIC
  ${CMAKE_SOURCE_DIR})