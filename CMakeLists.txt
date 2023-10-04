cmake_minimum_required(VERSION 3.23)
project(ekf-test-task LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)

add_executable(mylittlekalman
  include/constants.h
  include/functions.h
  include/structures.h
  src/functions.cpp
  src/main.cpp
)
target_link_libraries(mylittlekalman Eigen3::Eigen)
target_include_directories(mylittlekalman PUBLIC include)

