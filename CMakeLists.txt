project(ekf-test-task LANGUAGES CXX)

find_package(Eigen3 REQUIRED)

add_executable(mylittlekalman
  include/constants.h
  include/functions_h.h
  include/structures.h
  src/functions.cpp
  src/main.cpp
)
target_link_libraries(mylittlekalman Eigen3::Eigen)
target_include_directories(mylittlekalman PUBLIC include)

