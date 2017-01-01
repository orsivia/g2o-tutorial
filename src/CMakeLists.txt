cmake_minimum_required (VERSION 2.6)
project (g2o_examples)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY /home/zixuan/g2o_example/exe/)
add_executable(3d_optimize
    3d_optimize.cpp
)
add_executable(1d_optimize
    1d_optimize.cpp
)

target_link_libraries(3d_optimize /home/zixuan/g2o_example/g2o/lib/libg2o.so)
target_link_libraries(1d_optimize /home/zixuan/g2o_example/g2o/lib/libg2o.so)
