cmake_minimum_required(VERSION 3.10)
project(PointCloudComparison)

find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED)

add_executable(PointCloudComparison main.cpp)
target_link_libraries(PointCloudComparison ${PCL_LIBRARIES} Eigen3::Eigen)

target_include_directories(PointCloudComparison PRIVATE ${PCL_INCLUDE_DIRS})
