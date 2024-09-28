cmake_minimum_required(VERSION 3.1)
project(path_finder)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(algorithm algorithm.cpp)
target_link_libraries(algorithm ${PCL_LIBRARIES})
