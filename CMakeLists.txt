cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -g3 -std=c++17")
project(cmakeSamples)

include_directories(${PROJECT_SOURCE_DIR} /opt/homebrew/include)
link_directories(${PROJECT_SOURCE_DIR} /opt/homebrew/lib)

find_package (OpenCV REQUIRED)

add_executable (main main.cpp)
target_link_libraries(main yaml-cpp ${OpenCV_LIBRARIES})
