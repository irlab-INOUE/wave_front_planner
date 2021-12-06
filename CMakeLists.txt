cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -g3 -std=c++17")
project(cmakeSamples)

include_directories(${PROJECT_SOURCE_DIR} /opt/homebrew/Cellar/yaml-cpp/0.6.3_1/include)
link_directories(${PROJECT_SOURCE_DIR} /opt/homebrew/Cellar/yaml-cpp/0.6.3_1/lib)

find_package (OpenCV REQUIRED)

add_executable (main main.cpp)
target_link_libraries(main yaml-cpp ${OpenCV_LIBRARIES})
