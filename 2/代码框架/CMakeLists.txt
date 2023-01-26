cmake_minimum_required(VERSION 3.13)
project(Rasterizer)

set(CMAKE_CXX_STANDARD 17)

# where to search for headers
include_directories(
    src
    includes
    D:/opencv/build/include
    D:/opencv/build/include/opencv2)

add_executable(${PROJECT_NAME} 
    src/main.cpp
    src/rasterizer.cpp
    src/Triangle.cpp)

target_link_directories(${PROJECT_NAME} PRIVATE D:/opencv/build/x64/vc16/lib)
target_link_libraries(${PROJECT_NAME} opencv_world470 opencv_world470d)