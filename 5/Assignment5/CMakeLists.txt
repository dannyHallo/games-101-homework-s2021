cmake_minimum_required(VERSION 3.13)
project(Assignment5)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    src
    includes
)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/Scene.cpp 
    src/Renderer.cpp)