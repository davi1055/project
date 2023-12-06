cmake_minimum_required(VERSION 3.14)
project(BuffDetect)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 3.10 EXACT REQUIRED COMPONENTS Development Interpreter)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(
    ${PROJECT_NAME}
    ./src/main.cpp
    ./include/BuffDetect_new.hpp
    ./general/GxCamera/GxCamera.cpp
    ./general/GxCamera/GxCamera.h
    ./general/GxCamera/include/GxIAPI.h
    ./general/GxCamera/include/DxImageProc.h
)

include_directories(
    ${OPENCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)


target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${X11_LIBRARIES}
    ${CERES_LIBRARIES}
    gxiapi
    pthread
    Python3::Python
    pybind11::pybind11
)