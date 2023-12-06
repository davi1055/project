cmake_minimum_required(VERSION 2.9)
project(TEST)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(Python3 3.10 EXACT REQUIRED COMPONENTS Development Interpreter)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(
    ${PROJECT_NAME}
    ./main.cpp
    ./src/ArmorCenter/ArmorCenter.h
    ./src/ArmorCenter/ArmorCenter.cpp
    ./src/ArmorCenter/AngleSolve.h
    ./src/ArmorCenter/AngleSolve.cpp
)

include_directories(
    ${OPENCV_INCLUDE_DIRS}
    ${JSONCPP_INC_PATH}
    ${HEADER_RAPIDJSON_INC}
    ${EIGEN3_INCLUDE_DIRS}
)


target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${THIRDPARTY_RAPIDJSON}
    ${TINYXML_LIBRARIES}
    Python3::Python
    pybind11::pybind11
)
#gxiapi是大恒相机的必须链接
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${X11_LIBRARIES} ${CERES_LIBRARIES} gxiapi pthread)
target_link_libraries(${PROJECT_NAME} Python3::Python pybind11::pybind11)