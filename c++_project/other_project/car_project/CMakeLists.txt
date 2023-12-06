cmake_minimum_required(VERSION 3.14)
project(car)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4 REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(INCLUDE_DIR ./include)		#inlcude目录
set(MAIN_DIR ./src)             #main目录

include_directories(
    include
    ${MAIN_DIR}
    ${INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIRS}
)

file(GLOB SRC ${MAIN_DIR}/*.cpp) #cpp文件

add_executable(
    ${PROJECT_NAME}
    ${SRC}
    
)

target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    
)