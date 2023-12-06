####设置项目基础相关属性####
cmake_minimum_required(VERSION 3.12) 	# 最小版本

project(rm_version4)					# 项目名

set(CMAKE_CXX_STANDARD 17)				#c++17
set(CMAKE_CXX_EXTENSIONS OFF)			#编译器使用ISO标准
set(CMAKE_CXX_STANDARD_REQUIRED ON) 	#编译器不支持17则报错
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0") #优化等级设定
####设置项目基础属性####

####DEBUG开关####
set(DEBUG	ON)							#是否开启DEBUG			
if(DEBUG)
	set(CMAKE_BUILD_TYPE "Debug")
	set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
	add_definitions(-D_DEBUG_)			#控制是否debug的宏
else()
	set(CMAKE_CONFIGURATION_TYPES "Release")
endif()
####DEBUG开关####

####摄像头设置####
set(USINGVIDEO0 OFF)					#用不用电脑自带摄像头	
if(USINGVIDEO0)
	add_definitions(-D_USING_VIDEO_)	
endif()
####摄像头设置####

####设置src相关属性####
set(INCLUDE_DIR src/c++/include)		#inlcude目录
set(GENERAL_DIR src/general)			#general目录
set(DATA_FILE_DIR src/data_file)		#data_file目录
set(MAIN src/c++/cppcode/main.cpp)  	#main的位置
####设置src相关属性####

####需要引入的外部包####
find_package(Eigen3)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 3 EXACT REQUIRED COMPONENTS Development Interpreter)
# find_package(glog REQUIRED)
####需要引入的外部包####

####设置include路径####
include_directories(
	include 
	# 内部
	${INCLUDE_DIR}	
	${GENERAL_DIR}
	${DATA_FILE_DIR}
	# 外部
	${EIGEN3_INCLUDE_DIR} 
	${CERES_INCLUDE_DIRS} 
	${GLOG_INCLUDE_DIRS} 
	${PYBIND11_INCLUDE_DIRS} 
	)
####设置include路径####

####添加c和cpp并生成####
file(GLOB SRC ${MAIN} 
${INCLUDE_DIR}/image_updating/GxCamera/*.cpp) #cpp文件引入


add_executable(${PROJECT_NAME}	${SRC})		#生成可执行文件
####添加c和cpp并生成####

####项目链接libs####
if(DEBUG)									#debug link libs
	set(IMGUI_LIBS ${PROJECT_SOURCE_DIR}/src/libs/libimgui.so GL glfw dl
	${PROJECT_SOURCE_DIR}/src/libs/libimguigl3.a
	)
	target_link_libraries(${PROJECT_NAME} ${IMGUI_LIBS})
endif()

set(LIBS ${OpenCV_LIBS} ${X11_LIBRARIES} ${CERES_LIBRARIES} 
pthread Python3::Python pybind11::pybind11)								

target_link_libraries(${PROJECT_NAME} ${LIBS})
#gxiapi是大恒相机的必须链接
target_link_libraries(${PROJECT_NAME} gxiapi)
target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS})
####项目链接libs####

