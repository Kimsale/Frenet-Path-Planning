cmake_minimum_required(VERSION 3.1)
project(CPP_ROBOTICS)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set(CMAKE_BUILD_TYPE Debug)
#add_definitions("-g")
#set(CMAKE_CXX_FLAGS_RELEASE “-O3”)

## 交叉编译器的路径
#set(tool_path /home/jin/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu)
## 指定编译器的可执行文件路径
#set(CMAKE_C_COMPILER ${tool_path}/bin/aarch64-none-linux-gnu-gcc)
#set(CMAKE_CXX_COMPILER ${tool_path}/bin/aarch64-none-linux-gnu-g++)
## 指定交叉编译的Eigen3库的包含路径
#set(EIGEN3_INCLUDE_DIR /home/jin/compile_eigen3.4/arm_eigen3/install/include/eigen3)


#############################
#Dependencies
#############################

find_package(Eigen3 REQUIRED)

#指定查找头文件的路径
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(./include)


##############################
#PathPlanning
##############################

add_executable(threemode_119 src/three_mode.cpp)
target_link_libraries(threemode_119 Eigen3::Eigen)
#add_executable(threemode_mix_noneon src/three_mode.cpp)
#target_link_libraries(threemode_mix_noneon Eigen3::Eigen)