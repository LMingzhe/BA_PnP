cmake_minimum_required(VERSION 2.8)
project(myslam)

# 编译子目录下的CMakeLists.txt
add_subdirectory(src)
add_subdirectory(app)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_STANDARD 11)  # 使用C++11标准
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) # 输出可执行文件到bin目录下
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)  # 输出库文件到lib目录下

# Eigen
include_directories("/usr/local/include/eigen3")
# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(THIRD_PARTY_LIBS 
    ${Sophus_LIBRARIES}
    ${OpenCV_LIBS}
    )



