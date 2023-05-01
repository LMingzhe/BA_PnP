set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories("/usr/local/include/eigen3")
add_library(myslam SHARED 
    KeyPoint.cpp
    FastDetect.cpp
    GetBrief.cpp
    Match.cpp
    BFMatch.cpp
    BundleAdjustment.cpp)

target_link_libraries(myslam ${THIRD_PARTY_LIBS})

