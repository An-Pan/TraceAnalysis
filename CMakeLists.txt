cmake_minimum_required(VERSION 3.10)
project(TraceAnalysis)


set( CMAKE_CXX_FLAGS "-std=c++11 -O0" )
find_package(GDAL REQUIRED)
find_package(OpenCV REQUIRED)
find_package( PCL 1.8.0 )
find_package( Boost 1.58.0 REQUIRED)
find_package(Eigen3)

message(STATUS "GDAL library status:")
message(STATUS " version: ${GDAL_VERSION}")
message(STATUS " libraries: ${GDAL_LIBRARY}")
message(STATUS " include path: ${GDAL_INCLUDE_DIRS}")

set(CMAKE_CXX_STANDARD 11)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GDAL_INCLUDE_DIRS})

#link_directories(/usr/lib/x86_64-linux-gnu)
file(GLOB SOURCE
        "./*.h"
        "./*.cpp"
        )

add_executable(TraceAnalysis ${SOURCE})

target_link_libraries(TraceAnalysis ${TENSORRT_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${GDAL_LIBRARY}
        -lboost_system -lboost_filesystem -lboost_regex -lshp libproj.so -lpthread  )