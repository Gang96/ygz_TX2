cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# arm and intel cpu configurations
IF(DEFINED ENV{ARM_ARCHITECTURE})
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -march=native")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake-modules)

# thirdparty libs 
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    message(STATUS "Don't get Opencv 3.0, looking for OpenCV 2.4.")
   find_package(OpenCV 2.4 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Glog REQUIRED )
find_package(Boost COMPONENTS system)

# to show 'include' in QtCreator
#FILE(GLOB_RECURSE INC_ALL "*/*.h")
#add_custom_target(headers SOURCES ${INC_ALL})

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../
${PROJECT_SOURCE_DIR}/../../include
${PROJECT_SOURCE_DIR}/../../src
${PROJECT_SOURCE_DIR}/../../Thirdparty/fast/include
${PROJECT_SOURCE_DIR}/../../Thirdparty/DBoW2
${PROJECT_SOURCE_DIR}/../../Thirdparty/g2o/
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)

# thirdparty
set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
	${Boost_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/../../Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/../../Thirdparty/fast/build/libfast.so
        ${PROJECT_SOURCE_DIR}/../../Thirdparty/g2o/lib/libg2o.so
        #${PROJECT_SOURCE_DIR}/../../Thirdparty/lib/libboost_filesystem.so
        #${PROJECT_SOURCE_DIR}/../../Thirdparty/lib/libboost_system.so
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${GLOG_LIBRARY}
        
)

# subdirectories
include_directories(
        ${PROJECT_SOURCE_DIR}/../../util/include
        ${PROJECT_SOURCE_DIR}/../../cv/include
        ${PROJECT_SOURCE_DIR}/../../common/include
        ${PROJECT_SOURCE_DIR}/../../backend/include
        ${PROJECT_SOURCE_DIR}/../../system/include
)
add_subdirectory(${PROJECT_SOURCE_DIR}/../../util ${PROJECT_SOURCE_DIR}/../../util)

add_subdirectory(${PROJECT_SOURCE_DIR}/../../common ${PROJECT_SOURCE_DIR}/../../common)

add_subdirectory(${PROJECT_SOURCE_DIR}/../../cv ${PROJECT_SOURCE_DIR}/../../cv)

add_subdirectory(${PROJECT_SOURCE_DIR}/../../backend ${PROJECT_SOURCE_DIR}/../../backend)

add_subdirectory(${PROJECT_SOURCE_DIR}/../../system ${PROJECT_SOURCE_DIR}/../../system)

set(YGZ_LIBS ygz-common ygz-cv ygz-util ygz-backend ygz-system)

# Node for stereo camera without IMU
rosbuild_add_executable(StereoROS src/StereoROS.cpp)
target_link_libraries(StereoROS
        ${YGZ_LIBS}
        ${THIRD_PARTY_LIBS}
        )

# Node for stereo camera with IMU
rosbuild_add_executable(StereoVioROS src/StereoVioROS.cpp)
target_link_libraries(StereoVioROS
        ${YGZ_LIBS}
        ${THIRD_PARTY_LIBS}
        )
