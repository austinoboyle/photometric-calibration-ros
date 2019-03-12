cmake_minimum_required(VERSION 3.2)
set(PROJECT_NAME online_photometric_calibration)

project(${PROJECT_NAME})

# We need at least C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# set march=native unless specified
if(NOT CXX_MARCH)
  set(CXX_MARCH native)
endif()
SET(CMAKE_CXX_FLAGS_RELEASE "-march=${CXX_MARCH} ${CMAKE_CXX_FLAGS_RELEASE}")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "-march=${CXX_MARCH} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

# warnings
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-sign-compare")

# NOTE: OpenCV 3 is shipped with ROS Kinetic.  Using OpenCV 2 here causes a
# conflict between cv_bridge and photometric_calibration
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs std_msgs
message_filters image_transport)
find_package(OpenCV 3 REQUIRED)
find_package(Threads REQUIRED)


catkin_package(
#  INCLUDE_DIRS src
 LIBRARIES online_pcalib
 CATKIN_DEPENDS roscpp
 DEPENDS OpenCV
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# include path

# Source files
set(ONLINE_CALIB_SOURCE_FILES
  src/Database.cpp
  src/GainRobustTracker.cpp
  src/ImageReader.cpp
  src/JacobianGenerator.cpp
  src/NonlinearOptimizer.cpp
  src/OptimizationBlock.cpp
  src/RapidExposureTimeEstimator.cpp
  src/Tracker.cpp
  src/VignetteModel.cpp
)

# add include files to custom target so they show up in IDEs like
# QtCreator in the project view
# file(GLOB_RECURSE _INCLUDE_FILES "src/*.h" "src/*.hpp")
# add_custom_target(_include_files_fix_target SOURCES ${_INCLUDE_FILES})

# main library
add_library(online_pcalib SHARED ${ONLINE_CALIB_SOURCE_FILES})
# add_dependencies(online_pcalib ${catkin_EXPORTED_TARGETS})

message("OPENCV LIBS ${OpenCV_LIBRARIES}")

target_link_libraries(online_pcalib
  ${OpenCV_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

# message("CATKIN LIBS ${catkin_LIBRARIES}")
# demo executable for online calibration
add_executable(online_pcalib_demo src/main.cpp)
target_link_libraries(online_pcalib_demo ${catkin_LIBRARIES} online_pcalib)