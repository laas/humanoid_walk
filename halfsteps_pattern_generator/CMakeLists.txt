cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# WARNING: due to a subtle bug, -O2 produces invalid code.
# Do *not* change the build type.
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

rosbuild_add_boost_directories()

# Make sure Boost.Filesystem v2 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=2)

# Main library.
rosbuild_add_library(${PROJECT_NAME}
  src/libhalfsteps_pattern_generator/newPGstepStudy.cpp
  src/libhalfsteps_pattern_generator/halfsteps_pattern_generator.cpp
  include/halfsteps_pattern_generator.hh
  include/yaml.hh)
rosbuild_link_boost(${PROJECT_NAME} filesystem date_time system)

# Node.
rosbuild_add_executable(generator
  src/nodes/generator.cpp)
target_link_libraries(generator ${PROJECT_NAME})

rosbuild_add_boost_directories()

# Tests.
rosbuild_add_gtest(pattern-generator test/pattern-generator.cpp)
target_link_libraries(pattern-generator ${PROJECT_NAME})
rosbuild_link_boost(pattern-generator filesystem date_time system)
