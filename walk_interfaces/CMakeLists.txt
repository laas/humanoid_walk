cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Tests.
rosbuild_add_gtest(test/stamped-position test/stamped-position.cpp)
rosbuild_add_gtest(test/trajectory-3d test/trajectory-3d.cpp)
rosbuild_add_gtest(test/dummy-pattern-generator
  test/dummy-pattern-generator.cpp)
