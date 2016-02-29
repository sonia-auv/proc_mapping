# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     06/02/2016
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the GNU GPLv3 license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.0)
project(proc_mapping)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release
      RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s")

set(proc_mapping_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    geometry_msgs
    sensor_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
    lib_atlas
    sonia_msgs
    provider_sonar
)

catkin_package(
    INCLUDE_DIRS ${proc_mapping_SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
    geometry_msgs
    sensor_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
    lib_atlas
    sonia_msgs
    provider_sonar
)

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE proc_mapping_FILES
    "${proc_mapping_SRC_DIR}/*.cc"
    "${proc_mapping_SRC_DIR}/*.h")

list(REMOVE_ITEM proc_mapping_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/${proc_mapping_SRC_DIR}/${PROJECT_NAME}/main.cc)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${proc_mapping_SRC_DIR}
    ${lib_atlas_INCLUDE_DIRS}
    ${provider_sonar_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

add_executable(${PROJECT_NAME}_node ${proc_mapping_SRC_DIR}/${PROJECT_NAME}/main.cc ${proc_mapping_FILES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${lib_atlas_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node sonia_msgs_generate_messages_cpp)

#==========================================================================
# U N I T   T E S T S

if (CATKIN_ENABLE_TESTING AND ${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif ()
