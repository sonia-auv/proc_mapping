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
# Store the mlpack include directory in MLPACK_INCLUDE_DIR.
# The PATHS variable may be specified to give hints for where to find core.hpp.
find_path(MLPACK_INCLUDE_DIR
        NAMES core.hpp prereqs.hpp
        PATHS /usr/include/mlpack
        )

# Find libmlpack.so (or equivalent) and store it in MLPACK_LIBRARY.
# If this example script were smarter, it would also find other dependencies of mlpack and store them in
# an MLPACK_LIBRARIES variable instead.
find_library(MLPACK_LIBRARY
        NAMES mlpack
        PATHS /usr/lib/x86_64-linux-gnu /usr/local/lib/x86_64-linux-gnu /usr/lib64/ /usr/lib/ /usr/local/lib64/ /usr/local/)

find_library(ARMADILLO_LIBRARY
        NAMES armadillo
        PATHS /usr/lib/x86_64-linux-gnu /usr/local/lib/x86_64-linux-gnu /usr/lib64/ /usr/lib/ /usr/local/lib64/ /usr/local/
        )
find_path(ARMADILLO_INCLUDE_DIR
        NAMES armadillo
        PATHS /usr/include
        )


find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        cmake_modules
        roscpp
        std_msgs
        visualization_msgs
        message_generation
        message_runtime
        proc_hydrophone
)

#===============================================================================
# G E N E R A T E   M E S S A G E S   &   S E R V I C E S

add_message_files(DIRECTORY msg
        FILES
        PoseSavedRequest.msg
        PingerLocation.msg)

add_service_files(DIRECTORY srv
        FILES
        ObjectiveReset.srv
        PingerLocationService.srv)

generate_messages(DEPENDENCIES visualization_msgs)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        message_runtime
        geometry_msgs
        sensor_msgs
        nav_msgs
        visualization_msgs
        proc_hydrophone
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
        ${MLPACK_INCLUDE_DIR}
        ${ARMADILLO_INCLUDE_DIR}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

add_executable(${PROJECT_NAME}_node ${proc_mapping_SRC_DIR}/${PROJECT_NAME}/main.cc ${proc_mapping_FILES})
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${MLPACK_LIBRARY}
        ${ARMADILLO_LIBRARY}
        yaml-cpp
        )

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

#==========================================================================
# U N I T   T E S T S

if (CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif ()
