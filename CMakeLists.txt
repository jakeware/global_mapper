cmake_minimum_required(VERSION 2.8.3)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

project(global_mapper)

set(CMAKE_VERBOSE_MAKEFILE FALSE)

add_definitions("-std=c++11")
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

## find catkin macros and libraries
set(REQ_CATKIN_PKGS   
  roscpp
  nodelet
  fla_utils
  pcl_ros
)
find_package(catkin REQUIRED COMPONENTS ${REQ_CATKIN_PKGS})
# add_message_files(...)
# add_service_files(...)
# add_action_files(...)
# generate_messages(...)
 
# setup downstream dependencies
catkin_package(
 INCLUDE_DIRS src
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${REQ_CATKIN_PKGS}
 DEPENDS
)

# setup global include directories
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

# global_mapper_nodelet
add_subdirectory("src")

# global_mapper_ros wrapper library
add_subdirectory("src/global_mapper_ros")

# global_mapper library
add_subdirectory("src/global_mapper")
