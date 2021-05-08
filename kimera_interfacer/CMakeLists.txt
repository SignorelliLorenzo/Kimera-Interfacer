cmake_minimum_required(VERSION 2.8.3)
project(kimera_interfacer)

find_package(catkin_simple REQUIRED)
#find_package(catkin REQUIRED COMPONENTS
#        sensor_msgs
#        message_generation)

catkin_simple(ALL_DEPS_REQUIRED)

set ( CMAKE_BUILD_TYPE  Debug )
add_definitions(-std=c++17 -Wall -Wextra)

#############
# LIBRARIES #
#############


############
# BINARIES #
############

cs_add_executable(kimera_interfacer src/kimera_interfacer.cpp)

##########
# EXPORT #
##########
cs_install()

cs_install_scripts(scripts/dl_mock.py)

cs_export()