cmake_minimum_required(VERSION 2.8.3)
project(kimera_interfacer)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set ( CMAKE_BUILD_TYPE  Release )
set( CMAKE_CXX_FLAGS_RELEASE "-O3")

add_definitions(-std=c++17 -Wall -Wextra)
# set( CMAKE_CXX_FLAGS_RELEASE "-O3")

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
