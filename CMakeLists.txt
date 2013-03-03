cmake_minimum_required(VERSION 2.6)
project(bulletsim)

set(Boost_USE_STATIC_LIBS off)
include(cmake/main.cmake)

set(BULLETSIM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
ENDIF (NOT CMAKE_BUILD_TYPE)

set(BUILD_SHARED_LIBS on)

# external libraries
find_package(Boost COMPONENTS system python filesystem program_options REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenRAVE REQUIRED)

add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")
add_definitions("-DBT_NO_PROFILE")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

# directories for libraries packaged in this tree
set(BULLET_DIR ${BULLETSIM_SOURCE_DIR}/lib/bullet-2.79)
set(BULLET_LIBS BulletFileLoader BulletSoftBody BulletDynamics BulletCollision LinearMath HACD)

set(JSON_DIR ${BULLETSIM_SOURCE_DIR}/lib/json)
set(JSON_INCLUDE_DIR ${JSON_DIR}/include)
set(TETGEN_DIR ${BULLETSIM_SOURCE_DIR}/lib/tetgen-1.4.3)
set(LOG4CPLUS_DIR ${BULLETSIM_SOURCE_DIR}/lib/log4cplus-1.1.0-rc3)
set(LOG4CPLUS_INCLUDE_DIRS ${LOG4CPLUS_DIR}/include ${CMAKE_BINARY_DIR}/include)
set(LOG4CPLUS_LIBRARY "log4cplus")

add_subdirectory(lib)
add_subdirectory(src)
