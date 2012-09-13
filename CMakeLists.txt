cmake_minimum_required(VERSION 2.6)
project(bulletsim)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")
set(BULLETSIM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

if (NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif (NOT CMAKE_BUILD_TYPE)

# external libraries
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenRAVE REQUIRED)
find_package(OpenThreads REQUIRED)
find_package(osg REQUIRED)
find_package(osgDB REQUIRED)
find_package(osgGA REQUIRED)
find_package(osgText REQUIRED)
find_package(osgUtil REQUIRED)
find_package(osgViewer REQUIRED)
set(OSG_LIBS ${OPENTHREADS_LIBRARY} ${OSG_LIBRARY} ${OSGDB_LIBRARY} ${OSGGA_LIBRARY} ${OSGTEXT_LIBRARY} ${OSGUTIL_LIBRARY} ${OSGVIEWER_LIBRARY})
set(OSG_INCLUDE_DIRS ${OPENTHREADS_LIBRARY_INCLUDE_DIR} ${OSG_LIBRARY_INCLUDE_DIR} ${OSGDB_LIBRARY_INCLUDE_DIR} ${OSGGA_LIBRARY_INCLUDE_DIR} ${OSGTEXT_LIBRARY_INCLUDE_DIR} ${OSGUTIL_LIBRARY_INCLUDE_DIR} ${OSGVIEWER_LIBRARY_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(ROS REQUIRED COMPONENTS
 sensor_msgs geometry_msgs std_msgs
 roscpp rosconsole message_filters
 cpp_common rostime roscpp_traits
 roscpp_serialization
 bulletsim_msgs
)
set(ROS_ROOT_DIR /opt/ros/fuerte)

find_package(OpenCV REQUIRED)

add_definitions(${PCL_DEFINITIONS})
add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")
add_definitions("-DBT_NO_PROFILE")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

# directories for libraries packaged in this tree
set(BULLET_DIR ${BULLETSIM_SOURCE_DIR}/lib/bullet-2.79)
set(BULLET_LIBS BulletFileLoader BulletSoftBody BulletDynamics BulletCollision LinearMath HACD)

set(OSGWORKS_DIR ${BULLETSIM_SOURCE_DIR}/lib/osgWorks-2.0)
set(OSGBULLET_DIR ${BULLETSIM_SOURCE_DIR}/lib/osgBullet-2.0)
set(JSON_DIR ${BULLETSIM_SOURCE_DIR}/lib/json)
set(JSON_INCLUDE_DIR ${JSON_DIR}/include)
set(TETGEN_DIR ${BULLETSIM_SOURCE_DIR}/lib/tetgen-1.4.3)
set(LOG4CPLUS_DIR ${BULLETSIM_SOURCE_DIR}/lib/log4cplus-1.1.0-rc3)
set(LOG4CPLUS_INCLUDE_DIRS ${LOG4CPLUS_DIR}/include ${CMAKE_BINARY_DIR}/include)
set(LOG4CPLUS_LIBRARY "log4cplus")
set(PHASESPACE_DIR ${BULLETSIM_SOURCE_DIR}/lib/phasespace)
set(PHASESPACE_INCLUDE_DIR ${PHASESPACE_DIR}/include)
set(PHASESPACE_LIBS ${PHASESPACE_DIR}/libowlsock.so)

# other external libraries, hardcoded
set(WM_ROOT "~/Src/GeometricTools/WildMagic5")
set(WM_INCLUDE_DIR "${WM_ROOT}/SDK/Include")
set(WM_LIBRARY_DIR "${WM_ROOT}/SDK/Library/Debug")
set(WM_LIBRARIES Wm5Core Wm5Imagics Wm5Mathematics Wm5Physics)
set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}/include")
set(GUROBI_LIBRARY_DIR "$ENV{GUROBI_HOME}/lib")

# build options
option(BUILD_TESTS "Build test/demo programs" off)
option(BUILD_TRACKING "Build tracking code" on)

set(BUILD_SHARED_LIBS off)

add_subdirectory(lib)
add_subdirectory(src)
