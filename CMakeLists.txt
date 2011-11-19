cmake_minimum_required(VERSION 2.6)
project(SURGICAL_PHYSICS)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")

# bullet directory (packaged in this tree)
set(BULLET_DIR ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/bullet-2.79)
set(OSGWORKS_DIR ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/osgWorks-2.0)
set(OSGBULLET_DIR ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/osgBullet-2.0)

IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE) 

# eigen2 directory (external)
# taken from https://raw.github.com/cryos/avogadro/master/cmake/modules/FindEigen2.cmake
if (EIGEN2_INCLUDE_DIR)
    # in cache already
    set(EIGEN2_FOUND TRUE)
else (EIGEN2_INCLUDE_DIR)
    find_package(PkgConfig)
    pkg_check_modules(PC_EIGEN2 QUIET eigen2)
    find_path(EIGEN2_INCLUDE_DIR NAMES Eigen/Core
        HINTS ${PC_EIGEN2_INCLUDEDIR} ${PC_EIGEN2_INCLUDE_DIRS}
        PATH_SUFFIXES eigen2
    )
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Eigen2 DEFAULT_MSG EIGEN2_INCLUDE_DIR)
    mark_as_advanced(EIGEN2_INCLUDE_DIR)
endif(EIGEN2_INCLUDE_DIR)

find_package(OpenRAVE 0.4.0 REQUIRED)

find_package(OpenThreads)
find_package(osg)
find_package(osgDB)
find_package(osgUtil)
find_package(osgViewer)
set(OSG_LIBS ${OPENTHREADS_LIBRARY} ${OSG_LIBRARY} ${OSGDB_LIBRARY} ${OSGUTIL_LIBRARY} ${OSGVIEWER_LIBRARY})
set(OSG_INCLUDE_DIRS ${OPENTHREADS_LIBRARY_INCLUDE_DIR} ${OSG_LIBRARY_INCLUDE_DIR} ${OSGDB_LIBRARY_INCLUDE_DIR} ${OSGUTIL_LIBRARY_INCLUDE_DIR} ${OSGVIEWER_LIBRARY_INCLUDE_DIR})

add_subdirectory(lib)
add_subdirectory(src)
