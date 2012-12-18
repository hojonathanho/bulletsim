#cmake_minimum_required(VERSION 2.6)
#project(scp)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)
find_package(Eigen3 REQUIRED)
find_package(osg REQUIRED)
find_package(osgUtil REQUIRED)
find_package(osgViewer REQUIRED)
find_package(OpenThreads REQUIRED) 
find_package(Gurobi REQUIRED)
find_package(Boost REQUIRED)
find_package(GLUT REQUIRED) 
find_package(OpenGL REQUIRED)

MACRO(LIB_HEADERS return_list)
  FILE(GLOB_RECURSE new_list lib/*.h)
  SET(dir_list "")
  FOREACH(file_path ${new_list})
    GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
    SET(dir_list ${dir_list} ${dir_path})
  ENDFOREACH()
  LIST(REMOVE_DUPLICATES dir_list)
  SET(${return_list} ${dir_list})
ENDMACRO()

MACRO(UTIL_HEADERS return_list)
  FILE(GLOB_RECURSE new_list util/*.h)
  SET(dir_list "")
  FOREACH(file_path ${new_list})
    GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
    SET(dir_list ${dir_list} ${dir_path})
  ENDFOREACH()
  LIST(REMOVE_DUPLICATES dir_list)
  SET(${return_list} ${dir_list})
ENDMACRO()

set(OSG_INCLUDE_DIRS  ${OSG_INCLUDE_DIR}
                      ${OSGUTIL_INCLUDE_DIR}
                      ${OSGVIEWER_INCLUDE_DIR}
                      ${OPENTHREADS_INCLUDE_DIR}
)

set(OSG_LIBS ${OSG_LIBRARIES}
             ${OSGUTIL_LIBRARIES}
             ${OSGVIEWER_LIBRARIES}
             ${OPENTHREADS_LIBRARIES}
)

LIB_HEADERS(PROJECT_LIB_DIRS)
UTIL_HEADERS(PROJECT_UTIL_DIRS)

set(FRAMEWORK_HEADERS ${PROJECT_LIB_DIRS} ${PROJECT_UTIL_DIRS})

#add_definitions(-c -msse3 -O3 -DNDEBUG)
add_definitions(-c -msse3 -O3 )


add_subdirectory(lib)
add_subdirectory(apps)
