include_directories(${BULLET_DIR}/src
                    ${BULLET_DIR}/Extras
                    ${BULLET_DIR}/Extras/HACD
                    ${OSGBULLET_DIR}
                    ${OSGWORKS_DIR}
                    ${OpenRAVE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${YAMLCPP_DIR}/include
                    ${EIGEN3_INCLUDE_DIR}
                    ${BULLETSIM_SOURCE_DIR}/lib/haptics
                    ../utils
)

set(BULLET_LIBS BulletFileLoader BulletSoftBody BulletDynamics BulletCollision LinearMath HACD)

#if (BUILD_PERCEPTION)
#  find_package(PCL 1.5)
#  include_directories(${PCL_INCLUDE_DIRS})
#  link_directories(${PCL_LIBRARY_DIRS})
#  add_definitions(${PCL_DEFINITIONS})
#endif(BUILD_PERCEPTION)

set_property(
  DIRECTORY
  PROPERTY COMPILE_DEFINITIONS BULLETSIM_DATA_DIR=${BULLETSIM_SOURCE_DIR}/data)

if (BUILD_PERCEPTION)
  if (NOT DEFINED PCL)
    find_package(PCL 1.3)
  endif()
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  set_property(
    SOURCE plotting.cpp
    PROPERTY COMPILE_DEFINITIONS BUILD_PERCEPTION)
endif(BUILD_PERCEPTION)

add_library(simulation environment.cpp basicobjects.cpp openravesupport.cpp util.cpp simplescene.cpp softbodies.cpp rope.cpp plotting.cpp pr2.cpp convexdecomp.cpp grabbing.cpp
                       config.cpp config_bullet.cpp config_viewer.cpp recording.cpp)

target_link_libraries(simulation haptics osgBullet osgWorks ${Boost_LIBRARIES} ${OSG_LIBS} ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_subdirectory(tests)
