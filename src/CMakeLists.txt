add_definitions("-DBULLETSIM_DATA_DIR=${BULLETSIM_SOURCE_DIR}/data")
add_definitions("-DBULLETSIM_SRC_DIR=${BULLETSIM_SOURCE_DIR}/src")


include_directories(
    ${BULLETSIM_SOURCE_DIR}/src

    ${BULLET_DIR}/src
    ${OSGBULLET_DIR}
    ${OSGWORKS_DIR}
    ${OpenRAVE_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
    ${LOG4CPLUS_INCLUDE_DIRS}
    ${ROS_ROOT_DIR}/include
    ${PHASESPACE_INCLUDE_DIR}
)
link_directories(${BULLETSIM_BINARY_DIR}/src)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_subdirectory(simulation)
add_subdirectory(robots)
add_subdirectory(utils)
add_subdirectory(comm)
add_subdirectory(clouds)

if (BUILD_TESTS)
	add_subdirectory(tests)
endif (BUILD_TESTS)

if (BUILD_TRACKING)
  add_subdirectory(tracking)
	add_subdirectory(phasespace)
endif (BUILD_TRACKING)
