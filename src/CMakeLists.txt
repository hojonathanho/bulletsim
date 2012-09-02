add_definitions("-DBULLETSIM_DATA_DIR=${BULLETSIM_SOURCE_DIR}/data")


include_directories("/opt/ros/fuerte/include")


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
)
link_directories(${BULLETSIM_BINARY_DIR}/src)
option(TESTS OFF)
option(JOHNS_ADVENTURES ON)
option(JONATHANS_ADVENTURES OFF)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_subdirectory(simulation)
add_subdirectory(robots)
add_subdirectory(utils)
add_subdirectory(comm)
add_subdirectory(clouds)

if (TESTS)
	add_subdirectory(tests)
endif(TESTS)
if (JOHNS_ADVENTURES)
   add_subdirectory(john_sandbox)
   add_subdirectory(sqp)
   #add_subdirectory(knots)
   add_subdirectory(tracking)
endif(JOHNS_ADVENTURES)
if (JONATHANS_ADVENTURES)
   add_subdirectory(flattening)
endif(JONATHANS_ADVENTURES)
add_subdirectory(lfd)
