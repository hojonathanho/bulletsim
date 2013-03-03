add_definitions("-DBULLETSIM_DATA_DIR=${BULLETSIM_SOURCE_DIR}/data")


include_directories(
    ${BULLETSIM_SOURCE_DIR}/src

    ${BULLET_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
    ${LOG4CPLUS_INCLUDE_DIRS}
)
link_directories(${BULLETSIM_BINARY_DIR}/src)
option(BUILD_TESTS OFF)
option(JOHNS_ADVENTURES OFF)
option(JONATHANS_ADVENTURES OFF)
option(ALEXS_ADVENTURES ON)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_subdirectory(simulation)
#add_subdirectory(robots)
#add_subdirectory(utils)

if (BUILD_TESTS)
	add_subdirectory(tests)
endif(BUILD_TESTS)
