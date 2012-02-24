include_directories(
    ${BULLET_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${BULLETSIM_SOURCE_DIR}/src
    ${OSGBULLET_DIR}
)
link_directories(${BULLETSIM_BINARY_DIR}/src)

add_executable(test_plotting test_plotting.cpp)
target_link_libraries(test_plotting simulation)

add_executable(test_plotting2 test_plotting2.cpp)
target_link_libraries(test_plotting2 simulation)

