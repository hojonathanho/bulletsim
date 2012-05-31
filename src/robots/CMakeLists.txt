include_directories(
    ${BULLETSIM_SOURCE_DIR}/src
    ${BULLET_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${BULLETSIM_SOURCE_DIR}/lib/haptics
    ${EIGEN3_INCLUDE_DIR}
    ${OSGBULLET_DIR}
    
)
link_directories(${BULLETSIM_BINARY_DIR}/src/simulation)

add_library(robots grabbing.cpp pr2.cpp ros2rave.cpp)
target_link_libraries(robots simulation)
