include_directories(
    ${BULLET_DIR}/Extras
    ${BULLET_DIR}/Extras/HACD
    ${BULLETSIM_SOURCE_DIR}/lib/haptics
    ${BULLETSIM_SOURCE_DIR}/src
    ${PCL_INCLUDE_DIRS}
    ${TETGEN_DIR}
)

SET(CMAKE_CXX_FLAGS "-Wall -Wno-sign-compare -Wno-reorder")


FIND_PACKAGE( OpenCV REQUIRED )

add_library(simulation
    environment.cpp
    basicobjects.cpp
    openravesupport.cpp
    util.cpp
    simplescene.cpp
    softbodies.cpp
    softBodyHelpers.cpp
    rope.cpp
    hand.cpp
    plotting.cpp
    convexdecomp.cpp
    recording.cpp
    config_bullet.cpp
    config_viewer.cpp
    bullet_io.cpp
    fake_gripper.cpp
    mouse_picking.cpp
    tetgen_helpers.cpp
)

target_link_libraries(simulation
    utils
    haptics
    osgBullet
    osgWorks
    tetgen
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${OSG_LIBS}
    ${BULLET_LIBS}
    ${OpenRAVE_LIBRARIES}
    ${OpenRAVE_CORE_LIBRARIES}
    ${LOG4CPLUS_LIBRARY}
)
