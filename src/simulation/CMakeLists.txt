include_directories(
    ${BULLET_DIR}/Extras
    ${BULLET_DIR}/Extras/HACD
    ${BULLETSIM_SOURCE_DIR}/lib/haptics
    ${BULLETSIM_SOURCE_DIR}/src
    ${TETGEN_DIR}
)

#SET(CMAKE_CXX_FLAGS "-Wall -Wno-sign-compare -Wno-reorder")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -fvisibility=default")


add_library(simulation
    environment.cpp
    basicobjects.cpp
    openravesupport.cpp
    util.cpp
#    softbodies.cpp
#    softBodyHelpers.cpp
#    rope.cpp
    config_bullet.cpp
    bullet_io.cpp
#    tetgen_helpers.cpp
    logging.cpp
    config.cpp
    conversions.cpp
    utils_vector.cpp
    bulletsim_lite.cpp
)

target_link_libraries(simulation
#    utils
    #haptics
    #tetgen
    ${Boost_LIBRARIES}
    ${BULLET_LIBS}
    ${OpenRAVE_LIBRARIES}
    ${OpenRAVE_CORE_LIBRARIES}
    ${LOG4CPLUS_LIBRARY}
)

boost_python_module(cbulletsimpy bulletsimpy.cpp)
target_link_libraries(cbulletsimpy simulation)
