include_directories(
    ${BULLET_DIR}/Extras
    ${BULLET_DIR}/Extras/HACD
    ${BULLETSIM_SOURCE_DIR}/lib/haptics
    ${BULLETSIM_SOURCE_DIR}/src
)

add_library(simulation environment.cpp basicobjects.cpp openravesupport.cpp util.cpp simplescene.cpp softbodies.cpp rope.cpp plotting.cpp convexdecomp.cpp recording.cpp config_bullet.cpp config_viewer.cpp bullet_io.cpp)

target_link_libraries(simulation utils haptics osgBullet osgWorks ${Boost_LIBRARIES} ${OSG_LIBS} ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
