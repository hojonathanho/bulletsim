include_directories(${BULLET_DIR}/src
                    ${OSGBULLET_DIR}
                    ${OSGWORKS_DIR}
                    ${OpenRAVE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
		    ${EIGEN3_INCLUDE_DIR}
                    ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/haptics)
set(BULLET_LIBS BulletSoftBody BulletDynamics BulletCollision LinearMath)

add_library(simulation environment.cpp basicobjects.cpp openravesupport.cpp util.cpp simplescene.cpp softbodies.cpp rope.cpp)

target_link_libraries(simulation haptics osgBullet osgWorks ${OSG_LIBS} ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(simplescene_test simplescene_test.cpp)
target_link_libraries(simplescene_test simulation)

add_executable(softbody_test softbody_test.cpp)
target_link_libraries(softbody_test simulation)

#add_executable(test_playback_joints test_playback_joints.cpp)
#target_link_libraries(test_playback_joints simulation)

add_executable(test_rope test_rope.cpp)
target_link_libraries(test_rope simulation)

find_package(PCL 1.3)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(test_kinect_rope test_kinect_rope.cpp dist_math.cpp plotting.cpp)
target_link_libraries(test_kinect_rope simulation ${PCL_LIBRARIES})

add_executable(test_plotting test_plotting.cpp plotting.cpp)
target_link_libraries(test_plotting simulation ${PCL_LIBRARIES})

add_executable(test_dist_math test_dist_math.cpp dist_math.cpp)