include_directories(${BULLET_DIR}/src
                    ${BULLET_DIR}/Extras
                    ${OSGBULLET_DIR}
                    ${OSGWORKS_DIR}
                    ${OpenRAVE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${YAMLCPP_DIR}/include
                    ${EIGEN3_INCLUDE_DIR}
                    ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/haptics)

set(BULLET_LIBS BulletFileLoader BulletSoftBody BulletDynamics BulletCollision LinearMath)




if (BUILD_PERCEPTION)
  find_package(PCL 1.3)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif(BUILD_PERCEPTION)

add_library(simulation environment.cpp basicobjects.cpp openravesupport.cpp util.cpp simplescene.cpp softbodies.cpp rope.cpp plotting.cpp config.cpp config_bullet.cpp config_viewer.cpp)

target_link_libraries(simulation haptics osgBullet osgWorks ${Boost_LIBRARIES} ${OSG_LIBS} ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_executable(test_emptyscene test_emptyscene.cpp emptyscene.cpp)
target_link_libraries(test_emptyscene simulation)

add_executable(test_simplescene test_simplescene.cpp)
target_link_libraries(test_simplescene simulation)

add_executable(test_softbody test_softbody.cpp)
target_link_libraries(test_softbody simulation)

add_executable(test_cloth_grasping test_cloth_grasping.cpp)
target_link_libraries(test_cloth_grasping simulation)

add_executable(test_rigidbody_grasping test_rigidbody_grasping.cpp)
target_link_libraries(test_rigidbody_grasping simulation)

#add_executable(test_playback_joints test_playback_joints.cpp)
#target_link_libraries(test_playback_joints simulation)

add_executable(test_rope test_rope.cpp)
target_link_libraries(test_rope simulation)

if (BUILD_PERCEPTION)
  add_executable(test_plotting test_plotting.cpp plotting.cpp)
  target_link_libraries(test_plotting simulation ${PCL_LIBRARIES})
endif(BUILD_PERCEPTION)

add_executable(test_fork test_fork.cpp)
target_link_libraries(test_fork simulation)

add_executable(test_shirt test_shirt.cpp)
target_link_libraries(test_shirt simulation)

add_executable(test_collision_detection test_collision_detection.cpp)
target_link_libraries(test_collision_detection simulation)

add_executable(test_action test_action.cpp)
target_link_libraries(test_action simulation)
