include_directories( ${TETGEN_DIR}
    ${PCL_INCLUDE_DIRS} )

link_directories( ${TETGEN_DIR} )

add_executable(test_simplescene test_simplescene.cpp)
target_link_libraries(test_simplescene simulation robots)

add_executable(test_softbody test_softbody.cpp)
target_link_libraries(test_softbody simulation)

add_executable(test_cloth_grasping test_cloth_grasping.cpp)
target_link_libraries(test_cloth_grasping simulation robots)

add_executable(test_rigidbody_grasping test_rigidbody_grasping.cpp)
target_link_libraries(test_rigidbody_grasping simulation robots)

#add_executable(test_playback_joints test_playback_joints.cpp)
#target_link_libraries(test_playback_joints simulation)

add_executable(test_rope test_rope.cpp)
target_link_libraries(test_rope simulation robots)

add_executable(test_rope_fork test_rope_fork.cpp)
target_link_libraries(test_rope_fork simulation robots)

add_executable(test_fork test_fork.cpp)
target_link_libraries(test_fork simulation)

add_executable(test_fork_softbody test_fork_softbody.cpp)
target_link_libraries(test_fork_softbody simulation)

add_executable(test_shirt test_shirt.cpp)
target_link_libraries(test_shirt simulation)

add_executable(test_collision_detection test_collision_detection.cpp)
target_link_libraries(test_collision_detection simulation)

add_executable(test_action test_action.cpp)
target_link_libraries(test_action simulation)

add_executable(test_scene_record test_scene_record.cpp)
target_link_libraries(test_scene_record simulation)

add_executable(test_logging test_logging.cpp)
target_link_libraries(test_logging simulation)

add_executable(test_tetgen test_tetgen.cpp)
target_link_libraries(test_tetgen simulation)

add_executable(test_picking test_picking.cpp)
target_link_libraries(test_picking simulation utils)

add_executable(test_hand test_hand.cpp)
target_link_libraries(test_hand simulation)
