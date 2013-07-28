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

add_executable(test_frame test_frame.cpp)
target_link_libraries(test_frame simulation robots)

add_executable(test_logging test_logging.cpp)
target_link_libraries(test_logging simulation)

add_executable(test_tetgen test_tetgen.cpp)
target_link_libraries(test_tetgen simulation)

add_executable(test_picking test_picking.cpp)
target_link_libraries(test_picking simulation utils)

add_executable(test_hand test_hand.cpp)
target_link_libraries(test_hand simulation)

#add_executable(test_raven test_raven.cpp)
#target_link_libraries(test_raven robots simulation)

add_executable(test_softbody_mappings test_softbody_mappings.cpp)
target_link_libraries(test_softbody_mappings simulation)

################################################################
#include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/test_surgical)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/ravens)

#link_directories( ${CMAKE_CURRENT_SOURCE_DIR}/test_surgical )
#add_executable(test_suture_cloth test_suture_cloth.cpp
#                            	 test_surgical/CustomScene.cpp
#                            	 test_surgical/CustomKeyHandler.cpp
#                            	 test_surgical/RavePlanners.cpp)
#target_link_libraries(test_suture_cloth simulation robots)


# ---------------- BOOST PYTHON STUFF : LFD --------------------------
set(RAVENS_DEMO_DIR $ENV{HOME}/Dropbox/simruns)
set(RAVENS_STORE_DIR /media/0846DE7846DE65CA/suturing_runs)
add_definitions("-DRAVENS_DEMO_DIR=${RAVENS_DEMO_DIR}")
add_definitions("-DRAVENS_STORE_DIR=${RAVENS_STORE_DIR}")

add_subdirectory(ravens/lfd)

find_package(Boost COMPONENTS python)
include_directories (${Boost_INCLUDE_DIRS})
find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})


add_executable(ravens ravens/test_ravens.cpp
					  ravens/RavensRigidBodyAction.cpp
					  ravens/RavenGrabMonitor.cpp
                      ravens/CustomScene.cpp
                      ravens/jointPlayback.cpp
                      ravens/jointRecorder.cpp
                      ravens/jointLFDProcessor.cpp
                      ravens/CustomKeyHandler.cpp
                      ravens/RavenPlanners.cpp
                      ravens/ravens_config.cpp
                      ravens/SceneRecorder.cpp
                      ravens/ScenePlayer.cpp
                      ravens/BoxCloth.cpp)
target_link_libraries(ravens simulation robots lfd)
add_executable(test_lfd_wrapper ravens/test_lfd_wrapper.cpp )
target_link_libraries(test_lfd_wrapper simulation lfd)

#------------------------------------------------------------------
add_executable(test_boxcloth ravens/test_boxcloth.cpp
                      ravens/BoxCloth.cpp)
target_link_libraries(test_boxcloth simulation lfd)



add_executable(test_grid ravens/test_grid.cpp)
target_link_libraries(test_grid simulation)



add_executable(rave_cpp test_surgical/planning.cpp)
target_link_libraries(rave_cpp simulation robots)

add_executable(rave_plan test_surgical/pr2turnlever.cpp)
target_link_libraries(rave_plan simulation robots)
