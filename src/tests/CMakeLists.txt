include_directories(${BULLETSIM_SOURCE_DIR}/src)
link_directories(${BULLETSIM_BINARY_DIR}/src)

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
  if (NOT DEFINED PCL)
    find_package(PCL 1.3)
  endif()
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  set_property(
    SOURCE plotting.cpp test_plotting.cpp
    PROPERTY COMPILE_DEFINITIONS BUILD_PERCEPTION)
  add_executable(test_plotting test_plotting.cpp)
  target_link_libraries(test_plotting simulation ${PCL_LIBRARIES})
endif(BUILD_PERCEPTION)

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

add_executable(test_scene_record test_scene_record.cpp ../recording.cpp)
target_link_libraries(test_scene_record simulation)
