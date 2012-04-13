include_directories(
#   ${WM_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${GUROBI_INCLUDE_DIR}
    ${TETGEN_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS} 
  ${WM_LIBRARY_DIR}
  ${GUROBI_LIBRARY_DIR}
  ${TETGEN_DIR}
)

add_library(perception config_perception.cpp utils_perception.cpp make_bodies.cpp robot_geometry.cpp dist_math.cpp optimization_forces.cpp sparse_array.cpp apply_impulses.cpp visibility.cpp trackers.cpp matching.cpp get_nodes.cpp plotting_perception.cpp vision.cpp multihyp_vision.cpp)
target_link_libraries(perception ${PCL_LIBRARIES} comm clouds simulation tetgen utils)

add_executable(test_matching test_matching.cpp matching.cpp)
target_link_libraries(test_matching perception)

add_executable(test_prob_corr test_prob_corr.cpp)
target_link_libraries(test_prob_corr perception)

add_executable(test_pr2_playback test_pr2_playback.cpp)
target_link_libraries(test_pr2_playback ${PCL_LIBRARIES} simulation comm clouds perception robots)

add_executable(test_towel_track test_towel_track.cpp)
target_link_libraries(test_towel_track comm ${PCL_LIBRARIES} simulation clouds perception)

add_executable(test_tetra_track test_tetra_track.cpp)
target_link_libraries(test_tetra_track comm ${PCL_LIBRARIES} simulation clouds perception)

#add_executable(test_eigen test_eigen.cpp bullet_io.cpp)
#target_link_libraries(test_eigen simulation)

add_executable(test_kinect_rope test_kinect_rope.cpp)
target_link_libraries(test_kinect_rope clouds comm simulation comm clouds perception robots)

add_executable(test_pr2_knot test_pr2_knot.cpp)
target_link_libraries(test_pr2_knot clouds comm simulation comm clouds perception robots)

# add_executable(test_pr2_towel test_pr2_towel.cpp)
# target_link_libraries(test_pr2_towel clouds comm simulation perception robots ${PCL_LIBRARIES})
# commented for Monitor refactoring

add_executable(test_tetgen test_tetgen.cpp ${TETGEN_DIR}/tetgen.h)
target_link_libraries(test_tetgen tetgen)

add_executable(test_fake_kinect test_fake_kinect.cpp fake_kinect.cpp)
target_link_libraries(test_fake_kinect simulation comm perception clouds robots)

add_executable(calibrate_kinect calibrate_kinect)
target_link_libraries(calibrate_kinect clouds comm simulation perception robots ${PCL_LIBRARIES} boost_program_options)


# add_executable(test_new_towel_tracking test_new_towel_tracking.cpp vision.cpp)
# target_link_libraries(test_new_towel_tracking comm ${PCL_LIBRARIES} simulation clouds perception)

 add_executable(test_new_tracking2 test_new_tracking2.cpp)
 target_link_libraries(test_new_tracking2 comm ${PCL_LIBRARIES} simulation clouds perception)

add_executable(comm_track_towel comm_track_towel.cpp)
target_link_libraries(comm_track_towel perception clouds simulation utils comm gomp)
