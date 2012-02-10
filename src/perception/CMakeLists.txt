set_property(
  DIRECTORY
  PROPERTY COMPILE_DEFINITIONS BUILD_PERCEPTION BULLETSIM_DATA_DIR=${BULLETSIM_SOURCE_DIR}/data)

add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")

if (NOT DEFINED PCL_INCLUDE_DIRS)
  find_package(PCL 1.4 REQUIRED)
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
    ${BULLET_DIR}/src
    ${OSGBULLET_DIR}
    ${OSGWORKS_DIR}
    ${OpenRAVE_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
#		    ${WM_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${BULLETSIM_SOURCE_DIR}/src
    ${JSON_INCLUDE_DIR}
    ${GUROBI_INCLUDE_DIR}
    ${TETGEN_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS} 
  ${WM_LIBRARY_DIR}
  ${GUROBI_LIBRARY_DIR}
  ${TETGEN_DIR}
)
add_definitions(${PCL_DEFINITIONS})

# add_library(perception tracking.cpp utils_perception.cpp dist_math.cpp)
# target_link_libraries(perception ${PCL_LIBRARIES} simulation)

#add_executable(test_kinect_rope1 test_kinect_rope.cpp)
#target_link_libraries(test_kinect_rope1 simulation perception clouds ${PCL_LIBRARIES} json comm)



#add_executable(test_eigen_inc test_eigen_inc.cpp)

# add_executable(test_kinect_towel test_kinect_towel.cpp)
# target_link_libraries(test_kinect_towel ${PCL_LIBRARIES} json simulation perception clouds comm)

add_library(perception config_perception.cpp utils_perception.cpp make_bodies.cpp bullet_io.cpp openrave_joints.cpp robot_geometry.cpp dist_math.cpp optimization_forces.cpp sparse_array.cpp apply_impulses.cpp visibility.cpp trackers.cpp matching.cpp get_nodes.cpp)
target_link_libraries(perception ${PCL_LIBRARIES} comm2 clouds simulation gurobi46 gurobi_c++ pthread tetgen utils)

add_executable(test_matching test_matching.cpp matching.cpp)
target_link_libraries(test_matching perception)

add_executable(test_prob_corr test_prob_corr.cpp)
target_link_libraries(test_prob_corr perception)

add_executable(test_pr2_playback test_pr2_playback.cpp)
target_link_libraries(test_pr2_playback ${PCL_LIBRARIES} simulation comm2 clouds perception robots)

add_executable(test_towel_local_opt test_towel_local_opt.cpp update_bodies.cpp)
target_link_libraries(test_towel_local_opt comm2 ${PCL_LIBRARIES} simulation clouds perception)

add_executable(test_towel_track test_towel_track.cpp)
target_link_libraries(test_towel_track comm2 ${PCL_LIBRARIES} simulation clouds perception)

add_executable(test_tetra_track test_tetra_track.cpp)
target_link_libraries(test_tetra_track comm2 ${PCL_LIBRARIES} simulation clouds perception)

#add_executable(test_eigen test_eigen.cpp bullet_io.cpp)
#target_link_libraries(test_eigen simulation)

add_executable(test_kinect_rope test_kinect_rope.cpp)
target_link_libraries(test_kinect_rope clouds comm2 simulation comm2 clouds perception robots)

add_executable(test_pr2_knot test_pr2_knot.cpp)
target_link_libraries(test_pr2_knot clouds comm2 simulation comm2 clouds perception robots)

add_executable(test_pr2_towel test_pr2_towel.cpp)
target_link_libraries(test_pr2_towel clouds comm2 simulation perception robots ${PCL_LIBRARIES})

add_executable(calibrate_kinect calibrate_kinect)
target_link_libraries(calibrate_kinect clouds comm2 simulation perception robots ${PCL_LIBRARIES})

add_executable(test_tetgen test_tetgen.cpp ${TETGEN_DIR}/tetgen.h)
target_link_libraries(test_tetgen tetgen)
