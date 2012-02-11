set(WM_ROOT "~/Src/GeometricTools/WildMagic5")
set(WM_INCLUDE_DIR "${WM_ROOT}/SDK/Include")
set(WM_LIBRARY_DIR "${WM_ROOT}/SDK/Library/Debug")
set(WM_LIBRARIES Wm5Core Wm5Imagics Wm5Mathematics Wm5Physics)
set(JSON_INCLUDE_DIR "${JSON_DIR}/include")
# set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}/include")
# set(GUROBI_LIBRARY_DIR "$ENV{GUROBI_HOME}/lib")

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_GLIBCXX_DEBUG")

set_property(
  DIRECTORY
  PROPERTY COMPILE_DEFINITIONS BUILD_PERCEPTION BULLETSIM_DATA_DIR=${SURGICAL_PHYSICS_SOURCE_DIR}/data)
add_definitions(PCL_DEFINITIONS)

include_directories(${BULLET_DIR}/src
                    ${OSGBULLET_DIR}
                    ${OSGWORKS_DIR}
                    ${OpenRAVE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
		    ${EIGEN3_INCLUDE_DIR}
#		    ${WM_INCLUDE_DIR}
                    ${BULLETSIM_SOURCE_DIR}/lib/haptics
		    ${PCL_INCLUDE_DIRS}
		    ${BULLETSIM_SOURCE_DIR}/src
		    ${JSON_INCLUDE_DIR}
#		    ${GUROBI_INCLUDE_DIR}
            ${TETGEN_DIR}
            ../utils
            clouds
            comm
            .
)

set(BULLET_LIBS BulletSoftBody BulletDynamics BulletCollision LinearMath)
link_directories(
  ${PCL_LIBRARY_DIRS} 
  ${WM_LIBRARY_DIR}
#  ${GUROBI_LIBRARY_DIR}
  ${TETGEN_DIR}
)


# add_library(perception tracking.cpp utils_perception.cpp dist_math.cpp)
# target_link_libraries(perception ${PCL_LIBRARIES} simulation)

#add_executable(test_kinect_rope1 test_kinect_rope.cpp)
#target_link_libraries(test_kinect_rope1 simulation perception clouds ${PCL_LIBRARIES} json comm)



#add_executable(test_eigen_inc test_eigen_inc.cpp)

# add_executable(test_kinect_towel test_kinect_towel.cpp)
# target_link_libraries(test_kinect_towel ${PCL_LIBRARIES} json simulation perception clouds comm)

add_library(perception ../utils/vector_io.cpp config_perception.cpp utils_perception.cpp make_bodies.cpp bullet_io.cpp openrave_joints.cpp robot_geometry.cpp dist_math.cpp optimization_forces.cpp sparse_array.cpp apply_impulses.cpp visibility.cpp trackers.cpp matching.cpp)
target_link_libraries(perception ${PCL_LIBRARIES} comm2 clouds simulation  tetgen)

add_executable(test_matching test_matching.cpp matching.cpp)
target_link_libraries(test_matching  perception)


add_executable(test_prob_corr test_prob_corr.cpp)
target_link_libraries(test_prob_corr perception clouds simulation)

add_executable(test_pr2_playback test_pr2_playback.cpp)
target_link_libraries(test_pr2_playback ${PCL_LIBRARIES} simulation comm2 clouds perception)

add_executable(test_towel_local_opt test_towel_local_opt.cpp update_bodies.cpp clouds/utils_pcl.cpp)
target_link_libraries(test_towel_local_opt comm2 ${PCL_LIBRARIES} simulation clouds perception)

add_executable(test_towel_track test_towel_track.cpp clouds/utils_pcl.cpp get_nodes.cpp)
target_link_libraries(test_towel_track comm2 ${PCL_LIBRARIES} simulation clouds perception)

add_executable(test_tetra_track test_tetra_track.cpp clouds/utils_pcl.cpp get_nodes.cpp)
target_link_libraries(test_tetra_track comm2 ${PCL_LIBRARIES} simulation clouds perception)

#add_executable(test_eigen test_eigen.cpp bullet_io.cpp)
#target_link_libraries(test_eigen simulation)

add_executable(test_kinect_rope test_kinect_rope.cpp clouds/utils_pcl.cpp) # need utils_pcl because of bizarre linker error
target_link_libraries(test_kinect_rope clouds comm2 simulation comm2 clouds perception)

add_executable(test_pr2_knot test_pr2_knot.cpp clouds/utils_pcl.cpp ../src/grabbing.cpp) # need utils_pcl because of bizarre linker error
target_link_libraries(test_pr2_knot clouds comm2 simulation comm2 clouds perception)

add_executable(test_pr2_towel test_pr2_towel.cpp clouds/utils_pcl.cpp ../src/grabbing.cpp get_nodes.cpp) # need utils_pcl because of bizarre linker error
target_link_libraries(test_pr2_towel clouds comm2 simulation perception ${PCL_LIBRARIES})

add_executable(calibrate_kinect calibrate_kinect clouds/utils_pcl.cpp ../src/grabbing.cpp get_nodes.cpp) # need utils_pcl because of bizarre linker error
target_link_libraries(calibrate_kinect clouds comm2 simulation perception ${PCL_LIBRARIES})

add_executable(test_tetgen test_tetgen.cpp ../lib/tetgen-1.4.3/tetgen.h)
target_link_libraries(test_tetgen tetgen)

add_subdirectory(comm)
add_subdirectory(clouds)
