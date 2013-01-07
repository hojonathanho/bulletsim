include_directories(
    ${PCL_INCLUDE_DIRS}
    ${BULLETSIM_SOURCE_DIR}/src
)

include_directories( ${TETGEN_DIR}
    ${PCL_INCLUDE_DIRS} )

link_directories( ${TETGEN_DIR} )

include_directories("/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/include") # XXX bad
include_directories("/opt/ros/fuerte/stacks/geometry/tf/include"
					"/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include"
					"/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include") # fuck this shit
link_directories("/opt/ros/fuerte/stacks/geometry/tf/lib"
	"/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/lib")

add_library(tracking 
	algorithm_common.cpp 
	config_tracking.cpp
	initialization.cpp
	plotting_tracking.cpp
	simple_physics_tracker.cpp 
	physics_tracker.cpp 
	stochastic_tracker.cpp
	multihyp_tracker.cpp 
	surface_sampling.cpp 
	sparse_utils.cpp
	tracked_object.cpp 
	tracked_rope.cpp
	tracked_cloth.cpp
	tracked_towel.cpp
	tracked_sponge.cpp
	tracked_box.cpp
	utils_tracking.cpp
	visibility.cpp
	feature_extractor.cpp
	tracked_compound.cpp
  cam_sync.cpp
)
	
target_link_libraries(tracking
	simulation clouds robots cv_bridge)
	
add_executable(tracker_node tracker_node.cpp)
target_link_libraries(tracker_node tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${ROS_LIBRARIES} boost_signals)

add_executable(robot_tracker_node robot_tracker_node.cpp)
target_link_libraries(robot_tracker_node tracking simulation clouds tf cv_bridge ${ROS_LIBRARIES} boost_signals)

#add_executable(threaded_tracker_node threaded_tracker_node.cpp)
#target_link_libraries(threaded_tracker_node tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${ROS_LIBRARIES} boost_signals)

add_executable(simulated_tracker simulated_tracker.cpp)
target_link_libraries(simulated_tracker tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${ROS_LIBRARIES} boost_signals)

add_executable(simulated_hand_tracker simulated_hand_tracker.cpp)
target_link_libraries(simulated_hand_tracker tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${ROS_LIBRARIES} boost_signals)

add_executable(hand_tracker_node hand_tracker_node.cpp)
target_link_libraries(hand_tracker_node tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${ROS_LIBRARIES} boost_signals)

#add_executable(test_pairwise_dist_scaled sandbox/test_pairwise_dist_scaled.cpp ../utils/clock.cpp)
#target_link_libraries(test_pairwise_dist_scaled gomp)
