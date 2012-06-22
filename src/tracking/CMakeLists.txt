include_directories(
    ${PCL_INCLUDE_DIRS}
#    ${WM_INCLUDE_DIR}
#    ${BULLET_INCLUDE_DIR}
#    ${JSON_INCLUDE_DIR}
#    ${EIGEN3_INCLUDE_DIR}
    ${BULLETSIM_SOURCE_DIR}/src
#    ${BULLET_DIR}/src
)


include_directories("/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/include") # XXX bad
include_directories("/opt/ros/fuerte/stacks/geometry/tf/include"
					"/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include"
					"/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include") # fuck this shit
link_directories("/opt/ros/fuerte/stacks/geometry/tf/lib"
	"/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/lib")
link_directories(${WM_LIBRARY_DIR})

add_library(tracking 
	algorithm_common.cpp 
	config_tracking.cpp
	initialization.cpp
	plotting_tracking.cpp
	simple_physics_tracker.cpp 
	sparse_utils.cpp
	tracked_object.cpp 
	tracked_rope.cpp
	tracked_towel.cpp
	tracked_box.cpp
	utils_tracking.cpp
	visibility.cpp
)
	
target_link_libraries(tracking
	simulation clouds robots)
	
#add_executable(test_sparse sandbox/test_sparse.cpp)
#target_link_libraries(test_sparse tracking)

add_executable(tracker_node tracker_node.cpp)
target_link_libraries(tracker_node tracking simulation clouds tf cv_bridge ${OpenCV_LIBS} ${ROS_LIBRARIES} boost_signals)
