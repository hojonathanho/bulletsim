include_directories(
    ${PCL_INCLUDE_DIRS}
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

add_library(experiments 
	message_conversions.cpp
)
	
target_link_libraries(experiments
	clouds simulation tracking utils ${PHASESPACE_LIBS} ${ROS_LIBRARIES})
