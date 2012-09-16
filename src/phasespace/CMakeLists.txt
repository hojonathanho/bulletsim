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

add_library(phasespace 
	phasespace.cpp
	phasespace_utils.cpp
	config_phasespace.cpp
)
	
target_link_libraries(phasespace
	clouds simulation tracking utils ${PHASESPACE_LIBS} ${ROS_LIBRARIES})
	
add_executable(phasespace_publisher phasespace_publisher.cpp)
target_link_libraries(phasespace_publisher phasespace)

add_executable(phasespace_simple_publisher phasespace_simple_publisher.cpp)
target_link_libraries(phasespace_simple_publisher phasespace)

add_executable(phasespace_subscriber phasespace_subscriber.cpp)
target_link_libraries(phasespace_subscriber phasespace)

add_executable(test_phasespace test_phasespace.cpp)
target_link_libraries(test_phasespace phasespace)

add_executable(test_phasespace_rigid test_phasespace_rigid.cpp)
target_link_libraries(test_phasespace_rigid phasespace)

add_executable(phasespace_kinect_calibration phasespace_kinect_calibration.cpp)
target_link_libraries(phasespace_kinect_calibration phasespace ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable(phasespace_transforms_node phasespace_transforms_node.cpp)
target_link_libraries(phasespace_transforms_node phasespace clouds ${PCL_LIBRARIES} boost_signals tf)

add_executable(tf_phasespace_robot tf_phasespace_robot.cpp)
target_link_libraries(tf_phasespace_robot phasespace tf clouds)
