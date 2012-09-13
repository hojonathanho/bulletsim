include_directories(
  ${PCL_INCLUDE_DIRS}
  ${BULLETSIM_SOURCE_DIR}/src
  ${ROS_ROOT_DIR}/include
  ${ROS_ROOT_DIR}/stacks/vision_opencv/cv_bridge/include
  ${ROS_ROOT_DIR}/stacks/geometry/tf/include
  ${ROS_ROOT_DIR}/stacks/geometry/tf/msg_gen/cpp/include
  ${ROS_ROOT_DIR}/stacks/geometry/tf/srv_gen/cpp/include
)

link_directories(
  ${ROS_ROOT_DIR}/stacks/geometry/tf/lib
	${ROS_ROOT_DIR}/stacks/vision_opencv/cv_bridge/lib
)

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
