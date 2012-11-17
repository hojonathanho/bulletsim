include_directories(
    ${BULLET_DIR}/src
    ${OSGBULLET_DIR}
    ${OSGWORKS_DIR}
    ${OpenRAVE_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${BULLETSIM_SOURCE_DIR}/src
    ${JSON_INCLUDE_DIR}
    ${GUROBI_INCLUDE_DIR}
    ${TETGEN_DIR}
    ${BULLETSIM_SOURCE_DIR}/src
)

include_directories("/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/include")
include_directories("/opt/ros/fuerte/stacks/geometry/tf/include"
					"/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include"
					"/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include")
link_directories("/opt/ros/fuerte/stacks/geometry/tf/lib"
	"/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/lib")


link_directories(
  ${BULLETSIM_BINARY_DIR}/src
)

include_directories(/home/joschu/Src/gurobi501/linux64/include)
link_directories(/home/joschu/Src/gurobi501/linux64/lib)

add_executable(test_plotting test_plotting.cpp)
target_link_libraries(test_plotting simulation)

add_executable(test_eigen_threads test_eigen_threads.cpp)

add_executable(test_plotting2 test_plotting2.cpp)
target_link_libraries(test_plotting2 simulation)

add_executable(test_multithread_ik test_multithread_ik.cpp)
target_link_libraries(test_multithread_ik simulation robots)

add_executable(test_numerical_ik test_numerical_ik.cpp)
target_link_libraries(test_numerical_ik simulation robots)

add_executable(test_kb_control kb_control.cpp)
target_link_libraries(test_kb_control simulation robots)

add_executable(test_rave_load test_rave_load.cpp)
target_link_libraries(test_rave_load simulation robots)

add_executable(test_dynamic_pr2 test_dynamic_pr2.cpp)
target_link_libraries(test_dynamic_pr2 simulation robots)

add_executable(test_ros test_ros.cpp)
target_link_libraries(test_ros simulation robots ${ROS_LIBRARIES})

#add_executable(test_hydra_teleop test_hydra_teleop.cpp)
#target_link_libraries(test_hydra_teleop simulation robots ${ROS_LIBRARIES})

add_executable(test_shadow_hand test_shadow_hand.cpp)
target_link_libraries(test_shadow_hand simulation utils)

add_executable(test_surface_sampling test_surface_sampling.cpp)
target_link_libraries(test_surface_sampling simulation utils clouds tracking)

add_executable(get_cloud_robot_frame get_cloud_robot_frame.cpp)
target_link_libraries(get_cloud_robot_frame utils ${ROS_LIBRARIES} clouds tf robots)

add_executable(test_recording test_recording.cpp)
target_link_libraries(test_recording simulation)

add_executable(try_config_vectors try_config_vectors.cpp)
target_link_libraries(try_config_vectors simulation)

find_package(PythonLibs REQUIRED)
add_executable(test_box_overlap test_box_overlap.cpp)
target_link_libraries(test_box_overlap simulation ${PYTHON_LIBRARIES} boost_python sqp2)

add_executable(test_grab test_grab.cpp)
target_link_libraries(test_grab simulation robots)