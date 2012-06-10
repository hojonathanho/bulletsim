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
link_directories(
  ${BULLETSIM_BINARY_DIR}/src
  ${WM_LIBRARY_DIR}
)

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

add_executable(test_hydra_teleop test_hydra_teleop.cpp)
target_link_libraries(test_hydra_teleop simulation robots ${ROS_LIBRARIES})