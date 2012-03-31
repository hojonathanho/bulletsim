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

add_executable(eigen_threads eigen_threads.cpp)

add_executable(test_plotting2 test_plotting2.cpp)
target_link_libraries(test_plotting2 simulation)

add_executable(kb_control kb_control.cpp)
target_link_libraries(kb_control simulation robots)


  # add_executable(test_preproc test_preproc.cpp ../clouds/preprocessing.cpp)
  # target_link_libraries(test_preproc clouds gomp comm)

  # add_executable(test_preproc2 test_preproc2.cpp ../clouds/preprocessing.cpp)
  # target_link_libraries(test_preproc2 clouds comm perception simulation gomp)

  #add_executable(cloudptr_share cloudptr_share.cpp)
  #target_link_libraries(cloudptr_share clouds comm)

  #add_executable(depth_image depth_image.cpp)
  #target_link_libraries(depth_image clouds simulation robots)

#   add_executable(distance_benchmark distance_benchmark.cpp)
#   target_link_libraries(distance_benchmark perception simulation)

add_executable(ik_benchmark ik_benchmark.cpp)
target_link_libraries(ik_benchmark simulation robots)

add_executable(numerical_ik numerical_ik.cpp)
target_link_libraries(numerical_ik simulation robots)

add_executable(fake_gripper fake_gripper.cpp)
target_link_libraries(fake_gripper simulation robots)