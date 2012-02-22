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

if(BUILD_PERCEPTION)
  add_definitions(${PCL_DEFINITIONS})

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

  add_executable(test_preproc test_preproc.cpp ../clouds/preprocessing.cpp)
  target_link_libraries(test_preproc clouds gomp comm)

  add_executable(test_preproc2 test_preproc2.cpp ../clouds/preprocessing.cpp)
  target_link_libraries(test_preproc2 clouds comm perception simulation gomp)

  
  add_executable(cloudptr_share cloudptr_share.cpp)
  target_link_libraries(cloudptr_share clouds comm)
  
#   add_executable(distance_benchmark distance_benchmark.cpp)
#   target_link_libraries(distance_benchmark perception simulation)
endif(BUILD_PERCEPTION)