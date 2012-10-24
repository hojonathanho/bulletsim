find_package(PythonLibs REQUIRED)
find_package(HDF5 REQUIRED)

include_directories(
  ${BULLET_DIR}/src
  ${OpenRAVE_INCLUDE_DIRS}
  ${BULLETSIM_SOURCE_DIR}/src
  ${OSGBULLET_DIR}
  #${WM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIR}
)
link_directories(
  ${BULLETSIM_BINARY_DIR}/src
  ${PCL_LIBRARY_DIRS}
  ${WM_LIBRARY_DIR}
)
add_definitions(${PCL_DEFINITIONS})

add_library(lfd
  task_execution.cpp
  utils_python.cpp
  lfd_python_wrapper.cpp
  rope_scenes.cpp
  lfd_rope_common.cpp
  dynamics.cpp
)
target_link_libraries(lfd
  simulation
  robots
  boost_python
  ${PYTHON_LIBRARIES}
)

add_executable(lfd_rope lfd_rope.cpp)
target_link_libraries(lfd_rope lfd)

add_executable(record_sim_tracking record_sim_tracking.cpp)
target_link_libraries(record_sim_tracking lfd hdf5_cpp hdf5)

add_executable(test_simpletrajoptimizer test_simpletrajoptimizer.cpp)
target_link_libraries(test_simpletrajoptimizer lfd)
