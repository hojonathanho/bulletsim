find_package(PythonLibs REQUIRED)
include_directories(
  ${BULLET_DIR}/src
  ${OpenRAVE_INCLUDE_DIRS}
  ${BULLETSIM_SOURCE_DIR}/src
  ${OSGBULLET_DIR}
  #${WM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
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
)
target_link_libraries(lfd
  simulation
  robots
  boost_python
  ${PYTHON_LIBRARIES}
)

add_executable(lfd_rope lfd_rope.cpp)
target_link_libraries(lfd_rope lfd)
