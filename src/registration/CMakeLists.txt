include_directories(
    ${PCL_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${BULLETSIM_SOURCE_DIR}/src
    /opt/ros/fuerte/stacks/visualization/interactive_markers/include
)
link_directories(
  ${PCL_LIBRARY_DIRS} 
  /opt/ros/fuerte/stacks/visualization/interactive_markers/lib
)

add_library(registration registration.cpp)
target_link_libraries(registration simulation)
add_executable(test_plotting3 test_plotting3.cpp)
target_link_libraries(test_plotting3 ${OSG_LIBS} clouds registration simulation tracking)

add_executable(test_interactive_markers test_interactive_markers.cpp)
target_link_libraries(test_interactive_markers interactive_markers ${ROS_LIBRARIES} boost_thread utils ${PCL_LIBRARIES} clouds)
