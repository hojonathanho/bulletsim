include_directories(
#  ${WM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
#  ${GUROBI_INCLUDE_DIR}
  ${TETGEN_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS} 
  ${WM_LIBRARY_DIR}
#  ${GUROBI_LIBRARY_DIR}
  ${TETGEN_DIR}
)

add_library(knots knots.cpp utils_python.cpp arm_base_traj.cpp trajectory_library.cpp numpy_boost.cpp)
target_link_libraries(knots simulation perception robots boost_python python2.6)

add_executable(record_knot_tie record_knot_tie.cpp)
target_link_libraries(record_knot_tie simulation robots perception knots)

add_executable(execute_knot_tie execute_knot_tie.cpp)
target_link_libraries(execute_knot_tie simulation  robots perception boost_python python2.6 knots)

add_executable(execute_knot_tie_telepathy execute_knot_tie_telepathy.cpp)
target_link_libraries(execute_knot_tie_telepathy simulation robots perception knots)


add_executable(record_demo_knot record_demo_knot.cpp)
target_link_libraries(record_demo_knot simulation robots python2.6 boost_python perception knots)

add_executable(lookup_and_execute lookup_and_execute.cpp)
target_link_libraries(lookup_and_execute simulation robots python2.6 boost_python perception knots)
