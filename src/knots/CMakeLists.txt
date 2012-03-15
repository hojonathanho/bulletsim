include_directories(
  ${WM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GUROBI_INCLUDE_DIR}
  ${TETGEN_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS} 
  ${WM_LIBRARY_DIR}
  ${GUROBI_LIBRARY_DIR}
  ${TETGEN_DIR}
)

add_executable(record_knot_tie record_knot_tie.cpp)
target_link_libraries(record_knot_tie simulation comm clouds robots perception)
add_executable(execute_knot_tie execute_knot_tie.cpp)
target_link_libraries(execute_knot_tie simulation comm clouds robots perception)
