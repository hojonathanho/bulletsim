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

find_package(PythonLibs)

add_library(knots knots.cpp utils_python.cpp arm_base_traj.cpp trajectory_library.cpp rope_scenes.cpp)
target_link_libraries(knots simulation robots boost_python ${PYTHON_LIBRARIES})

add_executable(record_demo_knot record_demo_knot.cpp)
target_link_libraries(record_demo_knot simulation robots boost_python  knots ${PYTHON_LIBRARIES})

add_executable(lookup_and_execute lookup_and_execute.cpp)
target_link_libraries(lookup_and_execute simulation robots boost_python knots ${PYTHON_LIBRARIES})


