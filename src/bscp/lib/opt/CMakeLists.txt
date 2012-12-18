include_directories(${EIGEN3_INCLUDE_DIR}
                    ${GUROBI_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIRS}
                    ${FRAMEWORK_HEADERS}
                    ${OSG_INCLUDE_DIRS}
                    )
add_library(opt_gurobi gurobi_solver.cpp gurobi_optimize.cpp)
target_link_libraries(opt_gurobi ${GUROBI_LIBRARIES})
add_library(opt optimize.cpp scp_solver.cpp ilqr_solver.cpp)
target_link_libraries(opt ${GUROBI_LIBRARIES} opt_gurobi)
