include_directories(
    ${BULLET_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR} 
    ${BULLETSIM_SOURCE_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

include_directories(/home/joschu/Src/gurobi501/linux64/include)
link_directories(/home/joschu/Src/gurobi501/linux64/lib)

add_library(scp kinematics_utils.cpp traj_costs.cpp utils_scp.cpp scp_algorithm.cpp)

target_link_libraries(scp utils simulation robots ${Boost_LIBRARIES} ${BULLET_LIBS} ${OpenRAVE_LIBRARIES})

add_executable(test_traj_eval_grb test_traj_eval_grb.cpp)
target_link_libraries(test_traj_eval_grb scp gurobi_c++ gurobi50)