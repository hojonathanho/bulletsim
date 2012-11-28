include_directories(
    ${BULLET_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR} 
    ${BULLETSIM_SOURCE_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${BULLETSIM_MSGS_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-Wall -Wno-sign-compare -Wno-reorder")

set(GUROBI_INCLUDE_DIR "${GUROBI_HOME}/include")
set(GUROBI_LIBRARY_DIR "${GUROBI_HOME}/lib")
include_directories(${GUROBI_INCLUDE_DIR})
link_directories(${GUROBI_LIBRARY_DIR})


include_directories(${PCL_INCLUDE_DIRS})


#########################################

add_executable(test_grb_eigen1 test_grb_eigen.cpp)
target_link_libraries(test_grb_eigen1 gurobi_c++ gurobi50 utils)

add_library(sqp2 expr_ops.cpp sqp.cpp config_sqp.cpp traj_sqp.cpp 
collisions.cpp kinematics_utils.cpp plotters.cpp utils_sqp.cpp functions.cpp
traj_safety_checks.cpp planning_problems2.cpp state_setter.cpp)
target_link_libraries(sqp2 utils gurobi_c++ gurobi50 simulation)

add_executable(test_new_arm_table test_new_arm_table.cpp)
target_link_libraries(test_new_arm_table sqp2 robots simulation)

add_executable(test_rave_env test_rave_env.cpp)
target_link_libraries(test_rave_env simulation robots sqp2 json)

add_executable(test_scalar_opt test_scalar_opt.cpp)
target_link_libraries(test_scalar_opt sqp2)

add_executable(test_vector_opt test_vector_opt.cpp)
target_link_libraries(test_vector_opt sqp2)

add_executable(test_dynamics_solver test_dynamics_solver2.cpp rigid_body_dynamics.cpp)
target_link_libraries(test_dynamics_solver sqp2 simulation)

add_executable(test_rodrigues_jac test_rodrigues_jac.cpp)
target_link_libraries(test_rodrigues_jac sqp2)

find_package(PythonLibs REQUIRED)
add_executable(test_box_picking test_box_picking2.cpp)
target_link_libraries(test_box_picking simulation robots sqp2 ${PYTHON_LIBRARIES} boost_python)

add_executable(test_cart_with_base test_cart_with_base.cpp)
target_link_libraries(test_cart_with_base sqp2 robots)


#################

add_library(sqp_ros sqp_ros.cpp collision_map_tools collision_boxes.cpp)
target_link_libraries(sqp_ros sqp2 simulation robots clouds ${ROS_LIBRARIES})

add_executable(planning_server planning_server.cpp)
target_link_libraries(planning_server sqp_ros)

add_executable(interact_with_point_cloud interact_with_point_cloud.cpp)
target_link_libraries(interact_with_point_cloud clouds sqp_ros)

add_executable(test_ros_ser test_ros_ser.cpp)
target_link_libraries(test_ros_ser ${ROS_LIBRARIES})


#############

add_library(drc drc_costs.cpp drc_info.cpp drc_dynamics.cpp)
target_link_libraries(drc sqp2)

add_executable(test_drc test_drc.cpp)
target_link_libraries(test_drc simulation sqp2 drc)