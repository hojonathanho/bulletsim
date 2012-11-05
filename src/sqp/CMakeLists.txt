include_directories(
    ${BULLET_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR} 
    ${BULLETSIM_SOURCE_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GUROBI_INCLUDE_DIR}
)

SET(CMAKE_CXX_FLAGS "-Wall -Wno-sign-compare -Wno-reorder")

link_directories(${GUROBI_LIBRARY_DIR})

#add_executable(test_grb_eigen test_grb_eigen.cpp)
#target_link_libraries(test_grb_eigen gurobi_c++ gurobi50)

add_library(sqp collisions.cpp utils_sqp.cpp config_sqp.cpp sqp_algorithm.cpp collision_boxes.cpp 
collision_map_tools.cpp planning_problems.cpp plotters.cpp state_setter.cpp kinematics_utils.cpp
functions.cpp)

target_link_libraries(sqp utils simulation robots ${Boost_LIBRARIES} ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} gurobi_c++ gurobi50 json)


add_executable(test_arm_table test_arm_table.cpp)
target_link_libraries(test_arm_table sqp)

add_executable(test_cart_with_base test_cart_with_base.cpp)
target_link_libraries(test_cart_with_base sqp)


include_directories(${PCL_INCLUDE_DIRS})
include_directories("/opt/ros/fuerte/stacks/geometry/tf/include"
					"/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include"
					"/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include")

include_directories("${BULLETSIM_SOURCE_DIR}/bulletsim_msgs/build/gen/cpp")

link_directories("/opt/ros/fuerte/stacks/geometry/tf/lib")

#add_executable(planning_server planning_server.cpp)
#target_link_libraries(planning_server ${ROS_LIBRARIES} sqp clouds tf)

add_executable(interact_with_point_cloud interact_with_point_cloud.cpp)
target_link_libraries(interact_with_point_cloud clouds sqp)

add_executable(test_ros_ser test_ros_ser.cpp)
target_link_libraries(test_ros_ser ${ROS_LIBRARIES})

add_executable(test_rave_env test_rave_env.cpp)
target_link_libraries(test_rave_env simulation robots sqp json)

find_package(PythonLibs REQUIRED)
add_executable(test_box_picking test_box_picking.cpp)
target_link_libraries(test_box_picking simulation robots sqp ${PYTHON_LIBRARIES} boost_python)

#########################################

add_library(sqp2 expr_ops.cpp sqp_dynamics.cpp sqp.cpp config_sqp.cpp traj_sqp.cpp collisions.cpp kinematics_utils.cpp plotters.cpp utils_sqp.cpp functions.cpp)
target_link_libraries(sqp2 utils gurobi_c++ gurobi50)

add_executable(test_new_arm_table test_new_arm_table.cpp)
target_link_libraries(test_new_arm_table sqp2 robots simulation)

add_executable(test_scalar_opt test_scalar_opt.cpp)
target_link_libraries(test_scalar_opt sqp2)

add_executable(test_vector_opt test_vector_opt.cpp)
target_link_libraries(test_vector_opt sqp2)

add_executable(test_dynamics_solver test_dynamics_solver.cpp sqp_dynamics.cpp state_setter.cpp kinematics_utils.cpp utils_sqp.cpp)
target_link_libraries(test_dynamics_solver sqp2 simulation)

add_executable(test_rodrigues_jac test_rodrigues_jac.cpp)
target_link_libraries(test_rodrigues_jac sqp2)
