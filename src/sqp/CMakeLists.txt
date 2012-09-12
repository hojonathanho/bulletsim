include_directories(
    ${BULLET_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR} 
    ${BULLETSIM_SOURCE_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

include_directories(/home/joschu/Src/gurobi501/linux64/include)
link_directories(/home/joschu/Src/gurobi501/linux64/lib)

add_executable(test_grb_eigen test_grb_eigen.cpp)
target_link_libraries(test_grb_eigen gurobi_c++ gurobi50)

add_library(sqp traj_costs.cpp utils_sqp.cpp sqp_algorithm.cpp collision_boxes.cpp collision_map_tools.cpp)

target_link_libraries(sqp utils simulation robots ${Boost_LIBRARIES} ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} gurobi_c++ gurobi50)

add_executable(test_traj_eval_grb test_traj_eval_grb2.cpp)
target_link_libraries(test_traj_eval_grb sqp)


include_directories(${PCL_INCLUDE_DIRS})
include_directories("/opt/ros/fuerte/stacks/geometry/tf/include"
					"/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include"
					"/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include")

link_directories("/opt/ros/fuerte/stacks/geometry/tf/lib")

add_executable(planning_server planning_server.cpp)
target_link_libraries(planning_server ${ROS_LIBRARIES} sqp clouds tf)

add_executable(interact_with_point_cloud interact_with_point_cloud.cpp)
target_link_libraries(interact_with_point_cloud clouds sqp)

add_executable(test_ros_ser test_ros_ser.cpp)
target_link_libraries(test_ros_ser ${ROS_LIBRARIES})