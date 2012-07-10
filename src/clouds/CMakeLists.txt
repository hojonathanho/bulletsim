# pcl_ros:
#include_directories("/opt/ros/fuerte/stacks/perception_pcl/pcl_ros/include")
#link_directories("/opt/ros/fuerte/stacks/perception_pcl/pcl_ros/lib")

include_directories("/opt/ros/fuerte/stacks/geometry/tf/include"
					"/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include")
link_directories("/opt/ros/fuerte/stacks/geometry/tf/lib")

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${BULLETSIM_SOURCE_DIR}/src
    ${BULLET_DIR}/src
)
link_directories(
  ${PCL_LIBRARY_DIRS} 
)

add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")

add_library(clouds geom.cpp  get_table2.cpp plane_finding.cpp utils_pcl.cpp cloud_ops.cpp utils_cv.cpp )
target_link_libraries(clouds utils ${PCL_LIBRARIES} ${WM_LIBRARIES} ${OpenCV_LIBS} ${BULLET_LIBS})

add_executable(preprocessor_node preprocessor_node.cpp)
target_link_libraries(preprocessor_node clouds comm utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${ROS_LIBRARIES} tf)

add_executable(viz_curves viz_curves.cpp)
target_link_libraries(viz_curves ${PCL_LIBRARIES} clouds boost_program_options utils)

add_executable(get_table_server get_table_server.cpp)
target_link_libraries(get_table_server clouds ${PCL_LIBRARIES} ${ROS_LIBRARIES} utils boost_program_options)
