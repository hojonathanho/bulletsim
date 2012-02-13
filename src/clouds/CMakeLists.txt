include_directories(
    ${PCL_INCLUDE_DIRS}
    ${WM_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${BULLETSIM_SOURCE_DIR}/src
    ${BULLET_DIR}/src
)
link_directories(
  ${PCL_LIBRARY_DIRS} 
  ${WM_LIBRARY_DIR}
)

add_definitions(${PCL_DEFINITIONS})

add_library(clouds geom.cpp get_table.cpp utils_pcl.cpp cloud_filtering.cpp comm_cv.cpp comm_pcl.cpp utils_cv.cpp)
target_link_libraries(clouds ${PCL_LIBRARIES} ${WM_LIBRARIES} ${OpenCV_LIBS})

#add_executable (write_pcds write_pcds.cpp)
#target_link_libraries (write_pcds ${PCL_LIBRARIES} boost_program_options comm)

# add_executable(test_get_table_full test_get_table_full.cpp)
# target_link_libraries(test_get_table_full  clouds ${PCL_LIBRARIES})

# add_executable(get_focal_length get_focal_length.cpp)
# target_link_libraries(get_focal_length ${PCL_LIBRARIES})

add_executable(record_one_pcd record_one_pcd.cpp)
target_link_libraries(record_one_pcd ${PCL_LIBRARIES} boost_program_options)

add_executable(pcd_picker pcd_picker.cpp)
target_link_libraries(pcd_picker ${PCL_LIBRARIES})

add_executable(comm_get_table comm_get_table.cpp)
target_link_libraries(comm_get_table ${PCL_LIBRARIES} clouds comm2 boost_program_options)

add_executable(comm_write_pcds comm_write_pcds.cpp)
target_link_libraries(comm_write_pcds clouds ${PCL_LIBRARIES} comm2 boost_program_options)


add_executable(test_cloud_filtering test_cloud_filtering.cpp)
target_link_libraries(test_cloud_filtering clouds ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(comm_downsample_clouds comm_downsample_clouds.cpp)
target_link_libraries(comm_downsample_clouds clouds ${PCL_LIBRARIES} boost_program_options comm2 ${OpenCV_LIBS})

add_executable(test_utils test_utils.cpp)
target_link_libraries(test_utils ${PCL_LIBRARIES} clouds)

add_executable(viz_curves viz_curves.cpp)
target_link_libraries(viz_curves ${PCL_LIBRARIES} clouds boost_program_options utils)

INSTALL(TARGETS record_one_pcd comm_get_table comm_write_pcds DESTINATION "~/usr/bin")
INSTALL(TARGETS clouds DESTINATION "~/usr/lib")
