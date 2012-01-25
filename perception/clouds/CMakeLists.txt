cmake_minimum_required(VERSION 2.0)
find_package(PCL 1.5 REQUIRED)
find_package(OpenCV REQUIRED)

set(WM_ROOT "~/Src/GeometricTools/WildMagic5")
set(WM_INCLUDE_DIR "${WM_ROOT}/SDK/Include")
set(WM_LIBRARY_DIR "${WM_ROOT}/SDK/Library/Debug")
set(WM_LIBRARIES Wm5Core Wm5Imagics Wm5Mathematics Wm5Physics)
set(BULLET_INCLUDE_DIR "/usr/local/include/bullet")

include_directories(
  "~/cpp"
  ${PCL_INCLUDE_DIRS}
  ${WM_INCLUDE_DIR}
  ${BULLET_INCLUDE_DIR}
  "~/Src/jsoncpp/include"
)
link_directories(
  ${PCL_LIBRARY_DIRS} 
  ${WM_LIBRARY_DIR}
  "~/usr/lib"
)
add_definitions(${PCL_DEFINITIONS})
add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")


link_directories("/usr/local/lib")	


add_library(clouds geom.cpp get_table.cpp utils_pcl.cpp cloud_filtering.cpp comm_cv.cpp comm_pcl.cpp utils_cv.cpp correction.cpp)
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
target_link_libraries(comm_get_table ${PCL_LIBRARIES} clouds comm2)

add_executable(comm_write_pcds comm_write_pcds.cpp)
target_link_libraries(comm_write_pcds clouds ${PCL_LIBRARIES} comm2)

add_executable(test_cloud_filtering test_cloud_filtering.cpp)
target_link_libraries(test_cloud_filtering clouds ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(comm_downsample_clouds comm_downsample_clouds.cpp)
target_link_libraries(comm_downsample_clouds clouds ${PCL_LIBRARIES} boost_program_options comm2 ${OpenCV_LIBS})



add_executable(test_utils test_utils.cpp)
target_link_libraries(test_utils ${PCL_LIBRARIES} clouds)

add_executable(test_correction test_correction.cpp ~/cpp/vector_io.cpp)
target_link_libraries(test_correction ${PCL_LIBRARIES} clouds)


add_executable(viz_curves viz_curves.cpp ~/cpp/vector_io.cpp)
target_link_libraries(viz_curves ${PCL_LIBRARIES} clouds boost_program_options)

INSTALL(TARGETS record_one_pcd comm_get_table comm_write_pcds DESTINATION "~/usr/bin")
INSTALL(TARGETS clouds DESTINATION "~/usr/lib")