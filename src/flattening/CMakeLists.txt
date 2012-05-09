include_directories(
    ${BULLET_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${BULLETSIM_SOURCE_DIR}/src
    ${OSGBULLET_DIR}
    ${WM_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)
link_directories(
    ${BULLETSIM_BINARY_DIR}/src
    ${PCL_LIBRARY_DIRS}
    ${WM_LIBRARY_DIR}
)
add_definitions(${PCL_DEFINITIONS})

add_library(clothmanip
    nodeactions.cpp
    search.cpp
    search_general.cpp
    clothutil.cpp
    cloth.cpp
    make_bodies.cpp
    edges.cpp
    facepicker.cpp
    graspingactions.cpp
    genmanip.cpp
    config_flattening.cpp
    folding.cpp
    clothscene.cpp
    storage.cpp
    cloudutils.cpp
)
set_source_files_properties(search.cpp PROPERTIES COMPILE_FLAGS -fopenmp)
set_source_files_properties(search_general.cpp PROPERTIES COMPILE_FLAGS -fopenmp)
target_link_libraries(clothmanip
    simulation
    perception
    comm
    ${WM_LIBRARIES}
    ${PCL_LIBRARIES}
    gomp
)

add_executable(gen_rand_states gen_rand_states.cpp)
target_link_libraries(gen_rand_states clothmanip)

add_executable(clothviewer clothviewer.cpp)
target_link_libraries(clothviewer clothmanip)

add_executable(check_cloth_integrity check_cloth_integrity.cpp)
target_link_libraries(check_cloth_integrity clothmanip)

add_executable(gencloud gencloud.cpp ../perception/fake_kinect.cpp)
target_link_libraries(gencloud clothmanip GLU)

file(COPY batch_genclouds.py
    FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
    DESTINATION ${EXECUTABLE_OUTPUT_PATH})

add_executable(match_obs match_obs.cpp)
set_source_files_properties(match_obs.cpp PROPERTIES COMPILE_FLAGS -fopenmp)
target_link_libraries(match_obs clothmanip)

add_executable(demo_flatten demo_flatten.cpp)
target_link_libraries(demo_flatten clothmanip)

add_executable(demo_pr2_manip demo_pr2_manip.cpp)
target_link_libraries(demo_pr2_manip clothmanip robots)

add_executable(test_folding test_folding.cpp)
target_link_libraries(test_folding clothmanip)

add_executable(test_icp test_icp.cpp)
target_link_libraries(test_icp clothmanip)
