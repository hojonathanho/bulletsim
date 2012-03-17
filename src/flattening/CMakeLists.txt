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
    clothutil.cpp
    make_bodies.cpp
    clothgrasping.cpp
    edges.cpp
)
set_source_files_properties(search.cpp PROPERTIES COMPILE_FLAGS -fopenmp)
target_link_libraries(clothmanip
    simulation
    ${WM_LIBRARIES}
    ${PCL_LIBRARIES}
    gomp
)

add_executable(demo_flatten demo_flatten.cpp)
target_link_libraries(demo_flatten clothmanip)

add_executable(demo_pr2_manip demo_pr2_manip.cpp)
target_link_libraries(demo_pr2_manip clothmanip robots)
