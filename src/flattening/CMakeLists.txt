include_directories(
    ${BULLET_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${BULLETSIM_SOURCE_DIR}/src
    ${OSGBULLET_DIR}
    ${WM_INCLUDE_DIR}
)
link_directories(${BULLETSIM_BINARY_DIR}/src ${WM_LIBRARY_DIR})

add_executable(demo_flatten demo_flatten.cpp nodeactions.cpp search.cpp clothmanip.cpp)
SET_SOURCE_FILES_PROPERTIES(search.cpp PROPERTIES COMPILE_FLAGS -fopenmp)
target_link_libraries(demo_flatten simulation ${WM_LIBRARIES} gomp)
