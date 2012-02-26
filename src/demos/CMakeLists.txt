include_directories(
    ${BULLET_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${BULLETSIM_SOURCE_DIR}/src
    ${OSGBULLET_DIR}
    ${WM_INCLUDE_DIR}
)
link_directories(${BULLETSIM_BINARY_DIR}/src ${WM_LIBRARY_DIR})

add_executable(cloth_actions cloth_actions.cpp)
target_link_libraries(cloth_actions simulation ${WM_LIBRARIES})
