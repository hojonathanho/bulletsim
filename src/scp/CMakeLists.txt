include_directories(
    ${BULLET_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR} 
    ${BULLETSIM_SOURCE_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)



add_library(scp scp.cpp)

target_link_libraries(scp utils simulation robots ${Boost_LIBRARIES} ${BULLET_LIBS} ${OpenRAVE_LIBRARIES})


add_executable(test_scp test_scp.cpp)
target_link_libraries(test_scp scp)
