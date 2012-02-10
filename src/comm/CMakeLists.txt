include_directories(
    ${JSON_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${BULLETSIM_SOURCE_DIR}/src
)

add_executable(test_comm2 test_comm2.cpp)
target_link_libraries(test_comm2 comm2 boost_system-mt json)

add_library(comm2 SHARED comm2.cpp comm_eigen.cpp)
target_link_libraries(comm2 ${Boost_LIBRARIES} json)

INSTALL(TARGETS comm2 DESTINATION "~/usr/lib")
SET(CMAKE_INSTALL_RPATH "~/usr/lib")
