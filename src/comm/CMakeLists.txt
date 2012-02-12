include_directories(
    ${JSON_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${BULLETSIM_SOURCE_DIR}/src
)

add_executable(test_comm test_comm.cpp)
target_link_libraries(test_comm comm boost_system-mt json)

add_library(comm SHARED comm.cpp comm_eigen.cpp)
target_link_libraries(comm ${Boost_LIBRARIES} json)

INSTALL(TARGETS comm DESTINATION "~/usr/lib")
SET(CMAKE_INSTALL_RPATH "~/usr/lib")
