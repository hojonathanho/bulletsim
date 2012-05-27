add_library(comm comm.cpp comm_eigen.cpp)
target_link_libraries(comm ${Boost_LIBRARIES} json utils)

add_executable(test_comm test_comm.cpp)
target_link_libraries(test_comm comm ${Boost_LIBRARIES} json utils)

#INSTALL(TARGETS comm DESTINATION "~/usr/lib")
#SET(CMAKE_INSTALL_RPATH "~/usr/lib")
