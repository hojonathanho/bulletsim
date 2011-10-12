include_directories(${EIGEN2_INCLUDE_DIR})
add_library(haptics UDPSocket.cpp thread_socket_interface.cpp)
