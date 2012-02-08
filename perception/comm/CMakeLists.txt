cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

include_directories(${JSON_INCLUDE_DIR})
include_directories("../../utils")
#link_directories("~/usr/lib")


if (DEFINED SURGICAL_PHYSICS_SOURCE_DIR)
  message("this is a subdirectory")
else()
  set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
  set(JSON_DIR "../../lib/json")

  include_directories(
    ${JSON_DIR}/include
    ${EIGEN3_INCLUDE_DIR})
  link_directories(
    "~/usr/lib"
  )

endif()  



#if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#    include_directories("/Users/joschu/Src/jsoncpp/include")
#    include_directories("/usr/local/include")
#    include_directories("/usr/local/include/eigen3")
#    link_directories("/usr/local/lib")
#else()
#    include_directories("/home/joschu/Src/jsoncpp/include")
#    link_directories("~/usr/lib")
#    include_directories("/usr/include/eigen3")
#endif()


find_package(Boost COMPONENTS program_options filesystem REQUIRED)

#add_library(comm comm.cpp)
#target_link_libraries(comm boost_filesystem)

#add_executable(test_comm test_comm.cpp)
#target_link_libraries(test_comm comm)

add_executable(test_comm2 test_comm2.cpp)
target_link_libraries(test_comm2 comm2 boost_system-mt json)

add_library(comm2 SHARED comm2.cpp comm_eigen.cpp)
target_link_libraries(comm2 ${Boost_LIBRARIES} json)

INSTALL(TARGETS comm2 DESTINATION "~/usr/lib")
SET(CMAKE_INSTALL_RPATH "~/usr/lib")
