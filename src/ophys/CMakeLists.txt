include_directories(
  ${BULLET_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR} 
  ${BULLETSIM_SOURCE_DIR}/src
  ${OpenRAVE_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GUROBI_INCLUDE_DIR}
)

set(CMAKE_CXX_FLAGS "-Wall -Wno-sign-compare -Wno-reorder")
link_directories(${GUROBI_LIBRARY_DIR})

add_library(ophys
  particlesystem2.cpp
  ophys_config.cpp
)
target_link_libraries(ophys sqp2 simulation)

#add_executable(test_particlesystem test_particlesystem.cpp)
#target_link_libraries(test_particlesystem ophys)

#add_executable(test_ropesystem test_ropesystem.cpp)
#target_link_libraries(test_ropesystem ophys)

add_executable(test_particlesystem2 test_particlesystem2.cpp)
target_link_libraries(test_particlesystem2 ophys)

#add_executable(test_ropesystem2 test_ropesystem2.cpp)
#target_link_libraries(test_ropesystem2 ophys)

add_executable(test_gurobi_constraints test_gurobi_constraints.cpp)
target_link_libraries(test_gurobi_constraints ophys)

add_executable(nlopt_rope nlopt_rope.cpp)
target_link_libraries(nlopt_rope ophys nlopt)