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

add_library(ophys particlesystem.cpp ophys_config.cpp)
target_link_libraries(ophys sqp2 simulation)

add_executable(test_particlesystem test_particlesystem.cpp)
target_link_libraries(test_particlesystem ophys)
