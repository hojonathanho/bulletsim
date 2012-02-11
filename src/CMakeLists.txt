set_property(
  DIRECTORY
  PROPERTY COMPILE_DEFINITIONS BULLETSIM_DATA_DIR=${BULLETSIM_SOURCE_DIR}/data)

add_subdirectory(simulation)
add_subdirectory(robots)
add_subdirectory(utils)
add_subdirectory(comm)
add_subdirectory(clouds)
if(BUILD_PERCEPTION)
    add_subdirectory(perception)
endif(BUILD_PERCEPTION)

add_subdirectory(tests)
