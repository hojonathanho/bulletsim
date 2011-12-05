set(WM_ROOT "~/Src/GeometricTools/WildMagic5")
set(WM_INCLUDE_DIR "${WM_ROOT}/SDK/Include")
set(WM_LIBRARY_DIR "${WM_ROOT}/SDK/Library/Debug")
set(WM_LIBRARIES Wm5Core Wm5Imagics Wm5Mathematics Wm5Physics)
set(JSON_INCLUDE_DIR "${SURGICAL_PHYSICS_SOURCE_DIR}/lib/json")
set(JSON_LIBRARY_DIR "${SURGICAL_PHYSICS_SOURCE_DIR}/lib/json")

find_package(PCL 1.3)
include_directories(${BULLET_DIR}/src
                    ${OSGBULLET_DIR}
                    ${OSGWORKS_DIR}
                    ${OpenRAVE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
		    ${EIGEN3_INCLUDE_DIR}
		    ${WM_INCLUDE_DIR}
                    ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/haptics
		    ${PCL_INCLUDE_DIRS}
		    "../src"
)
set(BULLET_LIBS BulletSoftBody BulletDynamics BulletCollision LinearMath)
link_directories(
  ${PCL_LIBRARY_DIRS} 
  ${WM_LIBRARY_DIR}
${JSON_LIBRARY_DIR}
)
add_definitions(${PCL_DEFINITIONS})


add_executable (write_pcds write_pcds.cpp)
target_link_libraries (write_pcds ${PCL_LIBRARIES} boost_program_options)

add_executable(test_get_table_full test_get_table_full.cpp geom.cpp get_table.cpp)
target_link_libraries(test_get_table_full  ${WM_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_kinect_rope1 test_kinect_rope.cpp ../src/dist_math.cpp get_table.cpp geom.cpp)
target_link_libraries(test_kinect_rope1 simulation ${PCL_LIBRARIES} ${WM_LIBRARIES})

add_executable(test_eigen_inc test_eigen_inc.cpp)
add_executable(test_json test_json.cpp)
target_link_libraries(test_json json)