set(WM_ROOT "~/Src/GeometricTools/WildMagic5")
set(WM_INCLUDE_DIR "${WM_ROOT}/SDK/Include")
set(WM_LIBRARY_DIR "${WM_ROOT}/SDK/Library/Debug")
set(WM_LIBRARIES Wm5Core Wm5Imagics Wm5Mathematics Wm5Physics)
set(JSON_INCLUDE_DIR "${SURGICAL_PHYSICS_SOURCE_DIR}/lib/json")
set(JSON_LIBRARY_DIR "${SURGICAL_PHYSICS_SOURCE_DIR}/lib/json")

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_GLIBCXX_DEBUG")



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
		    ${SURGICAL_PHYSICS_SOURCE_DIR}/src
		    ${JSON_INCLUDE_DIR}
)

set(BULLET_LIBS BulletSoftBody BulletDynamics BulletCollision LinearMath)
link_directories(
  ${PCL_LIBRARY_DIRS} 
  ${WM_LIBRARY_DIR}
  ${JSON_LIBRARY_DIR}
)
add_definitions(${PCL_DEFINITIONS})

include_directories("~/cpp")
link_directories("~/cpp/clouds/build" "~/cpp/comm/build")


add_library(perception tracking.cpp utils_perception.cpp dist_math.cpp)
target_link_libraries(perception ${PCL_LIBRARIES} simulation)

add_executable(test_kinect_rope1 test_kinect_rope.cpp)
target_link_libraries(test_kinect_rope1 simulation perception clouds ${PCL_LIBRARIES} json comm)



#add_executable(test_eigen_inc test_eigen_inc.cpp)

add_executable(test_json test_json.cpp)
target_link_libraries(test_json json)

add_executable(test_kinect_towel test_kinect_towel.cpp)
target_link_libraries(test_kinect_towel ${PCL_LIBRARIES} json simulation perception clouds comm)