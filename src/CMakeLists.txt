include_directories(${BULLET_DIR}/src
                    ${OpenRAVE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN2_INCLUDE_DIR}
                    ${OSGBULLET_DIR}
                    ${OSGWORKS_DIR}
                    ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/haptics)
set(BULLET_LIBS BulletSoftBody BulletDynamics BulletCollision LinearMath)

IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE) 

add_executable(newscene newscene.cpp environment.cpp basicobjects.cpp openravesupport.cpp util.cpp)
target_link_libraries(newscene haptics osgBullet osgWorks ${OSG_LIBS} ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
