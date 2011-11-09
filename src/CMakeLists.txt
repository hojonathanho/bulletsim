include_directories(${BULLET_DIR}/src
                    ${BULLET_DIR}/Demos/OpenGL # for debug drawing
#                    ${BULLET_COLLADA_DIR}/BulletColladaConverter
                    ${OSGBULLET_DIR}
                    ${OSGWORKS_DIR}
                    ${OpenRAVE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN2_INCLUDE_DIR}
                    ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/haptics)
set(BULLET_LIBS OpenGLSupport BulletSoftBody BulletDynamics BulletCollision LinearMath)
#set(BULLET_COLLADA_LIBS BulletColladaConverter ColladaDom XML)

IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE) 

#add_executable(bulletscene bulletscene.cpp openravesupport.cpp environment.cpp)
#set_target_properties(bulletscene PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
#set_target_properties(bulletscene PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
#target_link_libraries(bulletscene
#    haptics
#    ${BULLET_COLLADA_LIBS}
#    ${BULLET_LIBS}
#
#    ${OpenRAVE_LIBRARIES}
#    ${OpenRAVE_CORE_LIBRARIES}
#
#    ${Boost_SERIALIZATION_LIBRARY}
#
#    osgBullet osgWorks
#    ${OSG_LIBS}
#
#    ${GLUT_glut_LIBRARY}
#    ${OPENGL_gl_LIBRARY}
#    ${OPENGL_glu_LIBRARY})

add_executable(newscene newscene.cpp environment.cpp basicobjects.cpp openravesupport.cpp util.cpp)
target_link_libraries(newscene haptics osgBullet osgWorks ${OSG_LIBS} ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
