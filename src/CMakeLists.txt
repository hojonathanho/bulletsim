include_directories(${BULLET_DIR}/src
                    ${BULLET_DIR}/Demos/OpenGL # for debug drawing
                    ${EIGEN2_INCLUDE_DIR}
                    ${SURGICAL_PHYSICS_SOURCE_DIR}/lib/haptics)
set(BULLET_LIBS OpenGLSupport BulletDynamics BulletCollision LinearMath)

add_executable(bulletscene bulletscene.cpp)

target_link_libraries(bulletscene haptics ${BULLET_LIBS} ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
