include_directories(${EIGEN3_INCLUDE_DIR}
                    ${FRAMEWORK_HEADERS}
                    ${OSG_INCLUDE_DIRS}
                    ${GUROBI_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIR}
                    ${BULLET_INCLUDE_DIR}
                    ${BULLETSIM_SOURCE_DIR}/src)

#add_executable(osg_demo osg_demo.cpp)
#target_link_libraries(osg_demo scp_robots ${OSG_LIBS} ${GUROBI_LIBRARIES})

#add_executable(car_demo car_demo.cpp)
#target_link_libraries(car_demo scp_robots opt ${OSG_LIBS} ${GUROBI_LIBRARIES})

#add_executable(car_ekf car_ekf.cpp)
#target_link_libraries(car_ekf scp_robots opt ${OSG_LIBS} ${GUROBI_LIBRARIES})

add_executable(car_bsp car_bsp.cpp)
target_link_libraries(car_bsp scp_robots opt ${OSG_LIBS} ${GUROBI_LIBRARIES} ${OPENGL_gl_LIBRARY})

#add_executable(car_localizer car_localizer.cpp)
#target_link_libraries(car_localizer scp_robots opt ${OSG_LIBS} ${GUROBI_LIBRARIES})

add_executable(car_camera_test car_camera_test.cpp)
target_link_libraries(car_camera_test scp_robots opt ${OSG_LIBS} ${GUROBI_LIBRARIES} ${OPENGL_gl_LIBRARY})

#add_executable(car_bsp_ilqr car_bsp_ilqr.cpp)
#target_link_libraries(car_bsp_ilqr scp_robots opt ${OSG_LIBS} ${GUROBI_LIBRARIES})

#add_executable(gurobi_demo gurobi_demo.cpp)
#target_link_libraries(gurobi_demo ${GUROBI_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS program_options system prg_exec_monitor filesystem signals)

#add_executable(pr2_demo pr2_demo.cpp)
#target_link_libraries(pr2_demo opt scp_robots simulation robots ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(pr2_ekf pr2_ekf.cpp)
#target_link_libraries(pr2_ekf opt scp_robots simulation robots ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(pr2_beacons pr2_beacons.cpp)
#target_link_libraries(pr2_beacons opt scp_robots simulation robots ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pr2_bsp_collision pr2_bsp_collision.cpp)
target_link_libraries(pr2_bsp_collision opt scp_robots simulation robots ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})


#add_executable(pr2_localizer pr2_localizer.cpp)
#target_link_libraries(pr2_localizer opt scp_robots simulation robots ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pr2_push_button pr2_push_button.cpp)
target_link_libraries(pr2_push_button opt scp_robots simulation robots ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pr2_ring_peg pr2_ring_peg.cpp)
target_link_libraries(pr2_ring_peg opt scp_robots simulation robots ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})
