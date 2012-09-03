include_directories(${EIGEN3_INCLUDE_DIR}
                    ${FRAMEWORK_HEADERS}
                    ${OSG_INCLUDE_DIRS}
                    ${GUROBI_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIR}
                    ${BULLET_INCLUDE_DIR}
                    ${BULLETSIM_SOURCE_DIR}/src)

add_executable(osg_demo osg_demo.cpp)
target_link_libraries(osg_demo scp_robots ${OSG_LIBS} ${GUROBI_LIBRARIES})

add_executable(car_demo car_demo.cpp)
target_link_libraries(car_demo scp_robots opt ${OSG_LIBS} ${GUROBI_LIBRARIES})

add_executable(car_ekf car_ekf.cpp)
target_link_libraries(car_ekf scp_robots opt ${OSG_LIBS} ${GUROBI_LIBRARIES})

add_executable(car_bsp car_bsp.cpp)
target_link_libraries(car_bsp scp_robots opt ${OSG_LIBS} ${GUROBI_LIBRARIES})

add_executable(gurobi_demo gurobi_demo.cpp)
target_link_libraries(gurobi_demo ${GUROBI_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS program_options system exception prg_exec_monitor filesystem signals)

add_executable(pr2_demo pr2_demo.cpp)
target_link_libraries(pr2_demo opt scp_robots simulation robots ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pr2_ekf pr2_ekf.cpp)
target_link_libraries(pr2_ekf opt scp_robots simulation robots ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pr2_beacons pr2_beacons.cpp)
target_link_libraries(pr2_beacons opt scp_robots simulation robots ${BULLET_LIBS} ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})
