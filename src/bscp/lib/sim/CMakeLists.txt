include_directories(${EIGEN3_INCLUDE_DIR}
                    ${OSG_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${FRAMEWORK_HEADERS}) 

add_library(scp_robots robots.cpp sensors.cpp) 
target_link_libraries(scp_robots ${OSG_LIBS})

