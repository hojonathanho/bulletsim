#BOOST PYTHON STUFF
find_package(Boost COMPONENTS python)
include_directories (${Boost_INCLUDE_DIRS})
find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})

#add_library(utils_python utils_python.cpp )
#target_link_libraries(utils_python simulation boost_python ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})

add_library(lfd LfdRpmWrapper.cpp
				RavensLfd.cpp
				utils_python.cpp
				)

target_link_libraries(lfd simulation robots 
                          boost_python
                          ${PYTHON_LIBRARIES}
                          ${Boost_LIBRARIES}
                     )
                     

add_executable(test_savez test_savez.cpp)
target_link_libraries(test_savez lfd)
