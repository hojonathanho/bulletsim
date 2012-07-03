include_directories( ${PCL_INCLUDE_DIRS} )

add_library(utils config.cpp vector_io.cpp yes_or_no.cpp interpolation.cpp logging.cpp conversions.cpp cvmat.cpp clock.cpp)
target_link_libraries(utils ${LOG4CPLUS_LIBRARY} ${BOOST_LIBRARIES} ${BULLET_LIBS} ${OpenCV_LIBS})
