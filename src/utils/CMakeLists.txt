include_directories( ${PCL_INCLUDE_DIRS} )

add_library(utils config.cpp vector_io.cpp utils_vector.cpp testing.cpp yes_or_no.cpp interpolation.cpp logging.cpp conversions.cpp cvmat.cpp clock.cpp file.cpp)
target_link_libraries(utils ${LOG4CPLUS_LIBRARY} ${BOOST_LIBRARIES} ${BULLET_LIBS} ${OpenCV_LIBS})

add_executable(video_merger video_merger.cpp)
target_link_libraries(video_merger utils ${OpenCV_LIBS} boost_signals)