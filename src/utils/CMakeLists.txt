add_library(utils config.cpp vector_io.cpp yes_or_no.cpp interpolation.cpp logging.cpp)
target_link_libraries(utils ${LOG4CPLUS_LIBRARY})
