find_package(PythonLibs REQUIRED)


include_directories(
  ${PYTHON_INCLUDE_DIRS}
)

add_library(lfd LfdRpmWrapper.cpp)
target_link_libraries(lfd boost_python ${PYTHON_LIBRARIES})