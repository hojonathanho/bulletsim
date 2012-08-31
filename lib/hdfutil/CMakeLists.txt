include_directories(/opt/local/include)
add_library(hdfutil hdfutil.cpp)
target_link_libraries(hdfutil hdf5 hdf5_cpp)
