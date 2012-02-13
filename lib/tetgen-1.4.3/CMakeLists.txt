set_source_files_properties(predicates.cxx PROPERTIES COMPILE_FLAGS "-O0")
set_source_files_properties(tetgen.cxx PROPERTIES COMPILE_FLAGS "-O2")
add_library(tetgen STATIC tetgen.cxx predicates.cxx)
