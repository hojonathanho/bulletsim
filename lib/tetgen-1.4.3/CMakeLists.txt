set_source_files_properties(predicates.cxx PROPERTIES COMPILE_FLAGS "-O0")
add_library(tetgen tetgen.cxx predicates.cxx)
set_target_properties(tetgen PROPERTIES COMPILE_DEFINITIONS "TETLIBRARY")
