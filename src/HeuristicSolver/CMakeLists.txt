if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	set_source_files_properties (ParallelSolver.cpp PROPERTIES COMPILE_FLAGS -Wno-unknown-pragmas)
endif ()

file (GLOB SOURCES "*.cpp")
add_library (HeuristicSolver ${LIBRARY_LINKAGE} ${SOURCES})
target_link_libraries (HeuristicSolver ILPSolver)
install (TARGETS HeuristicSolver DESTINATION lib64)
