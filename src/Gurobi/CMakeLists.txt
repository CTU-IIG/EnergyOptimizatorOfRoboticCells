find_package (GUROBI REQUIRED)
include_directories (${GUROBI_INCLUDE_DIRS})
add_definitions (${GUROBI_COMPILER_FLAGS})

file (GLOB SOURCES "*.cpp")
add_library (GurobiSolver ${LIBRARY_LINKAGE} ${SOURCES})
target_link_libraries (GurobiSolver ${GUROBI_LIBRARIES})
install (TARGETS GurobiSolver DESTINATION lib64)
