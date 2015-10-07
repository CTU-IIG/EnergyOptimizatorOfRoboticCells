find_package (LpSolve REQUIRED)

file (GLOB SOURCES "*.cpp")
include_directories (${LPSOLVE_INCLUDE_DIR})
add_library (LPSolveSolver ${LIBRARY_LINKAGE} ${SOURCES})
target_link_libraries (LPSolveSolver ${LPSOLVE_LIBRARIES})
install (TARGETS LPSolveSolver DESTINATION lib64)
