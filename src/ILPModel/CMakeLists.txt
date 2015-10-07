file (GLOB SOURCES "*.cpp")
add_library (ILPModel ${LIBRARY_LINKAGE} ${SOURCES})
install (TARGETS ILPModel DESTINATION lib64)
