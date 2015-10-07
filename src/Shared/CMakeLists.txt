file (GLOB SOURCES "*.cpp")
add_library (Shared ${LIBRARY_LINKAGE} ${SOURCES})
install (TARGETS Shared DESTINATION lib64)
