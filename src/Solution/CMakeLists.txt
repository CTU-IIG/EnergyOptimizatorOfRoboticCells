file (GLOB SOURCES "*.cpp")
add_library (Solution ${LIBRARY_LINKAGE} ${SOURCES})
install (TARGETS Solution DESTINATION lib64)
