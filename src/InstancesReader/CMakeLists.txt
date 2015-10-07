find_package (LibXML++ REQUIRED)
include_directories (${LibXML++_INCLUDE_DIRS})

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	set_source_files_properties (XmlReader.cpp PROPERTIES COMPILE_FLAGS -Wno-deprecated-register)
	set_source_files_properties (InstancesReader.cpp PROPERTIES COMPILE_FLAGS -Wno-deprecated-register)
endif ()

file (GLOB SOURCES "*.cpp")
add_library (InstancesReader ${LIBRARY_LINKAGE} ${SOURCES})
target_link_libraries (InstancesReader ${LibXML++_LIBRARIES})
install (TARGETS InstancesReader DESTINATION lib64)

