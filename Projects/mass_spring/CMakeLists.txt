# Find eigen in system directory and treat it as an interface library
find_package(Eigen3 REQUIRED)
add_library(eigen INTERFACE IMPORTED)

# Generate the local Makefile
if(CREATE_LOCAL_MAKEFILES)
  file(RELATIVE_PATH CISPBA_RELATIVE_PATH ${CISPBA_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(${CISPBA_SOURCE_DIR}/Scripts/Makefile.in ${CMAKE_CURRENT_SOURCE_DIR}/Makefile @ONLY)
endif()

set(headers
	main.h
	MassSpringSystem.h
	SimulationDriver.h
	utils.h
)

set(sources
	main.cpp
	utils.cpp
)

list(SORT headers)
list(SORT sources)

source_group(Headers FILES ${headers})
source_group(Sources FILES ${sources})

# Configure this project
add_cispba_executable(mass_spring ${headers} ${sources})
target_include_directories(mass_spring SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(mass_spring eigen)
