cmake_minimum_required(VERSION 3.26 FATAL_ERROR)
project(percolation)
add_executable(percolation main.cpp)
set_property(TARGET percolation PROPERTY CXX_STANDARD 17)
find_package(VTK 9.2.6 EXACT REQUIRED)
target_link_libraries(percolation ${VTK_LIBRARIES})
vtk_module_autoinit(
	TARGETS percolation
	MODULES ${VTK_LIBRARIES})
