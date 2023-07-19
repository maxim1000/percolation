cmake_minimum_required(VERSION 3.26 FATAL_ERROR)
project(percolation)
add_executable(percolation main.cpp)
set_property(TARGET percolation PROPERTY CXX_STANDARD 17)
