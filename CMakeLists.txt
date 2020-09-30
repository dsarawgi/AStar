cmake_minimum_required(VERSION 3.17)
project(ASTAR)

set(CMAKE_CXX_STANDARD 14)

add_executable(ASTAR main.cpp AStar.cpp AStar.h)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
target_link_libraries(ASTAR ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
