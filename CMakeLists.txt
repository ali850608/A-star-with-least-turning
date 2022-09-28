project(ASTAR_least_turning)

cmake_minimum_required (VERSION 2.6)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g3 -Wall --std=c++0x")
SET(CMAKE_BUILD_TYPE "DEBUG")

include_directories("include")
file(GLOB SOURCES "src/*.cpp")
add_executable(ASTAR_least_turning ${SOURCES})

find_package(Boost REQUIRED COMPONENTS graph program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(ASTAR_least_turning ${Boost_LIBRARIES})

find_library(DW_LIB dw REQUIRED)
target_compile_definitions(ASTAR_least_turning PRIVATE "BACKWARD_HAS_DW=1")
target_link_libraries(ASTAR_least_turning PUBLIC "${DW_LIB}")
