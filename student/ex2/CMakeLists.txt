cmake_minimum_required(VERSION 2.8)
project(planner)
set(CMAKE_BUILD_TYPE RELWITHDEBINFO)

# add_compile_options(-std=c++0x)
add_compile_options(-std=c++11)

find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-3.0) 

link_directories(
    ${GTKMM_LIBRARY_DIRS}  
)

include_directories(
    ${GTKMM_INCLUDE_DIRS}  
)


aux_source_directory(src SRC)
include_directories(src)

add_executable(planner ${SRC})

target_link_libraries(planner 
    ${GTKMM_LIBRARIES}  
    pthread
)

