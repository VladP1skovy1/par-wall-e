cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(explorer)
set(CMAKE_BUILD_TYPE RELWITHDEBINFO)
add_compile_options(-std=c++11)


find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-3.0) 
link_directories(${GTKMM_LIBRARY_DIRS})
include_directories(${GTKMM_INCLUDE_DIRS})


#find_package(ignition-transport8 QUIET REQUIRED)
find_package(ignition-transport4 QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/../../build/interface)
include_directories(${CMAKE_SOURCE_DIR}/../../interface/include)
link_directories(${CMAKE_SOURCE_DIR}/../../build/interface)
include_directories(src)

add_executable(ex3
    main.cc
    src/snd.cc
    src/gap_and_valley.cc
    src/exploration.cc
    src/control.cc
    src/canvas.cc
    src/map_grid.cc
    src/planner.cc
    src/point.cc
    src/window.cc
)

target_link_libraries(ex3
  ${IGNITION-TRANSPORT_LIBRARIES}
  robot_client
  ${GTKMM_LIBRARIES}  
  pthread
)


# aux_source_directory(src SRC)
# include_directories(src)
# add_executable(planner ${SRC})


