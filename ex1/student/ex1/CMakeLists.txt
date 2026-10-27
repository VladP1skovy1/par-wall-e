#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_minimum_required(VERSION 3.10)
project(ParWallE)
# message("CMAKE_BINARY_DIR: ${IGNITION-TRANSPORT_HEADERS}")
# message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

find_package(ignition-transport4 QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/../../build/interface)
include_directories(${CMAKE_SOURCE_DIR}/../../interface/include)
link_directories(${CMAKE_SOURCE_DIR}/../../build/interface)

add_executable(ex1
    main.cc
)

target_link_libraries(ex1
  ${IGNITION-TRANSPORT_LIBRARIES}
  robot_client
)


