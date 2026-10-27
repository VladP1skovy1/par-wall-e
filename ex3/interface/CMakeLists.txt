add_subdirectory(msgs)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(include)


#find_package(ignition-transport8 QUIET REQUIRED)
find_package(ignition-transport4 QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})


add_library(robot_client SHARED 
  src/robot/common.cc
  src/robot/laser_sensor.cc
 	src/robot/robot_interface.cc

 	src/robot_client/robot_client.cc
 	src/robot_client/robot_client_laser.cc
 	src/robot_messages/robot_messages.cc
)

target_link_libraries(robot_client 
	${IGNITION-TRANSPORT_LIBRARIES}
	robot_messages
)
# add_dependencies(robot_client protobuf_compilation)



