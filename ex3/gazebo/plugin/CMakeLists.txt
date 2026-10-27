find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(${CMAKE_BINARY_DIR}/interface)
# include_directories(include)

add_library(turtlebot_plugin SHARED 
    plugin.cc
)

target_link_libraries(turtlebot_plugin 
    robot_messages
    ${GAZEBO_LIBRARIES}
)



add_library(laser_plugin SHARED 
    laserplugin.cc
)

target_link_libraries(laser_plugin 
    robot_messages
    ${GAZEBO_LIBRARIES}
)



# add_executable(app
#     main.cc
#     ${PROTO_SRC}
# )


# target_link_libraries(app
#   ${IGNITION-TRANSPORT_LIBRARIES}
#   proto
# )

# add_executable(sub
#     subscriber.cc
# )

# target_link_libraries(sub
#   ${IGNITION-TRANSPORT_LIBRARIES}
# )
