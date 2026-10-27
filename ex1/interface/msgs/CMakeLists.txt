find_package(Protobuf REQUIRED)

PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER
  robot_messages.proto
)

add_library(robot_messages SHARED 
  ${PROTO_SRC}
)

# add_custom_target(protobuf_compilation DEPENDS ${PROTO_SRC} ${PROTO_HEADER})
