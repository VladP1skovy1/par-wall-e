#include <google/protobuf/message.h>
#include <iostream>
#include <string>
#include <ignition/transport.hh>
void cb(const google::protobuf::Message &_msg,
        const ignition::transport::MessageInfo &_info)
{
  std::cout << "XXXXXXXXXXXXXX   Topic: [" << _info.Topic() << "]" << std::endl;
  std::cout << _msg.DebugString() << std::endl;
}
int main(int argc, char **argv)
{
  ignition::transport::Node node;
  std::string topic = "/foo";
  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
    return -1;
  }
  std::cout << "Zzzzzz." << std::endl;
  ignition::transport::waitForShutdown();
  return 0;
}