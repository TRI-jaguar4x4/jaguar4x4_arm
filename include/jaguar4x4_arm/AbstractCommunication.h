#include <string>

class AbstractCommunication {
  // TODO: how does TRI feel about exceptions?
  // Are they allowed... if not, return code instead of void.
  virtual void connect(std::string& ip, uint16_t port) = 0;
  virtual void sendCommand() = 0;
  virtual std::string recvMessage(std::string& boundary) = 0;
}
