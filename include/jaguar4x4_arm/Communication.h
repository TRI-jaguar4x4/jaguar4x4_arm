#pragma once

#include <string>

#include "AbstractCommunication.h"

class Communication : public AbstractCommunication {
 public:
  void connect(std::string& ip, uint16_t port) override;
  void sendCommand(std::string& cmd) override;
  std::string recvMessage(std::string& boundary) override;
 private:
  int ipValidator(std::string& ip);
  int fd_;
};
