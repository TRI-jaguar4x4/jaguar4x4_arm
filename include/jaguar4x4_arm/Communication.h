#pragma once

#include <string>

#include "AbstractCommunication.h"

class Communication : public AbstractCommunication {
 public:
  void connect(const std::string& ip, uint16_t port) override;
  void sendCommand(const std::string& cmd) override;
  std::string recvMessage(const std::string& boundary, int timeout_msec)
    override;

 private:
  int ipValidator(const std::string& ip);
  int fd_;
  std::string partial_buffer_;
  static const size_t BUFFER_MAX_SIZE_ = 1024;
};
