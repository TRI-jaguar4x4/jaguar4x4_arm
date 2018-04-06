// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <jaguar4x4_comms/AbstractCommunication.h>

class HandCommand final {
 public:
  enum class Joint {
    rotator,
    gripper,
  };

  explicit HandCommand(std::shared_ptr<AbstractCommunication> comm);
  void rotateHandLeft(int value);
  void rotateHandRight(int value);
  void gripperOpen(int value);
  void gripperClose(int value);
  void gripperStop();
  void configure(uint32_t time_interval_ms);
  void resume();
  void eStop();
  void ping();

private:
  std::string buildHandCommand(std::string cmd, HandCommand::Joint joint, int value);
  std::shared_ptr<AbstractCommunication> comm_;
  std::mutex send_mutex;
};
