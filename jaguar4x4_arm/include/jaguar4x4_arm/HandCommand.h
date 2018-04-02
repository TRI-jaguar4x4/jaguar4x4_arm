// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <mutex>
#include <string>

#include <jaguar4x4_comms/AbstractCommunication.h>

class HandCommand {
 public:
  enum class Joint {
    rotator,
    gripper,
  };

  HandCommand(AbstractCommunication* comm);
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
  AbstractCommunication* comm_; // shared_ptr?
  std::mutex send_mutex;
};
