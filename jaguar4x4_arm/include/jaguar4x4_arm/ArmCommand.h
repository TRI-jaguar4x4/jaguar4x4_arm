// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <jaguar4x4_comms/AbstractCommunication.h>

class ArmCommand {
 public:
  enum class Joint {
    lower_arm,
    upper_arm,
  };

  ArmCommand(std::shared_ptr<AbstractCommunication> comm);
  void moveArmUp(ArmCommand::Joint arm, int value);
  void moveArmDown(ArmCommand::Joint arm, int value);
  void configure(uint32_t time_interval_ms);
  void resume();
  void eStop();
  void ping();

private:
  std::string buildArmCommand(ArmCommand::Joint arm, int value);
  std::shared_ptr<AbstractCommunication> comm_;
  std::mutex send_mutex;
};
