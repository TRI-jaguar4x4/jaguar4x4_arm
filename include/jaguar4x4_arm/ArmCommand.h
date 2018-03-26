// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <mutex>

#include "AbstractCommunication.h"

class ArmCommand {
 public:
  enum class Joint {
    lower_arm,
    upper_arm,
  };

  ArmCommand(AbstractCommunication* comm);
  // TODO: parameterize "how much" to move the arm up/down"
  void moveArmUp(ArmCommand::Joint arm, int value);
  void moveArmDown(ArmCommand::Joint arm, int value);
  void configure(uint32_t time_interval_ms);
  void resume();
  void eStop();
  void ping();

private:
  std::string buildArmCommand(ArmCommand::Joint arm, int value);
  AbstractCommunication* comm_; // shared_ptr?
  std::mutex send_mutex;
};
