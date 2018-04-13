// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <jaguar4x4_comms/AbstractCommunication.h>

enum class ArmJoint {
  lower_arm,
  upper_arm,
};

class ArmCommand final {
 public:
  explicit ArmCommand(std::shared_ptr<AbstractCommunication> comm);
  void moveArmToRelativeEncoderPos(ArmJoint arm, int value);
  void moveArmToAbsoluteEncoderPos(ArmJoint arm, int value);
  void configure(uint32_t time_interval_ms);
  void resume();
  void eStop();
  void ping();

private:
  std::shared_ptr<AbstractCommunication> comm_;
  std::mutex send_mutex_;
};
