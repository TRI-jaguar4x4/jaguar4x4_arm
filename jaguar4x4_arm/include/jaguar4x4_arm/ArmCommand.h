// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <jaguar4x4_comms/AbstractCommunication.h>

// The motors used in the Jaguar4x4 arm are the Roboteq SDC2130:
// https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file

enum class ArmJoint {
  lower_arm = 1,
  upper_arm = 2,
};

enum class ArmMotorMode {
  open_loop = 0,
  closed_loop_speed = 1,
  closed_loop_pos_relative = 2,
  closed_loop_count_position = 3,
  closed_loop_pos_tracking = 4,
  torque = 5,
  closed_loop_speed_pos = 6,
};

class ArmCommand final {
 public:
  explicit ArmCommand(std::shared_ptr<AbstractCommunication> comm);
  void moveArmToRelativeEncoderPos(ArmJoint arm, int value);
  void moveArmToAbsoluteEncoderPos(ArmJoint arm, int value);
  void moveArmAtSpeed(ArmJoint arm, int value);
  void configure(uint32_t time_interval_ms);
  void resume();
  void eStop();
  void ping();
  void setMotorMode(ArmJoint arm, ArmMotorMode mode);
  void setMotorMaxRPM(ArmJoint arm, int value);
  void setArmPositionControlSpeed(ArmJoint arm, int value);
  void setMotorAcceleration(ArmJoint arm, int value);
  void setMotorPID(ArmJoint arm, int p, int i, int d);

private:
  std::shared_ptr<AbstractCommunication> comm_;
  std::mutex send_mutex_;
};
