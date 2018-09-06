// Copyright 2018 Toyota Research Institute.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
