// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <memory>
#include <mutex>
#include <string>

#include <jaguar4x4_comms/AbstractCommunication.h>

#include "jaguar4x4_arm/ArmCommand.h"

ArmCommand::ArmCommand(std::shared_ptr<AbstractCommunication> comm)
  : comm_(comm)
{
}

enum class ArmEncoderMotion
{
  RELATIVE,
  ABSOLUTE,
};

static std::string buildArmCommand(ArmEncoderMotion motion, ArmJoint arm, int value)
{
  std::string arm_command("!");
  switch (motion) {
  case ArmEncoderMotion::RELATIVE:
    arm_command.append("PR ");
    break;
  case ArmEncoderMotion::ABSOLUTE:
    arm_command.append("P ");
    break;
  }

  switch (arm) {
  case ArmJoint::lower_arm:
    arm_command.append("1");
    break;
  case ArmJoint::upper_arm:
    arm_command.append("2");
    break;
  }

  arm_command.append(" ");
  arm_command.append(std::to_string(value));
  arm_command.append("\r");

  return arm_command;
}

void ArmCommand::moveArmToRelativeEncoderPos(ArmJoint arm, int value)
{
  std::lock_guard<std::mutex> send_lock(send_mutex_);
  comm_->sendCommand(buildArmCommand(ArmEncoderMotion::RELATIVE, arm, value));
}

void ArmCommand::moveArmToAbsoluteEncoderPos(ArmJoint arm, int value)
{
  std::lock_guard<std::mutex> send_lock(send_mutex_);
  comm_->sendCommand(buildArmCommand(ArmEncoderMotion::ABSOLUTE, arm, value));
}

void ArmCommand::configure(uint32_t time_interval_ms)
{
  std::lock_guard<std::mutex> send_lock(send_mutex_);
  // output the following messages every time_interval_ms
  std::string cmd("# C_?A_?AI_?C_?FF_?P_?S_?T_?V_# ");
  cmd += std::to_string(time_interval_ms);
  cmd += "\r";
  comm_->sendCommand(cmd);
}

void ArmCommand::resume()
{
  std::lock_guard<std::mutex> send_lock(send_mutex_);
  comm_->sendCommand("!MG\r");
}

void ArmCommand::eStop()
{
  std::lock_guard<std::mutex> send_lock(send_mutex_);
  comm_->sendCommand("!EX\r");
}

void ArmCommand::ping()
{
  std::lock_guard<std::mutex> send_lock(send_mutex_);
  comm_->sendCommand("~MMOD\r");
}
