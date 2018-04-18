// Copyright 2018 Toyota Research Institute.  All rights reserved.

#include <iostream>
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

  arm_command.append(std::to_string(static_cast<int>(arm)));
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

void ArmCommand::moveArmAtSpeed(ArmJoint arm, int value)
{
  std::string arm_command("!G ");
  arm_command.append(std::to_string(static_cast<int>(arm)));
  arm_command.append(" ");
  arm_command.append(std::to_string(value));
  arm_command.append("\r");

  std::lock_guard<std::mutex> send_lock(send_mutex_);
  comm_->sendCommand(arm_command);
}

void ArmCommand::configure(uint32_t time_interval_ms)
{
  // output the following messages every time_interval_ms
  std::string cmd("# C_?A_?AI_?C_?FF_?P_?S_?T_?V_# ");
  cmd += std::to_string(time_interval_ms);
  cmd += "\r";

  std::lock_guard<std::mutex> send_lock(send_mutex_);
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

void ArmCommand::setMotorMode(ArmJoint arm, ArmMotorMode mode)
{
  std::string arm_command("^MMOD ");

  arm_command.append(std::to_string(static_cast<int>(arm)));
  arm_command.append(" ");
  arm_command.append(std::to_string(static_cast<int>(mode)));
  arm_command.append("\r");

  std::lock_guard<std::mutex> send_lock(send_mutex_);
  std::cerr << "Sending command: " << arm_command << std::endl;
  comm_->sendCommand(arm_command);
}

void ArmCommand::getMotorMaxRPM(ArmJoint arm)
{
  std::lock_guard<std::mutex> send_lock(send_mutex_);
  comm_->sendCommand("~MXRPM 1\r");
  comm_->sendCommand("~MAC 1\r");
}

void ArmCommand::setMotorMaxRPM(ArmJoint arm)
{
  std::lock_guard<std::mutex> send_lock(send_mutex_);
  comm_->sendCommand("^MXRPM 1 50000\r");
  comm_->sendCommand("^MAC 1 500000\r");
}
