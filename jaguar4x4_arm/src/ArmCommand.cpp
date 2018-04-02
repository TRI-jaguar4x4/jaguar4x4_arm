// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <mutex>
#include <string>

#include <jaguar4x4_comms/AbstractCommunication.h>

#include "jaguar4x4_arm/ArmCommand.h"

ArmCommand::ArmCommand(AbstractCommunication* comm)
  : comm_(comm)
{
}

std::string ArmCommand::buildArmCommand(ArmCommand::Joint arm, int value)
{
  std::string arm_command("!PR ");
  if (arm == Joint::lower_arm) {
    arm_command.append("1");
  } else {
    arm_command.append("2");
  }
  arm_command.append(" ");
  arm_command.append(std::to_string(value));
  arm_command.append("\r");

  return arm_command;
}

void ArmCommand::moveArmUp(ArmCommand::Joint arm, int value)
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand(buildArmCommand(arm, value));
}

void ArmCommand::moveArmDown(ArmCommand::Joint arm, int value)
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand(buildArmCommand(arm, value));
}

void ArmCommand::configure(uint32_t time_interval_ms)
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  // output the following messages every time_interval_ms
  std::string cmd("# C_?A_?AI_?C_?FF_?P_?S_?T_?V_# ");
  cmd += std::to_string(time_interval_ms);
  cmd += "\r";
  comm_->sendCommand(cmd);
}

void ArmCommand::resume()
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand("!MG\r");
}

void ArmCommand::eStop()
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand("!EX\r");
}

void ArmCommand::ping()
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand("~MMOD\r");
}
