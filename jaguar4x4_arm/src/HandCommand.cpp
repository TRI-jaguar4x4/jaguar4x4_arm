// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <mutex>
#include <string>

#include <jaguar4x4_comms/AbstractCommunication.h>

#include "jaguar4x4_arm/HandCommand.h"

HandCommand::HandCommand(AbstractCommunication* comm)
  : comm_(comm)
{
}

std::string HandCommand::buildHandCommand(std::string cmd, HandCommand::Joint joint, int value)
{
  std::string hand_command(cmd);
  hand_command.append(" ");
  if (joint == Joint::rotator) {
    hand_command.append("1");
  } else {
    hand_command.append("2");
  }
  hand_command.append(" ");
  hand_command.append(std::to_string(value));
  hand_command.append("\r");

  return hand_command;
}

void HandCommand::rotateHandLeft(int value)
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand(buildHandCommand("!PR", HandCommand::Joint::rotator, value));
}

void HandCommand::rotateHandRight(int value)
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand(buildHandCommand("!PR", HandCommand::Joint::rotator, value));
}

void HandCommand::gripperOpen(int value)
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand(buildHandCommand("!G", HandCommand::Joint::gripper, value));
}

void HandCommand::gripperClose(int value)
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand(buildHandCommand("!G", HandCommand::Joint::gripper, value));
}

void HandCommand::gripperStop()
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand(buildHandCommand("!G", HandCommand::Joint::gripper, 0));
}

void HandCommand::configure(uint32_t time_interval_ms)
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  // output the following messages every time_interval_ms
  std::string cmd("# C_?A_?AI_?C_?FF_?P_?S_?T_?V_# ");
  cmd += std::to_string(time_interval_ms);
  cmd += "\r";
  comm_->sendCommand(cmd);
}

void HandCommand::resume()
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand("!MG\r");
}

void HandCommand::eStop()
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand("!EX\r");
}

void HandCommand::ping()
{
  std::lock_guard<std::mutex> send_lock(send_mutex);
  comm_->sendCommand("~MMOD\r");
}
