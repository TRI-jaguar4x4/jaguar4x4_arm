// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <string>

#include "jaguar4x4_arm/AbstractCommunication.h"
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

void HandCommand::rotateHandLeft()
{
  HandSendLock send_lock(send_mutex);
  comm_->sendCommand(buildHandCommand("!PR", HandCommand::Joint::rotator, 30));
}

void HandCommand::rotateHandRight()
{
  HandSendLock send_lock(send_mutex);
  comm_->sendCommand(buildHandCommand("!PR", HandCommand::Joint::rotator, -30));
}

void HandCommand::gripperOpen()
{
  HandSendLock send_lock(send_mutex);
  comm_->sendCommand(buildHandCommand("!G", HandCommand::Joint::gripper, 200));
}

void HandCommand::gripperClose()
{
  HandSendLock send_lock(send_mutex);
  comm_->sendCommand(buildHandCommand("!G", HandCommand::Joint::gripper, -200));
}

void HandCommand::gripperStop()
{
  HandSendLock send_lock(send_mutex);
  comm_->sendCommand(buildHandCommand("!G", HandCommand::Joint::gripper, 0));
}

void HandCommand::configure()
{
  HandSendLock send_lock(send_mutex);
  // output the following messages every 100 ms
  comm_->sendCommand("# C_?A_?AI_?C_?FF_?P_?S_?T_?V_# 100\r");
}

void HandCommand::resume()
{
  HandSendLock send_lock(send_mutex);
  comm_->sendCommand("!MG\r");
}

void HandCommand::eStop()
{
  HandSendLock send_lock(send_mutex);
  comm_->sendCommand("!EX\r");
}

void HandCommand::ping()
{
  HandSendLock send_lock(send_mutex);
  comm_->sendCommand("~MMOD\r");
}
