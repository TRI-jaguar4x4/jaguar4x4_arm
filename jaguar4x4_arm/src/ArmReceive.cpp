// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <utility>

#include <jaguar4x4_comms/MotorParse.h>
#include <jaguar4x4_comms/Utils.h>

#include "jaguar4x4_arm/ArmReceive.h"

ArmReceive::ArmReceive(std::shared_ptr<AbstractCommunication> comm) : comm_(comm)
{
}

std::unique_ptr<AbstractMotorMsg> ArmReceive::getAndParseMessage()
{
  std::string msg = comm_->recvMessage("\r", 100);

  // nothing before the "\r"
  if (msg.empty()) {
    return std::unique_ptr<AbstractMotorMsg>(nullptr);
  }

  return parseMotorMessage(msg);
}
