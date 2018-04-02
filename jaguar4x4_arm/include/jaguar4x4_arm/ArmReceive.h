// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <memory>

#include <jaguar4x4_comms/AbstractCommunication.h>
#include <jaguar4x4_comms/MotorParse.h>

class ArmReceive {
 public:
  ArmReceive(std::shared_ptr<AbstractCommunication> comm);
  std::unique_ptr<AbstractMotorMsg> getAndParseMessage();

private:
  std::shared_ptr<AbstractCommunication> comm_;
};
