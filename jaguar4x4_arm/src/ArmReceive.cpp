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
