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

class HandCommand final {
 public:
  enum class Joint {
    rotator,
    gripper,
  };

  explicit HandCommand(std::shared_ptr<AbstractCommunication> comm);
  void rotateHandLeft(int value);
  void rotateHandRight(int value);
  void gripperOpen(int value);
  void gripperClose(int value);
  void gripperStop();
  void configure(uint32_t time_interval_ms);
  void resume();
  void eStop();
  void ping();

private:
  std::string buildHandCommand(std::string cmd, HandCommand::Joint joint, int value);
  std::shared_ptr<AbstractCommunication> comm_;
  std::mutex send_mutex;
};
