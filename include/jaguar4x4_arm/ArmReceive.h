#pragma once

#include "AbstractCommunication.h"

class ArmReceive {
 public:
  ArmReceive(AbstractCommunication* comm);
  void getAndParseMessage();
 private:
  AbstractCommunication* comm_; // shared_ptr?
};
