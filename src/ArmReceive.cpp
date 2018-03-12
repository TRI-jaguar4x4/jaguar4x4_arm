#include <iostream>

#include "jaguar4x4_arm/ArmReceive.h"

ArmReceive::ArmReceive(AbstractCommunication* comm) : comm_(comm)
{
}

void ArmReceive::getAndParseMessage()
{
  std::string msg = comm_->recvMessage("\r", 1000);
  std::cerr << "Message received: " << msg << std::endl;
}
