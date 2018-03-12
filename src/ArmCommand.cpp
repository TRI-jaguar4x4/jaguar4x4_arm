#include <string>

#include "jaguar4x4_arm/AbstractCommunication.h"
#include "jaguar4x4_arm/ArmCommand.h"

ArmCommand::ArmCommand(AbstractCommunication* comm)
  : comm_(comm)
{
}

void ArmCommand::moveArmUp(ArmCommand::Joint arm)
{
  std::string arm_command("!PR ");
  if (arm == lower_arm) {
    arm_command.append("1");
  } else {
    arm_command.append("2");
  }
  arm_command.append(" -30\r");

  comm_->sendCommand(arm_command);
}

void ArmCommand::moveArmDown(ArmCommand::Joint arm)
{
  std::string arm_command("!PR ");
  if (arm == lower_arm) {
    arm_command.append("1");
  } else {
    arm_command.append("2");
  }
  arm_command.append(" 30\r");
  
  comm_->sendCommand(arm_command);
}

void ArmCommand::resume()
{
  comm_->sendCommand("!MG\r");
}

void ArmCommand::eStop()
{
  comm_->sendCommand("!EX\r");
}

void ArmCommand::ping()
{
  comm_->sendCommand("~MMOD\r");
}

