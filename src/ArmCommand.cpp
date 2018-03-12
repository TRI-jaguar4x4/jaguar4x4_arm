#include <string>

#include "jaguar4x4_arm/AbstractCommunication.h"
#include "jaguar4x4_arm/ArmCommand.h"

ArmCommand::ArmCommand(AbstractCommunication* comm)
  : comm_(comm)
{
}

std::string ArmCommand::buildArmCommand(ArmCommand::Joint arm, int value)
{
  std::string arm_command("!PR ");
  if (arm == lower_arm) {
    arm_command.append("1");
  } else {
    arm_command.append("2");
  }
  arm_command.append(" ");
  arm_command.append(std::to_string(value));
  arm_command.append("\r");

  return arm_command;
}

void ArmCommand::moveArmUp(ArmCommand::Joint arm)
{
  ArmSendLock send_lock(send_mutex);
  comm_->sendCommand(buildArmCommand(arm, -30));
}

void ArmCommand::moveArmDown(ArmCommand::Joint arm)
{
  ArmSendLock send_lock(send_mutex);
  comm_->sendCommand(buildArmCommand(arm, 30));
}

void ArmCommand::resume()
{
  ArmSendLock send_lock(send_mutex);
  comm_->sendCommand("!MG\r");
}

void ArmCommand::eStop()
{
  ArmSendLock send_lock(send_mutex);
  comm_->sendCommand("!EX\r");
}

void ArmCommand::ping()
{
  ArmSendLock send_lock(send_mutex);
  comm_->sendCommand("~MMOD\r");
}
