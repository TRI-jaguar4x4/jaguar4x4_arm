#pragma once

#include "AbstractCommunication.h"

class ArmCommand {
 public:
  enum Joint {
	lower_arm,
	upper_arm,
  };

  ArmCommand(AbstractCommunication* comm);
  // TODO: parameterize "how much" to move the arm up/down"
  // TODO: consolidate duplicated code in moves
  void moveArmUp(ArmCommand::Joint arm);
  void moveArmDown(ArmCommand::Joint arm);
  void resume();
  void eStop();
  void ping();
  
 private:
  AbstractCommunication* comm_; // shared ptr?
};
