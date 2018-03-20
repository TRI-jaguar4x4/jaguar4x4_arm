#pragma once

#include <mutex>

#include "AbstractCommunication.h"

class ArmSendLock {
public:
  ArmSendLock(std::mutex &mtx) : mtx_(mtx)
  {
    mtx.lock();
  }

  ~ArmSendLock()
  {
    mtx_.unlock();
  }
private:
  std::mutex &mtx_;
};

class ArmCommand {
 public:
  enum class Joint {
    lower_arm,
    upper_arm,
  };

  ArmCommand(AbstractCommunication* comm);
  // TODO: parameterize "how much" to move the arm up/down"
  void moveArmUp(ArmCommand::Joint arm);
  void moveArmDown(ArmCommand::Joint arm);
  void configure();
  void resume();
  void eStop();
  void ping();

private:
  std::string buildArmCommand(ArmCommand::Joint arm, int value);
  AbstractCommunication* comm_; // shared_ptr?
  std::mutex send_mutex;
};
