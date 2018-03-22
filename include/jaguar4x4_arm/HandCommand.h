#pragma once

#include <mutex>

#include "AbstractCommunication.h"

class HandSendLock {
public:
  HandSendLock(std::mutex &mtx) : mtx_(mtx)
  {
    mtx.lock();
  }

  ~HandSendLock()
  {
    mtx_.unlock();
  }

private:
  std::mutex &mtx_;
};

class HandCommand {
 public:
  enum class Joint {
    rotator,
    gripper,
  };

  HandCommand(AbstractCommunication* comm);
  // TODO: parameterize "how much" to move the hand up/down"
  void rotateHandLeft();
  void rotateHandRight();
  void gripperOpen();
  void gripperClose();
  void configure();
  void resume();
  void eStop();
  void ping();

private:
  std::string buildHandCommand(std::string cmd, HandCommand::Joint joint, int value);
  AbstractCommunication* comm_; // shared_ptr?
  std::mutex send_mutex;
};
