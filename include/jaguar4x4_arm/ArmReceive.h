#pragma once

#include <chrono>
#include <utility>

#include "AbstractCommunication.h"

class ArmReceive {
 public:
  ArmReceive(AbstractCommunication* comm);
  void getAndParseMessage();
 private:
  AbstractCommunication* comm_; // shared_ptr?
};

class AbstractArmMsg {
 public:
  enum class MessageType {
    motor_amperage,     // A
    motor_temperature,  // AI
    encoder_position,   // C
    motor_power,        // P
    encoder_velocity,   // S
    board_temperature,  // T
    voltage,            // V
    motor_mode,         // MMOD
    flags,              // FF
    command_accepted,   // +
    command_rejected,   // -
  };
  
  explicit AbstractArmMsg(AbstractArmMsg::MessageType msg_type) : msg_type_(msg_type) {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t;
    std::chrono::duration<int, std::nano> now = t.time_since_epoch();
    ts_s_ = std::chrono::duration_cast<std::chrono::seconds>(now);
    ts_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  }

  std::pair<std::chrono::seconds, std::chrono::nanoseconds> getTime()
  {
    return std::make_pair(ts_s_, ts_ns_);
  }
protected:
  AbstractArmMsg::MessageType msg_type_;
  std::chrono::seconds ts_s_;
  std::chrono::nanoseconds ts_ns_;
};

class MotorAmpMsg : public AbstractArmMsg {
 public:
  MotorAmpMsg(double amp1, double amp2) : AbstractArmMsg(AbstractArmMsg::MessageType::motor_amperage), motor_amp_1_(amp1), motor_amp_2_(amp2) {}

  double motor_amp_1_;
  double motor_amp_2_;
};

class MotorTempMsg : public AbstractArmMsg {
 public:
  MotorTempMsg(uint16_t temp1, uint16_t temp2);
  
  uint16_t motor_temp_adc_1_;
  double motor_temp_1_;
  uint16_t motor_temp_adc_2_;
  double motor_temp_2_;
};
