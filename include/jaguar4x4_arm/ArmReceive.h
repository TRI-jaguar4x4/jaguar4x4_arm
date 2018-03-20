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
  AbstractArmMsg() {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t;
    std::chrono::duration<int, std::nano> now = t.time_since_epoch();
    ts_s_ = std::chrono::duration_cast<std::chrono::seconds>(now);
    ts_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  }
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
  
  std::pair<std::chrono::seconds, std::chrono::nanoseconds> getTime()
  {
    return std::make_pair(ts_s_, ts_ns_);
  }
protected:
  std::chrono::seconds ts_s_;
  std::chrono::nanoseconds ts_ns_;
};

class MotorAmpMsg : public AbstractArmMsg {
 public:
  MotorAmpMsg(double amp1, double amp2) : AbstractArmMsg(), motor_amp_1_(amp1), motor_amp_2_(amp2) {}
  std::tuple<double, double> get()
  {
    return std::make_tuple(motor_amp_1_, motor_amp_2_);
  }
 private:
  double motor_amp_1_;
  double motor_amp_2_;
};

class MotorTempMsg : public AbstractArmMsg {
 public:
  MotorTempMsg(uint16_t temp1, uint16_t temp2);
  
  std::tuple<uint16_t, double, uint16_t, double> get()
  {
    return std::make_tuple(motor_temp_adc_1_, motor_temp_1_, motor_temp_adc_2_, motor_temp_2_);
  }
 private:
  uint16_t motor_temp_adc_1_;
  double motor_temp_1_;
  uint16_t motor_temp_adc_2_;
  double motor_temp_2_;
};
