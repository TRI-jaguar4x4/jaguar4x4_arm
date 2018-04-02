// Copyright 2018 Toyota Research Institute.  All rights reserved.
#pragma once

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <jaguar4x4_comms/AbstractCommunication.h>

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
    motor_flags,        // FF
    command_accepted,   // +
    command_rejected,   // -
  };

  explicit AbstractArmMsg(AbstractArmMsg::MessageType msg_type)
    : msg_type_(msg_type) {
    std::chrono::time_point<std::chrono::system_clock,
                            std::chrono::nanoseconds> t;
    std::chrono::duration<int, std::nano> now = t.time_since_epoch();
    ts_s_ = std::chrono::duration_cast<std::chrono::seconds>(now);
    ts_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  }

  virtual ~AbstractArmMsg() = default;

  std::pair<std::chrono::seconds, std::chrono::nanoseconds> getTime() const
  {
    return std::make_pair(ts_s_, ts_ns_);
  }

  AbstractArmMsg::MessageType getType() const
  {
    return msg_type_;
  }

protected:
  AbstractArmMsg::MessageType msg_type_;
  std::chrono::seconds ts_s_;
  std::chrono::nanoseconds ts_ns_;
};

class MotorAmpMsg : public AbstractArmMsg {
 public:
  MotorAmpMsg(double amp1, double amp2)
    : AbstractArmMsg(AbstractArmMsg::MessageType::motor_amperage),
      motor_amp_1_(amp1),
      motor_amp_2_(amp2)
  {}

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



class MotorEncPosMsg : public AbstractArmMsg {
 public:
  MotorEncPosMsg(int64_t enc_pos_1, int64_t enc_pos_2)
    : AbstractArmMsg(AbstractArmMsg::MessageType::encoder_position),
      encoder_pos_1_(enc_pos_1),
      encoder_pos_2_(enc_pos_2)
  {}

  int64_t encoder_pos_1_;
  int64_t encoder_pos_2_;
};

class MotorPowerMsg : public AbstractArmMsg {
 public:
  MotorPowerMsg(int16_t motor_power_1, int16_t motor_power_2)
    : AbstractArmMsg(AbstractArmMsg::MessageType::motor_power),
      motor_power_1_(motor_power_1),
      motor_power_2_(motor_power_2)
  {}

  int16_t motor_power_1_;
  int16_t motor_power_2_;
};

class MotorEncVelMsg : public AbstractArmMsg {
 public:
  MotorEncVelMsg(int64_t encoder_velocity_1, int64_t encoder_velocity_2)
    : AbstractArmMsg(AbstractArmMsg::MessageType::encoder_velocity),
      encoder_velocity_1_(encoder_velocity_1),
      encoder_velocity_2_(encoder_velocity_2)
  {}

  int64_t encoder_velocity_1_;
  int64_t encoder_velocity_2_;
};

class MotorBoardTempMsg : public AbstractArmMsg {
 public:
  MotorBoardTempMsg(double temp_1, double temp_2)
    : AbstractArmMsg(AbstractArmMsg::MessageType::board_temperature),
      board_temp_1_(temp_1),
      board_temp_2_(temp_2)
  {}

  double board_temp_1_;
  double board_temp_2_;
};

class MotorVoltageMsg : public AbstractArmMsg {
 public:
  MotorVoltageMsg(double v_1, double v_2, double v_3)
    : AbstractArmMsg(AbstractArmMsg::MessageType::voltage),
      drv_voltage_(v_1),
      bat_voltage_(v_2),
      reg_5_voltage_(v_3)
  {}

  double drv_voltage_;
  double bat_voltage_;
  double reg_5_voltage_;
};

class MotorFlagsMsg : public AbstractArmMsg {
 public:
  MotorFlagsMsg(uint16_t flag) :
    AbstractArmMsg(AbstractArmMsg::MessageType::motor_flags)
  {
    overheat_ = (flag & 0x01);
    overvoltage_ = (flag & 0x02);
    undervoltage_ = (flag & 0x04);
    short_ = (flag & 0x08);
    ESTOP_ = (flag & 0x10);
  }

  bool overheat_;
  bool overvoltage_;
  bool undervoltage_;
  bool short_;
  bool ESTOP_;
};

class MotorModeMsg : public AbstractArmMsg {
 public:
  enum class MotorControlMode {
    OpenLoop = 0,
    ClosedSpeed = 1,
    Position_2=2,
    Position_3=3,
    Torque=4,
  };

  MotorModeMsg(int mode_1, int mode_2)
    : AbstractArmMsg(AbstractArmMsg::MessageType::motor_mode),
      mode_channel_1_(MotorModeMsg::MotorControlMode(mode_1)),
      mode_channel_2_(MotorModeMsg::MotorControlMode(mode_2))
  {}

  MotorModeMsg::MotorControlMode mode_channel_1_;
  MotorModeMsg::MotorControlMode mode_channel_2_;
};

class MotorCmdAcceptedMsg : public AbstractArmMsg {
 public:
  MotorCmdAcceptedMsg()
    : AbstractArmMsg(AbstractArmMsg::MessageType::command_accepted)
  {}
};

class MotorCmdRejectedMsg : public AbstractArmMsg {
 public:
  MotorCmdRejectedMsg()
    : AbstractArmMsg(AbstractArmMsg::MessageType::command_rejected)
  {}
};

class ArmReceive {
 public:
  ArmReceive(std::shared_ptr<AbstractCommunication> comm);
  std::vector<std::unique_ptr<AbstractArmMsg>> getAndParseMessage();

private:
  std::shared_ptr<AbstractCommunication> comm_;
};
