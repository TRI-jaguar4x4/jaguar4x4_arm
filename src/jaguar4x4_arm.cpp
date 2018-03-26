// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <atomic>
#include <chrono>
#include <future>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jaguar4x4_arm/ArmCommand.h"
#include "jaguar4x4_arm/ArmReceive.h"
#include "jaguar4x4_arm/HandCommand.h"

#include "jaguar4x4_arm/Communication.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Jaguar4x4Arm : public rclcpp::Node
{
public:
  Jaguar4x4Arm(const std::string& ip, uint16_t arm_port, uint16_t hand_port)
    : Node("jaguar4x4Arm"), arm_recv_thread_(), hand_recv_thread_(),
      num_pings_sent_(0), num_lift_pings_recvd_(0), num_hand_pings_recvd_(0),
      accepting_commands_(false)
  {
    lift_cmd_ = std::make_unique<ArmCommand>(&board_1_comm_);
    lift_rcv_ = std::make_unique<ArmReceive>(&board_1_comm_);
    board_1_comm_.connect(ip, arm_port);

    hand_cmd_ = std::make_unique<HandCommand>(&board_2_comm_);
    hand_rcv_ = std::make_unique<ArmReceive>(&board_2_comm_);
    board_2_comm_.connect(ip, hand_port);

    rmw_qos_profile_t z_position_qos_profile = rmw_qos_profile_sensor_data;
    z_position_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    z_position_qos_profile.depth = 50;
    z_position_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    z_position_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    z_pos_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "z_position", std::bind(&Jaguar4x4Arm::liftCallback,
                                this, std::placeholders::_1),
        z_position_qos_profile);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(Jaguar4x4Arm::kTimerIntervalMS),
      std::bind(&Jaguar4x4Arm::timerCallback, this));

    lift_cmd_->configure();
    lift_cmd_->resume();

    hand_cmd_->configure();
    hand_cmd_->resume();

    future_ = exit_signal_.get_future();

    arm_recv_thread_ = std::thread(&Jaguar4x4Arm::armRecvThread, this,
                               future_, std::move(lift_rcv_));
    hand_recv_thread_ = std::thread(&Jaguar4x4Arm::handRecvThread, this,
                               future_, std::move(hand_rcv_));
  }

  ~Jaguar4x4Arm()
  {
    exit_signal_.set_value();
    arm_recv_thread_.join();
    hand_recv_thread_.join();
  }

private:
  // TODO: find out how long it takes the robot to respond.
  // Bumping kPingRecvPercentage_ down to .6 from .8 seems to make things work
  // TODO:  figure out why static isn't working (linker unhappiness mystery)
  const uint32_t kTimerIntervalMS = 100;
  const uint32_t kWatchdogIntervalMS = 500;
  static constexpr double   kPingRecvPercentage = 0.6;
  const uint32_t kPingsPerWatchdogInterval = kWatchdogIntervalMS/kTimerIntervalMS;
  const uint32_t kMinPingsExpected = kPingsPerWatchdogInterval*kPingRecvPercentage;

  // clalancette: To test this by hand, the following command-line can
  // be used:
  // ros2 topic pub /z_position geometry_msgs/PoseStamped "{header:{stamp:{sec: 4, nanosec: 14}, frame_id: 'frame'}, pose:{position:{x: 1, y: 2, z: 3}, orientation:{x: 4, y: 5, z: 6, w: 7}}}"
  void liftCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!accepting_commands_) {
      return;
    }

    int64_t this_stamp;
    this_stamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

    if (this_stamp < last_stamp_) {
      // An older command came through, just ignore it.  Note that we currently
      // allow equal timestamps to make using the command-line easier.
      return;
    }
    last_stamp_ = this_stamp;
    // move arm up/down
    if (msg->pose.position.z > 0) {
      lift_cmd_->moveArmDown(ArmCommand::Joint::lower_arm, msg->pose.position.z);
      lift_cmd_->moveArmDown(ArmCommand::Joint::upper_arm, msg->pose.position.z);
    } else if (msg->pose.position.z < 0) {
      lift_cmd_->moveArmUp(ArmCommand::Joint::lower_arm, msg->pose.position.z);
      lift_cmd_->moveArmUp(ArmCommand::Joint::upper_arm, msg->pose.position.z);
    }

    // rotate wrist around x
    if (msg->pose.position.x > 0) {
      hand_cmd_->rotateHandLeft(msg->pose.position.x);
    } else if (msg->pose.position.x < 0){
      hand_cmd_->rotateHandRight(msg->pose.position.x);
    }

    // open/close gripper as y
    if (msg->pose.position.y > 0) {
      hand_cmd_->gripperOpen(msg->pose.position.y);
    } else if (msg->pose.position.y < 0){
      hand_cmd_->gripperClose(msg->pose.position.y);
    } else {
      hand_cmd_->gripperStop();
    }
  }

  void armRecvThread(std::shared_future<void> local_future,
                    std::unique_ptr<ArmReceive> lift_recv)
  {
    std::cerr << "entered the callback\n";
    std::future_status status;
    std::vector<std::unique_ptr<AbstractArmMsg>> arm_msgs;
    do {
      try {
        arm_msgs = lift_recv->getAndParseMessage();
      }
      catch (...) {
        std::cerr << "threw\n";
      }
      for(std::unique_ptr<AbstractArmMsg>& msg : arm_msgs) {
        switch(msg->getType()) {
          case AbstractArmMsg::MessageType::motor_amperage:
          {
            MotorAmpMsg *motor_amp = dynamic_cast<MotorAmpMsg*>(msg.get());
            std::cerr << "motor_amperage: " << motor_amp->motor_amp_1_ << " "
                      << motor_amp->motor_amp_2_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::motor_temperature:
          {
            MotorTempMsg *motor_temp = dynamic_cast<MotorTempMsg*>(msg.get());
            std::cerr << "motor_temperature: "
                      << motor_temp->motor_temp_adc_1_ << " "
                      << motor_temp->motor_temp_1_ << " "
                      << motor_temp->motor_temp_adc_2_ << " "
                      << motor_temp->motor_temp_2_ << " " << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::encoder_position:
          {
            MotorEncPosMsg *motor_enc_pos =
              dynamic_cast<MotorEncPosMsg*>(msg.get());
            std::cerr << "encoder_position: "
                      << motor_enc_pos->encoder_pos_1_ << " "
                      << motor_enc_pos->encoder_pos_2_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::motor_power:
          {
            MotorPowerMsg *motor_power =
              dynamic_cast<MotorPowerMsg*>(msg.get());
            std::cerr << "motor_power: " << motor_power->motor_power_1_ << " "
                      << motor_power->motor_power_2_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::encoder_velocity:
          {
            MotorEncVelMsg *motor_enc_vel =
              dynamic_cast<MotorEncVelMsg*>(msg.get());
            std::cerr << "encoder_velocity: "
                      << motor_enc_vel->encoder_velocity_1_ << " "
                      << motor_enc_vel->encoder_velocity_2_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::board_temperature:
          {
            MotorBoardTempMsg *motor_board_temp =
              dynamic_cast<MotorBoardTempMsg*>(msg.get());
            std::cerr << "board_temperature: "
                      << motor_board_temp->board_temp_1_ << " "
                      << motor_board_temp->board_temp_2_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::voltage:
          {
            MotorVoltageMsg *motor_voltage =
              dynamic_cast<MotorVoltageMsg*>(msg.get());
            std::cerr << "voltage: " << motor_voltage->drv_voltage_ << " "
                      << motor_voltage->bat_voltage_ << " "
                      << motor_voltage->reg_5_voltage_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::motor_mode:
          {
            num_lift_pings_recvd_++;
            MotorModeMsg *motor_mode = dynamic_cast<MotorModeMsg*>(msg.get());
            std::cerr << "motor_mode: "
                      << static_cast<std::underlying_type
                                     <MotorModeMsg::MotorControlMode>::type>
                                     (motor_mode->mode_channel_1_)
                      << " "
                      << static_cast<std::underlying_type
                                     <MotorModeMsg::MotorControlMode>::type>
                                     (motor_mode->mode_channel_2_)
                      << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::motor_flags:
          {
            MotorFlagsMsg *motor_flags =
              dynamic_cast<MotorFlagsMsg*>(msg.get());
            std::cerr << "motor_flags: "
                      << "overheat=" << motor_flags->overheat_
                      << " overvoltage=" << motor_flags->overvoltage_
                      << " undervoltage=" << motor_flags->undervoltage_
                      << " short=" << motor_flags->short_
                      <<  " ESTOP=" << motor_flags->ESTOP_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::command_accepted:
          {
            std::cerr << "command_accepted\n";
          }
          break;
          case AbstractArmMsg::MessageType::command_rejected:
          {
            std::cerr << "command_rejected\n";
          }
          break;
        }
      }
      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void handRecvThread(std::shared_future<void> local_future,
                    std::unique_ptr<ArmReceive> hand_recv)
  {
    std::cerr << "entered the hand callback\n";
    std::future_status status;
    std::vector<std::unique_ptr<AbstractArmMsg>> hand_msgs;
    do {
      try {
        hand_msgs = hand_recv->getAndParseMessage();
      }
      catch (...) {
        std::cerr << "threw\n";
      }
      for(std::unique_ptr<AbstractArmMsg>& msg : hand_msgs) {
        switch(msg->getType()) {
          case AbstractArmMsg::MessageType::motor_amperage:
          {
            MotorAmpMsg *motor_amp = dynamic_cast<MotorAmpMsg*>(msg.get());
            std::cerr << "        motor_amperage: " << motor_amp->motor_amp_1_ << " "
                      << motor_amp->motor_amp_2_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::motor_temperature:
          {
            MotorTempMsg *motor_temp = dynamic_cast<MotorTempMsg*>(msg.get());
            std::cerr << "        motor_temperature: "
                      << motor_temp->motor_temp_adc_1_ << " "
                      << motor_temp->motor_temp_1_ << " "
                      << motor_temp->motor_temp_adc_2_ << " "
                      << motor_temp->motor_temp_2_ << " " << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::encoder_position:
          {
            MotorEncPosMsg *motor_enc_pos =
              dynamic_cast<MotorEncPosMsg*>(msg.get());
            std::cerr << "        encoder_position: "
                      << motor_enc_pos->encoder_pos_1_ << " "
                      << motor_enc_pos->encoder_pos_2_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::motor_power:
          {
            MotorPowerMsg *motor_power =
              dynamic_cast<MotorPowerMsg*>(msg.get());
            std::cerr << "        motor_power: " << motor_power->motor_power_1_ << " "
                      << motor_power->motor_power_2_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::encoder_velocity:
          {
            MotorEncVelMsg *motor_enc_vel =
              dynamic_cast<MotorEncVelMsg*>(msg.get());
            std::cerr << "        encoder_velocity: "
                      << motor_enc_vel->encoder_velocity_1_ << " "
                      << motor_enc_vel->encoder_velocity_2_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::board_temperature:
          {
            MotorBoardTempMsg *motor_board_temp =
              dynamic_cast<MotorBoardTempMsg*>(msg.get());
            std::cerr << "        board_temperature: "
                      << motor_board_temp->board_temp_1_ << " "
                      << motor_board_temp->board_temp_2_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::voltage:
          {
            MotorVoltageMsg *motor_voltage =
              dynamic_cast<MotorVoltageMsg*>(msg.get());
            std::cerr << "        voltage: " << motor_voltage->drv_voltage_ << " "
                      << motor_voltage->bat_voltage_ << " "
                      << motor_voltage->reg_5_voltage_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::motor_mode:
          {
            num_hand_pings_recvd_++;
            MotorModeMsg *motor_mode = dynamic_cast<MotorModeMsg*>(msg.get());
            std::cerr << "        motor_mode: "
                      << static_cast<std::underlying_type
                                     <MotorModeMsg::MotorControlMode>::type>
                                     (motor_mode->mode_channel_1_)
                      << " "
                      << static_cast<std::underlying_type
                                     <MotorModeMsg::MotorControlMode>::type>
                                     (motor_mode->mode_channel_2_)
                      << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::motor_flags:
          {
            MotorFlagsMsg *motor_flags =
              dynamic_cast<MotorFlagsMsg*>(msg.get());
            std::cerr << "        motor_flags: "
                      << "overheat=" << motor_flags->overheat_
                      << " overvoltage=" << motor_flags->overvoltage_
                      << " undervoltage=" << motor_flags->undervoltage_
                      << " short=" << motor_flags->short_
                      <<  " ESTOP=" << motor_flags->ESTOP_ << "\n";
          }
          break;
          case AbstractArmMsg::MessageType::command_accepted:
          {
            std::cerr << "        command_accepted\n";
          }
          break;
          case AbstractArmMsg::MessageType::command_rejected:
          {
            std::cerr << "        command_rejected\n";
          }
          break;
        }
      }
      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void timerCallback()
  {
    // NOTE:  this implementation causes accepting_commands_ to be false
    // for the first 500ms of operation
    lift_cmd_->ping();
    hand_cmd_->ping();
    num_pings_sent_++;

    if (num_pings_sent_ < kPingsPerWatchdogInterval) {
      return;
    }

    std::cerr << "num_lift_pings_recvd_ = " << num_lift_pings_recvd_ <<  " num_hand_pings_recvd_ = " << num_hand_pings_recvd_ << "\n";
    if (num_lift_pings_recvd_ < kMinPingsExpected
        || num_hand_pings_recvd_ < kMinPingsExpected) {
      std::cerr << "haven't heard from the robot... no longer accepting commands\n";
      accepting_commands_ = false;
    } else {
      std::cerr << "have heard from the robot... go ahead and command it\n";
      accepting_commands_ = true;
    }
    num_lift_pings_recvd_ = 0;
    num_hand_pings_recvd_ = 0;
    num_pings_sent_ = 0;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr z_pos_cmd_sub_;
  Communication                board_1_comm_;
  Communication                board_2_comm_;
  std::unique_ptr<ArmCommand>  lift_cmd_;
  std::unique_ptr<ArmReceive>  lift_rcv_;
  std::unique_ptr<HandCommand> hand_cmd_;
  std::unique_ptr<ArmReceive>  hand_rcv_;
  int64_t                      last_stamp_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread                  arm_recv_thread_;
  std::thread                  hand_recv_thread_;
  std::promise<void>           exit_signal_;
  std::shared_future<void>     future_;
  uint32_t                     num_pings_sent_;
  std::atomic<uint32_t>        num_lift_pings_recvd_;
  std::atomic<uint32_t>        num_hand_pings_recvd_;
  std::atomic<bool>            accepting_commands_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jaguar4x4Arm>("192.168.0.63", 10001, 10002));
  rclcpp::shutdown();
  return 0;
}
