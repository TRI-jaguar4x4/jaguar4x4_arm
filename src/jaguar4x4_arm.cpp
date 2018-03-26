// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jaguar4x4_arm_msgs/msg/lift.hpp"
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

    lift_pub_msg_ = std::make_shared<jaguar4x4_arm_msgs::msg::Lift>();
    lift_pub_ = this->create_publisher<jaguar4x4_arm_msgs::msg::Lift>("liftInfo");

    ping_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(Jaguar4x4Arm::kPingTimerIntervalMS),
      std::bind(&Jaguar4x4Arm::pingTimerCallback, this));

    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(Jaguar4x4Arm::kPubTimerIntervalMS),
      std::bind(&Jaguar4x4Arm::pubTimerCallback, this));

    lift_cmd_->configure(kPubTimerIntervalMS);
    lift_cmd_->resume();

    hand_cmd_->configure(kPubTimerIntervalMS);
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
  // TODO:  figure out why static isn't working (linker unhappiness mystery)
  const uint32_t kPubTimerIntervalMS = 100;

  // TODO: find out how long it takes the robot to respond.
  // Bumping kPingRecvPercentage_ down to .6 from .8 seems to make things work
  const uint32_t kPingTimerIntervalMS = 100;
  const uint32_t kWatchdogIntervalMS = 500;
  static constexpr double   kPingRecvPercentage = 0.6;
  const uint32_t kPingsPerWatchdogInterval = kWatchdogIntervalMS/kPingTimerIntervalMS;
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
    // TODO - transform these numbers... we're passing the twist numbers
    // down directly and that's not right

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
    std::future_status status;
    std::vector<std::unique_ptr<AbstractArmMsg>> arm_msgs;
    do {
      try {
        arm_msgs = lift_recv->getAndParseMessage();
      }
      catch (...) {
        std::cerr << "threw\n";
      }
      std::lock_guard<std::mutex> sf_lock_guard(sensor_frame_mutex_);
      for(std::unique_ptr<AbstractArmMsg>& msg : arm_msgs) {
        switch(msg->getType()) {
          case AbstractArmMsg::MessageType::motor_amperage:
          {
            MotorAmpMsg *motor_amp = dynamic_cast<MotorAmpMsg*>(msg.get());
            lift_pub_msg_->arm.amp_1 = motor_amp->motor_amp_1_;
            lift_pub_msg_->arm.amp_2 = motor_amp->motor_amp_2_;
          }
          break;
          case AbstractArmMsg::MessageType::motor_temperature:
          {
            MotorTempMsg *motor_temp = dynamic_cast<MotorTempMsg*>(msg.get());
            lift_pub_msg_->arm.motor_temp_1 = motor_temp->motor_temp_1_;
            lift_pub_msg_->arm.motor_temp_2 = motor_temp->motor_temp_2_;
          }
          break;
          case AbstractArmMsg::MessageType::encoder_position:
          {
            MotorEncPosMsg *motor_enc_pos =
              dynamic_cast<MotorEncPosMsg*>(msg.get());
            lift_pub_msg_->arm.encoder_pos_1 = motor_enc_pos->encoder_pos_1_;
            lift_pub_msg_->arm.encoder_pos_2 = motor_enc_pos->encoder_pos_2_;
          }
          break;
          case AbstractArmMsg::MessageType::motor_power:
          {
            // TODO we've seen this coming out negative... what's up with that?
            // TODO what are the units?  What does this number mean?
            // 25 for power 1 and 0 for power 2 on the arm,
            // -22 and -1 for power on the arm?
            MotorPowerMsg *motor_power =
              dynamic_cast<MotorPowerMsg*>(msg.get());
            lift_pub_msg_->arm.motor_power_1 = motor_power->motor_power_1_;
            lift_pub_msg_->arm.motor_power_2 = motor_power->motor_power_2_;
          }
          break;
          case AbstractArmMsg::MessageType::encoder_velocity:
          {
            MotorEncVelMsg *motor_enc_vel =
              dynamic_cast<MotorEncVelMsg*>(msg.get());
            lift_pub_msg_->arm.encoder_vel_1 = motor_enc_vel->encoder_velocity_1_;
            lift_pub_msg_->arm.encoder_vel_2 = motor_enc_vel->encoder_velocity_2_;
          }
          break;
          case AbstractArmMsg::MessageType::board_temperature:
          {
            MotorBoardTempMsg *motor_board_temp =
              dynamic_cast<MotorBoardTempMsg*>(msg.get());
            lift_pub_msg_->arm.board_temp_1 = motor_board_temp->board_temp_1_;
            lift_pub_msg_->arm.board_temp_2 = motor_board_temp->board_temp_2_;
          }
          break;
          case AbstractArmMsg::MessageType::voltage:
          {
            MotorVoltageMsg *motor_voltage =
              dynamic_cast<MotorVoltageMsg*>(msg.get());
            lift_pub_msg_->arm.volt_main = motor_voltage->bat_voltage_;
            lift_pub_msg_->arm.volt_12v = motor_voltage->drv_voltage_;
            lift_pub_msg_->arm.volt_5v = motor_voltage->reg_5_voltage_;
          }
          break;
          case AbstractArmMsg::MessageType::motor_mode:
          {
            num_lift_pings_recvd_++;
            MotorModeMsg *motor_mode = dynamic_cast<MotorModeMsg*>(msg.get());
            lift_pub_msg_->arm.motor_control_mode_1 =
              static_cast<std::underlying_type
                          <MotorModeMsg::MotorControlMode>::type>
              (motor_mode->mode_channel_1_);
            lift_pub_msg_->arm.motor_control_mode_2 =
              static_cast<std::underlying_type
                          <MotorModeMsg::MotorControlMode>::type>
              (motor_mode->mode_channel_2_);
          }
          break;
          case AbstractArmMsg::MessageType::motor_flags:
          {
            MotorFlagsMsg *motor_flags =
              dynamic_cast<MotorFlagsMsg*>(msg.get());
            lift_pub_msg_->arm.overheat = motor_flags->overheat_;
            lift_pub_msg_->arm.overvoltage = motor_flags->overvoltage_;
            lift_pub_msg_->arm.undervoltage = motor_flags->undervoltage_;
            lift_pub_msg_->arm.e_short = motor_flags->short_;
            lift_pub_msg_->arm.estop = motor_flags->ESTOP_;
          }
          break;
          case AbstractArmMsg::MessageType::command_accepted:
          {
            lift_pub_msg_->arm.last_cmd_ack = true;
          }
          break;
          case AbstractArmMsg::MessageType::command_rejected:
          {
            lift_pub_msg_->arm.last_cmd_ack = false;
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
    std::future_status status;
    std::vector<std::unique_ptr<AbstractArmMsg>> hand_msgs;
    do {
      try {
        hand_msgs = hand_recv->getAndParseMessage();
      }
      catch (...) {
        std::cerr << "threw\n";
      }
      std::lock_guard<std::mutex> sf_lock_guard(sensor_frame_mutex_);
      for(std::unique_ptr<AbstractArmMsg>& msg : hand_msgs) {
        switch(msg->getType()) {
          case AbstractArmMsg::MessageType::motor_amperage:
          {
            MotorAmpMsg *motor_amp = dynamic_cast<MotorAmpMsg*>(msg.get());
            lift_pub_msg_->hand.amp_1 = motor_amp->motor_amp_1_;
            lift_pub_msg_->hand.amp_2 = motor_amp->motor_amp_2_;
          }
          break;
          case AbstractArmMsg::MessageType::motor_temperature:
          {
            MotorTempMsg *motor_temp = dynamic_cast<MotorTempMsg*>(msg.get());
            lift_pub_msg_->hand.motor_temp_1 = motor_temp->motor_temp_1_;
            lift_pub_msg_->hand.motor_temp_2 = motor_temp->motor_temp_2_;
          }
          break;
          case AbstractArmMsg::MessageType::encoder_position:
          {
            // TODO we've seen this coming out negative... what's up with that?
            MotorEncPosMsg *motor_enc_pos =
              dynamic_cast<MotorEncPosMsg*>(msg.get());
            lift_pub_msg_->hand.encoder_pos_1 = motor_enc_pos->encoder_pos_1_;
            lift_pub_msg_->hand.encoder_pos_2 = motor_enc_pos->encoder_pos_2_;
          }
          break;
          case AbstractArmMsg::MessageType::motor_power:
          {
            // TODO what are the units?  What does this number mean?
            // 25 for power 1 and 0 for power 2 on the arm,
            // -22 and -1 for power on the arm?
            MotorPowerMsg *motor_power =
              dynamic_cast<MotorPowerMsg*>(msg.get());
            lift_pub_msg_->hand.motor_power_1 = motor_power->motor_power_1_;
            lift_pub_msg_->hand.motor_power_2 = motor_power->motor_power_2_;
          }
          break;
          case AbstractArmMsg::MessageType::encoder_velocity:
          {
            MotorEncVelMsg *motor_enc_vel =
              dynamic_cast<MotorEncVelMsg*>(msg.get());
            lift_pub_msg_->hand.encoder_vel_1 = motor_enc_vel->encoder_velocity_1_;
            lift_pub_msg_->hand.encoder_vel_2 = motor_enc_vel->encoder_velocity_2_;
          }
          break;
          case AbstractArmMsg::MessageType::board_temperature:
          {
            MotorBoardTempMsg *motor_board_temp =
              dynamic_cast<MotorBoardTempMsg*>(msg.get());
            lift_pub_msg_->hand.board_temp_1 = motor_board_temp->board_temp_1_;
            lift_pub_msg_->hand.board_temp_2 = motor_board_temp->board_temp_2_;
          }
          break;
          case AbstractArmMsg::MessageType::voltage:
          {
            MotorVoltageMsg *motor_voltage =
              dynamic_cast<MotorVoltageMsg*>(msg.get());
            lift_pub_msg_->hand.volt_main = motor_voltage->bat_voltage_;
            lift_pub_msg_->hand.volt_12v = motor_voltage->drv_voltage_;
            lift_pub_msg_->hand.volt_5v = motor_voltage->reg_5_voltage_;
          }
          break;
          case AbstractArmMsg::MessageType::motor_mode:
          {
            num_hand_pings_recvd_++;
            MotorModeMsg *motor_mode = dynamic_cast<MotorModeMsg*>(msg.get());
            lift_pub_msg_->hand.motor_control_mode_1 =
              static_cast<std::underlying_type
                          <MotorModeMsg::MotorControlMode>::type>
              (motor_mode->mode_channel_1_);
            lift_pub_msg_->hand.motor_control_mode_2 =
              static_cast<std::underlying_type
                          <MotorModeMsg::MotorControlMode>::type>
              (motor_mode->mode_channel_2_);
          }
          break;
          case AbstractArmMsg::MessageType::motor_flags:
          {
            MotorFlagsMsg *motor_flags =
              dynamic_cast<MotorFlagsMsg*>(msg.get());
            lift_pub_msg_->hand.overheat = motor_flags->overheat_;
            lift_pub_msg_->hand.overvoltage = motor_flags->overvoltage_;
            lift_pub_msg_->hand.undervoltage = motor_flags->undervoltage_;
            lift_pub_msg_->hand.e_short = motor_flags->short_;
            lift_pub_msg_->hand.estop = motor_flags->ESTOP_;
          }
          break;
          case AbstractArmMsg::MessageType::command_accepted:
          {
            lift_pub_msg_->hand.last_cmd_ack = true;
          }
          break;
          case AbstractArmMsg::MessageType::command_rejected:
          {
            lift_pub_msg_->hand.last_cmd_ack = false;
          }
          break;
        }
      }
      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void pubTimerCallback()
  {
    // to set timestamp
    rcutils_time_point_value_t now;
    if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
      std::cerr << "unable to access time\n";
      return;
    }

    std::lock_guard<std::mutex> sf_lock_guard(sensor_frame_mutex_);

    lift_pub_msg_->header.stamp.sec = RCL_NS_TO_S(now);
    lift_pub_msg_->header.stamp.nanosec =
      now - RCL_S_TO_NS(lift_pub_msg_->header.stamp.sec);

    lift_pub_->publish(lift_pub_msg_);
  }

  void pingTimerCallback()
  {
    // NOTE:  this implementation causes accepting_commands_ to be false
    // for the first 500ms of operation
    lift_cmd_->ping();
    hand_cmd_->ping();
    num_pings_sent_++;

    if (num_pings_sent_ < kPingsPerWatchdogInterval) {
      return;
    }

//    std::cerr << "num_lift_pings_recvd_ = " << num_lift_pings_recvd_ <<  " num_hand_pings_recvd_ = " << num_hand_pings_recvd_ << "\n";
    if (num_lift_pings_recvd_ < kMinPingsExpected
        || num_hand_pings_recvd_ < kMinPingsExpected) {
//      std::cerr << "haven't heard from the robot... no longer accepting commands\n";
      accepting_commands_ = false;
    } else {
//      std::cerr << "have heard from the robot... go ahead and command it\n";
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
  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  std::thread                  arm_recv_thread_;
  std::thread                  hand_recv_thread_;
  std::promise<void>           exit_signal_;
  std::shared_future<void>     future_;
  uint32_t                     num_pings_sent_;
  std::atomic<uint32_t>        num_lift_pings_recvd_;
  std::atomic<uint32_t>        num_hand_pings_recvd_;
  std::atomic<bool>            accepting_commands_;
  std::mutex                   sensor_frame_mutex_;
  rclcpp::Publisher<jaguar4x4_arm_msgs::msg::Lift>::SharedPtr lift_pub_;
  std::shared_ptr<jaguar4x4_arm_msgs::msg::Lift> lift_pub_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jaguar4x4Arm>("192.168.0.63", 10001, 10002));
  rclcpp::shutdown();
  return 0;
}
