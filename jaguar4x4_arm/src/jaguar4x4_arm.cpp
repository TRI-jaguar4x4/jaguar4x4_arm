// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <jaguar4x4_comms/Communication.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "jaguar4x4_arm_msgs/msg/lift.hpp"
#include "jaguar4x4_arm/ArmCommand.h"
#include "jaguar4x4_arm/ArmReceive.h"
#include "jaguar4x4_arm/HandCommand.h"

using namespace std::chrono_literals;

class Jaguar4x4Arm : public rclcpp::Node
{
public:
  Jaguar4x4Arm(const std::string& ip, uint16_t arm_port, uint16_t hand_port)
    : Node("jaguar4x4Arm")
  {
    auto arm_board_comm = std::make_shared<Communication>();
    arm_board_comm->connect(ip, arm_port);

    auto hand_board_comm = std::make_shared<Communication>();
    hand_board_comm->connect(ip, hand_port);

    lift_cmd_ = std::make_unique<ArmCommand>(arm_board_comm);
    lift_rcv_ = std::make_unique<ArmReceive>(arm_board_comm);

    hand_cmd_ = std::make_unique<HandCommand>(hand_board_comm);
    hand_rcv_ = std::make_unique<ArmReceive>(hand_board_comm);

    rmw_qos_profile_t z_position_qos_profile = rmw_qos_profile_sensor_data;
    z_position_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    z_position_qos_profile.depth = 50;
    z_position_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    z_position_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    future_ = exit_signal_.get_future();

    lift_pub_msg_ = std::make_shared<jaguar4x4_arm_msgs::msg::Lift>();
    lift_pub_ = this->create_publisher<jaguar4x4_arm_msgs::msg::Lift>("lift_info");
    lift_pub_thread_ = std::thread(&Jaguar4x4Arm::liftPubThread, this, future_);

    ping_thread_ = std::thread(&Jaguar4x4Arm::pingThread, this, future_);

    lift_cmd_->configure(kPubTimerIntervalMS);

    // Configure the lower joint to the "defaults" (that is, the values it has
    // stored in EEPROM, what it uses when it comes out of reset).  For the
    // most part this is redundant, but gets the joint into a default mode if
    // we forgot to reset it after playing around with it.
    lift_cmd_->setMotorPID(ArmJoint::lower_arm, 30, 2, 0);
    // We sent ~MAC 1 down to the motor to find out that the default MAC
    // in closed_loop_count_position is 20000
    lift_cmd_->setMotorAcceleration(ArmJoint::lower_arm, 20000);
    // We sent ~MXRPM 1 down to the motor to find out that the default MXRPM
    // in closed_loop_count_position is 1000.
    lift_cmd_->setMotorMaxRPM(ArmJoint::lower_arm, 1000);
    lift_cmd_->setMotorMode(ArmJoint::lower_arm, ArmMotorMode::closed_loop_count_position);
    // We sent ~MVEL 1 down to the motor to find out that the default in
    // closed_loop_count_position is 200.
    lift_cmd_->setArmPositionControlSpeed(ArmJoint::lower_arm, 200);

    // In theory we would eStop here to be sure the arm is stopped, but the arm
    // seems to go into a passive mode when eStopped, meaning the arm can fall
    // because of its own weight.  Instead, we set the software eStopped_ to
    // true, but don't actually send an eStop, meaning that this code won't
    // accept commands but the robot isn't necessarily in eStop.
    // TODO: update the motor eStop state to say "software", "hardware", or "off"

    hand_cmd_->configure(kPubTimerIntervalMS);
    hand_cmd_->eStop();

    arm_recv_thread_ = std::thread(&Jaguar4x4Arm::armRecvThread, this,
                                   future_);
    hand_recv_thread_ = std::thread(&Jaguar4x4Arm::handRecvThread, this,
                                    future_);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy",
                                                                std::bind(&Jaguar4x4Arm::joyCallback, this, std::placeholders::_1),
                                                                z_position_qos_profile);

    arm_joint_zero_srv_ = this->create_service<std_srvs::srv::Trigger>("arm_joint_zero",
                                                                       std::bind(&Jaguar4x4Arm::armJointZero, this, std::placeholders::_1, std::placeholders::_2));
  }

  ~Jaguar4x4Arm()
  {
    exit_signal_.set_value();
    arm_recv_thread_.join();
    hand_recv_thread_.join();
    ping_thread_.join();
    lift_pub_thread_.join();
  }

private:
  void armRecvThread(std::shared_future<void> local_future)
  {
    std::future_status status;
    std::unique_ptr<AbstractMotorMsg> arm_msg;

    do {
      try {
        arm_msg = lift_rcv_->getAndParseMessage();
      }
      catch (...) {
        std::cerr << "threw\n";
      }

      std::lock_guard<std::mutex> sf_lock_guard(sensor_frame_mutex_);
      if (arm_msg) {
        abstractMotorToROS(arm_msg.get(), &lift_pub_msg_->arm);

        if (arm_msg->getType() == AbstractMotorMsg::MessageType::motor_mode) {
          num_lift_pings_recvd_++;
        } else if (arm_msg->getType() == AbstractMotorMsg::MessageType::encoder_position) {
          // We save off the encoder position so that resuming from eStop
          // drives to the current position, not whatever was in the command
          // buffer.
          MotorEncPosMsg *motor_enc_pos = dynamic_cast<MotorEncPosMsg*>(arm_msg.get());
          current_arm_enc_pos_lower_ = motor_enc_pos->encoder_pos_1_;
          current_arm_enc_pos_upper_ = motor_enc_pos->encoder_pos_2_;

          if (arm_zero_service_running_) {
            wakeup_reason_ = ZeroServiceWakeupReason::NEW_ENC_COUNT_AVAILABLE;
            encoder_pos_cv_.notify_one();
          }
        } else if (arm_msg->getType() == AbstractMotorMsg::MessageType::motor_power) {
          MotorPowerMsg *motor_power =
            dynamic_cast<MotorPowerMsg*>(arm_msg.get());

          if (arm_zero_service_running_) {
            if (std::abs(motor_power->motor_power_1_) < 5) {
              arm_zero_service_wait_for_motor_stop_cv_.notify_one();
            }
          }
        }
      }
      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void handRecvThread(std::shared_future<void> local_future)
  {
    std::future_status status;
    std::unique_ptr<AbstractMotorMsg> hand_msg;

    do {
      try {
        hand_msg = hand_rcv_->getAndParseMessage();
      }
      catch (...) {
        std::cerr << "threw\n";
      }
      std::lock_guard<std::mutex> sf_lock_guard(sensor_frame_mutex_);
      if (hand_msg) {
        abstractMotorToROS(hand_msg.get(), &lift_pub_msg_->hand);

        if (hand_msg->getType() == AbstractMotorMsg::MessageType::motor_mode) {
          num_hand_pings_recvd_++;
        } else if (hand_msg->getType() == AbstractMotorMsg::MessageType::encoder_position) {
          // We save off the encoder position so that resuming from eStop
          // drives to the current position, not whatever was in the command
          // buffer.
          MotorEncPosMsg *motor_enc_pos = dynamic_cast<MotorEncPosMsg*>(hand_msg.get());
          current_hand_enc_pos_wrist_ = motor_enc_pos->encoder_pos_1_;
        }
      }
      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void liftPubThread(std::shared_future<void> local_future)
  {
    std::future_status status;

    do {
      // to set timestamp
      rcutils_time_point_value_t now;
      if (rcutils_system_time_now(&now) == RCUTILS_RET_OK) {
        auto local_pub_msg = std::make_unique<jaguar4x4_arm_msgs::msg::Lift>();

        local_pub_msg->header.stamp.sec = RCL_NS_TO_S(now);
        local_pub_msg->header.stamp.nanosec =
          now - RCL_S_TO_NS(local_pub_msg->header.stamp.sec);

        {
          std::lock_guard<std::mutex> sf_lock_guard(sensor_frame_mutex_);
          // Both local_pub_msg->motor* and lift_pub_msg_->motor* are of type
          // jaguar4x4_comms_msgs::msg::MotorBoard, which is a class generated by the
          // ROS2 generator.  Those classes do not have copy constructors or =operator
          // implemented, so we copy by hand here.
          copyMotorBoardMessage(&(local_pub_msg->arm), &(lift_pub_msg_->arm));
          copyMotorBoardMessage(&(local_pub_msg->hand), &(lift_pub_msg_->hand));
        }

        lift_pub_->publish(local_pub_msg);
      } else {
        std::cerr << "unable to access time\n";
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(kPubTimerIntervalMS));

      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Button 11 is the Right Analog Stick press on the Logitech joystick, which
    // we use as "eStop the robot".  Button 10 is the Left Analog Stick press on
    // the joystick, which we use as "resume the robot".  We start out with the
    // robot in eStop, so you always must resume it to start using the robot.

    if (msg->buttons[11]) {
      // If we see eStop, set our eStopped_ atomic variable to true.  This will
      // ensure that the pingThread does not start accepting commands while we
      // are eStopped.
      eStopped_ = true;
      accepting_commands_ = false;

      // eStop the robot, and set the movement to 0.  The latter is so that the
      // robot won't continue moving at its last commanded power when we resume.
      lift_cmd_->eStop();
      hand_cmd_->eStop();

      std::cerr << "ESTOP" << std::endl;
    } else if (msg->buttons[10]) {
      // Resume the arm.  Since the motors are in position control, we drive it
      // to whatever the current arm position is so that it doesn't continue to
      // move.  We also set eStopped to false, and then rely on the pingThread
      // to set accepting_commands to true as appropriate.
      lift_cmd_->moveArmToAbsoluteEncoderPos(ArmJoint::lower_arm, current_arm_enc_pos_lower_);
      lift_cmd_->moveArmToAbsoluteEncoderPos(ArmJoint::upper_arm, current_arm_enc_pos_upper_);

      // TODO: Drive the hand back to the current position to stop motion.

      hand_cmd_->resume();
      lift_cmd_->resume();
      eStopped_ = false;
    } else if (msg->buttons[3]) {
      // Lower Joint UP
      if (!msg->buttons[5] || !accepting_commands_) {
        return;
      }
      lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::lower_arm, relative_pos_to_move_);
    } else if (msg->buttons[1]) {
      // Lower Joint DOWN
      if (!msg->buttons[5] || !accepting_commands_) {
        return;
      }
      lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::lower_arm, -relative_pos_to_move_);
    } else if (msg->buttons[0]) {
      // Upper Joint UP
      if (!msg->buttons[5] || !accepting_commands_) {
        return;
      }
      lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::upper_arm, relative_pos_to_move_);
    } else if (msg->buttons[2]) {
      // Upper Joint DOWN
      if (!msg->buttons[5] || !accepting_commands_) {
        return;
      }
      lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::upper_arm, -relative_pos_to_move_);
    } else if (msg->buttons[6]) {
      // Increase amount
      relative_pos_to_move_ += 1;
      if (relative_pos_to_move_ > 50) {
        relative_pos_to_move_ = 50;
      }
      std::cerr << "Now moving arm by " << relative_pos_to_move_ << std::endl;
    } else if (msg->buttons[7]) {
      // Decrease amount
      relative_pos_to_move_ -= 1;
      if (relative_pos_to_move_ < 1) {
        relative_pos_to_move_ = 1;
      }
      std::cerr << "Now moving arm by " << relative_pos_to_move_ << std::endl;
    }
  }

  void pingThread(std::shared_future<void> local_future)
  {
    std::future_status status;
    uint32_t num_pings_sent{0};

    do {
      lift_cmd_->ping();
      hand_cmd_->ping();
      num_pings_sent++;

      if (num_pings_sent >= kPingsPerWatchdogInterval) {

        if (!eStopped_) {
          if (num_lift_pings_recvd_ < kMinPingsExpected
              || num_hand_pings_recvd_ < kMinPingsExpected) {
            std::cerr << "Stopped accepting commands" << std::endl;
            accepting_commands_ = false;
            // If the arm zero service is running, we need to notify it to wake
            // up and deal with the fact that we stopped accepting commands.
            wakeup_reason_ = ZeroServiceWakeupReason::STOPPED_ACCEPTING_COMMANDS;
            encoder_pos_cv_.notify_one();
          } else {
            if (!accepting_commands_) {
              std::cerr << "accepting commands" << std::endl;
            }
            accepting_commands_ = true;
          }
        }
        num_lift_pings_recvd_ = 0;
        num_hand_pings_recvd_ = 0;
        num_pings_sent = 0;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(kPingTimerIntervalMS));

      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  void setArmPositionModeDefaults(ArmJoint joint)
  {
    // We sent ~KP, ~KI, ~KD down to the motor to find out that the default
    // PID tuning is 30, 2, 0 in closed_loop_count_position.
    lift_cmd_->setMotorPID(joint, 30, 2, 0);
    // We sent ~MAC 1 down to the motor to find out that the default MAC
    // in closed_loop_count_position is 20000
    lift_cmd_->setMotorAcceleration(joint, 20000);
    // We sent ~MXRPM 1 down to the motor to find out that the default MXRPM
    // in closed_loop_count_position is 1000.
    lift_cmd_->setMotorMaxRPM(joint, 1000);
    lift_cmd_->setMotorMode(joint, ArmMotorMode::closed_loop_count_position);
    // We sent ~MVEL 1 down to the motor to find out that the default in
    // closed_loop_count_position is 200.
    lift_cmd_->setArmPositionControlSpeed(joint, 200);
  }

  void setArmSpeedModeDefaults(ArmJoint joint)
  {
    // These are tunings that we found we needed to move the lower arm
    // from any position to do the zero check.
    lift_cmd_->setMotorMode(joint, ArmMotorMode::closed_loop_speed);
    lift_cmd_->setMotorMaxRPM(joint, 4000);
    lift_cmd_->setMotorAcceleration(joint, 120000);
    lift_cmd_->setMotorPID(joint, 50, 0, 0);
  }

  std::cv_status stopJoint(ArmJoint joint)
  {
    // To truly stop the arm, we have to first put it into open loop mode and
    // then make sure the speed is 0.
    lift_cmd_->setMotorMode(joint, ArmMotorMode::open_loop);
    lift_cmd_->moveArmAtSpeed(joint, 0);

    std::unique_lock<std::timed_mutex> stop_lk(arm_zero_service_wait_for_motor_stop_mutex_,
                                               std::defer_lock);
    bool got_lock = stop_lk.try_lock_for(std::chrono::milliseconds(100));
    if (!got_lock) {
      std::cerr << "Couldn't acquire stop mutex!" << std::endl;
      return std::cv_status::timeout;
    }

    return arm_zero_service_wait_for_motor_stop_cv_.wait_for(stop_lk,
                                                             std::chrono::milliseconds(10000));
  }

  std::string calibrateLowerArmToCradle()
  {
    std::cerr << "Setting mode to speed" << std::endl;
    setArmSpeedModeDefaults(ArmJoint::lower_arm);

    static const int NUM_SAME_ENC_COUNTS = 10;
    int num_enc_same = 0;
    std::string error;

    std::cerr << "Lowering arm" << std::endl;

    lift_cmd_->moveArmAtSpeed(ArmJoint::lower_arm, 150);

    // Sleep for a little while to let the arm start moving.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Now wait for updated encoder position counts.  If we go a number of
    // times where the count is the same, we assume we found the cradle
    // and return success.
    int64_t last_enc_count = current_arm_enc_pos_lower_;

    std::unique_lock<std::timed_mutex> lk(encoder_pos_mutex_, std::defer_lock);
    bool got_lock = lk.try_lock_for(std::chrono::milliseconds(100));
    while (got_lock && num_enc_same < NUM_SAME_ENC_COUNTS) {
      std::cv_status cv_status = encoder_pos_cv_.wait_for(lk,
                                                          std::chrono::milliseconds(kZeroNoDataIntervalMS));
      if (cv_status == std::cv_status::timeout) {
        std::cerr << "No data in " << kZeroNoDataIntervalMS << "ms, giving up" << std::endl;
        error = "No data before timeout";
        break;
      }

      if (wakeup_reason_ == ZeroServiceWakeupReason::STOPPED_ACCEPTING_COMMANDS) {
        std::cerr << "Stopped talking to robot; aborting zero service" << std::endl;
        error = "Stopped talking to robot";
        break;
      }

      if (current_arm_enc_pos_lower_ > (last_enc_count - 10) && current_arm_enc_pos_lower_ < (last_enc_count + 10)) {
        num_enc_same++;
      } else {
        num_enc_same = 0;
      }
      last_enc_count = current_arm_enc_pos_lower_;
    }

    std::cv_status end_stop_cv_status = stopJoint(ArmJoint::lower_arm);
    if (end_stop_cv_status == std::cv_status::timeout) {
      std::cerr << "Timed out waiting for stop at end" << std::endl;
      error = "Timed out waiting for stop at end";
      num_enc_same = 0;
    }

    if (num_enc_same == NUM_SAME_ENC_COUNTS) {
      std::cerr << "Zero encoder position: " << last_enc_count << std::endl;
    } else if (error.empty()) {
      error = "Bad calibration; arm was moving";
    }

    setArmPositionModeDefaults(ArmJoint::lower_arm);

    return error;
  }

  void armJointZero(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    std::cerr << "Called arm Joint zero" << std::endl;

    if (eStopped_) {
      response->success = false;
      response->message = "eStopped";
      return;
    }

    arm_zero_service_running_ = true;

    std::cv_status stop_cv_status = stopJoint(ArmJoint::lower_arm);
    if (stop_cv_status == std::cv_status::timeout) {
      response->success = false;
      response->message = "Lower joint didn't stop";
      setArmPositionModeDefaults(ArmJoint::lower_arm);
      arm_zero_service_running_ = false;
      return;
    }

    std::string error = calibrateLowerArmToCradle();

    arm_zero_service_running_ = false;

    if (error.empty()) {
      response->success = true;
      response->message = "Success";
    } else {
      response->success = false;
      response->message = error;
    }
  }

  // Tunable parameters
  const uint32_t kPubTimerIntervalMS = 100;
  const uint32_t kPingTimerIntervalMS = 100;
  const uint32_t kWatchdogIntervalMS = 500;
  // During the arm zero service, if we didn't get any data in this amount of
  // time, just give up and assume we aren't currently talking to the robot.
  const uint32_t kZeroNoDataIntervalMS = 500;
  static constexpr double kPingRecvPercentage = 0.6;

  // Constants calculated based on the tunable parameters above
  const uint32_t kPingsPerWatchdogInterval = kWatchdogIntervalMS / kPingTimerIntervalMS;
  const uint32_t kMinPingsExpected = kPingsPerWatchdogInterval * kPingRecvPercentage;

  std::unique_ptr<ArmCommand>                                      lift_cmd_;
  std::unique_ptr<ArmReceive>                                      lift_rcv_;
  std::unique_ptr<HandCommand>                                     hand_cmd_;
  std::unique_ptr<ArmReceive>                                      hand_rcv_;
  int64_t                                                          last_stamp_ = 0;
  std::thread                                                      ping_thread_;
  std::thread                                                      lift_pub_thread_;
  std::thread                                                      arm_recv_thread_;
  std::thread                                                      hand_recv_thread_;
  std::promise<void>                                               exit_signal_;
  std::shared_future<void>                                         future_;
  std::atomic<uint32_t>                                            num_lift_pings_recvd_{0};
  std::atomic<uint32_t>                                            num_hand_pings_recvd_{0};
  std::atomic<bool>                                                accepting_commands_{false};
  std::mutex                                                       sensor_frame_mutex_;
  rclcpp::Publisher<jaguar4x4_arm_msgs::msg::Lift>::SharedPtr      lift_pub_;
  std::shared_ptr<jaguar4x4_arm_msgs::msg::Lift>                   lift_pub_msg_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr           joy_sub_;
  std::atomic<bool>                                                eStopped_{true};
  std::atomic<int64_t>                                             current_arm_enc_pos_lower_;
  std::atomic<int64_t>                                             current_arm_enc_pos_upper_;
  std::atomic<int64_t>                                             current_hand_enc_pos_wrist_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr               arm_joint_zero_srv_;
  std::timed_mutex                                                 encoder_pos_mutex_;
  std::condition_variable_any                                      encoder_pos_cv_;
  std::atomic<bool>                                                arm_zero_service_running_{false};
  enum class ZeroServiceWakeupReason
  {
    NEW_ENC_COUNT_AVAILABLE,
    STOPPED_ACCEPTING_COMMANDS,
  };
  std::atomic<ZeroServiceWakeupReason>                             wakeup_reason_;
  std::timed_mutex                                                 arm_zero_service_wait_for_motor_stop_mutex_;
  std::condition_variable_any                                      arm_zero_service_wait_for_motor_stop_cv_;
  int                                                              relative_pos_to_move_{10};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jaguar4x4Arm>("192.168.0.63", 10001, 10002));
  rclcpp::shutdown();
  return 0;
}
