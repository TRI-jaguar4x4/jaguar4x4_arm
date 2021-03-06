// Copyright 2018 Toyota Research Institute.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "jaguar4x4_arm_msgs/msg/lift.hpp"
#include "jaguar4x4_arm/ArmCommand.h"
#include "jaguar4x4_arm/ArmReceive.h"
#include "jaguar4x4_arm/HandCommand.h"

using namespace std::chrono_literals;

class Jaguar4x4Arm : public rclcpp::Node
{
public:
  Jaguar4x4Arm(const std::string& ip, uint16_t arm_port, uint16_t hand_port)
    : Node("jaguar4x4arm")
  {
    auto arm_board_comm = std::make_shared<Communication>();
    arm_board_comm->connect(ip, arm_port);

    auto hand_board_comm = std::make_shared<Communication>();
    hand_board_comm->connect(ip, hand_port);

    lift_cmd_ = std::make_unique<ArmCommand>(arm_board_comm);
    lift_rcv_ = std::make_unique<ArmReceive>(arm_board_comm);

    hand_cmd_ = std::make_unique<HandCommand>(hand_board_comm);
    hand_rcv_ = std::make_unique<ArmReceive>(hand_board_comm);

    rmw_qos_profile_t tf_qos_profile = rmw_qos_profile_default;
    tf_qos_profile.depth = 100;
    tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf",
                                                               tf_qos_profile);

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

    lift_cmd_->configure(kMotorDriverBufferServiceIntervalMS);

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

    hand_cmd_->configure(kMotorDriverBufferServiceIntervalMS);
    hand_cmd_->eStop();

    arm_recv_thread_ = std::thread(&Jaguar4x4Arm::armRecvThread, this,
                                   future_);
    hand_recv_thread_ = std::thread(&Jaguar4x4Arm::handRecvThread, this,
                                    future_);

    // We use a separate callback group for the joystick callback so that a
    // thread can service this separate from all of the other callbacks,
    // including services.
    joy_cb_grp_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        std::bind(&Jaguar4x4Arm::joyCallback, this, std::placeholders::_1),
        z_position_qos_profile,
        joy_cb_grp_);

    arm_joint_zero_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "arm_joint_zero",
        std::bind(&Jaguar4x4Arm::armJointZero, this,
                  std::placeholders::_1, std::placeholders::_2));
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
        RCLCPP_INFO(get_logger(), "getAndParseMessage threw exception");
      }

      std::lock_guard<std::mutex> sf_lock_guard(sensor_frame_mutex_);
      if (arm_msg) {
        abstractMotorToROS(arm_msg.get(), &lift_pub_msg_->arm);

        num_lift_data_recvd_++;

        if (arm_msg->getType() == AbstractMotorMsg::MessageType::encoder_position) {
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

      arm_msg.reset();

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
        RCLCPP_INFO(get_logger(), "getAndParseMessage threw exception");
      }
      std::lock_guard<std::mutex> sf_lock_guard(sensor_frame_mutex_);
      if (hand_msg) {
        abstractMotorToROS(hand_msg.get(), &lift_pub_msg_->hand);

        num_hand_data_recvd_++;

        if (hand_msg->getType() == AbstractMotorMsg::MessageType::encoder_position) {
          // We save off the encoder position so that resuming from eStop
          // drives to the current position, not whatever was in the command
          // buffer.
          MotorEncPosMsg *motor_enc_pos = dynamic_cast<MotorEncPosMsg*>(hand_msg.get());
          current_hand_enc_pos_wrist_ = motor_enc_pos->encoder_pos_1_;
        }
      }

      hand_msg.reset();

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

        // publish associated tf transform
        // currently this is a static transform for the exact place we
        // put the arm for the detection problem
        auto lift_tf_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
        lift_tf_msg->header.frame_id = "base_link";
        lift_tf_msg->child_frame_id = "camera";
        // Stuff and publish /tf
        lift_tf_msg->header.stamp.sec = RCL_NS_TO_S(now);
        lift_tf_msg->header.stamp.nanosec =
          now - RCL_S_TO_NS(lift_tf_msg->header.stamp.sec);
        lift_tf_msg->transform.translation.x = 0.585;
        lift_tf_msg->transform.translation.y = 0.0;
        lift_tf_msg->transform.translation.z = 0.286;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        lift_tf_msg->transform.rotation.x = q.x();
        lift_tf_msg->transform.rotation.y = q.y();
        lift_tf_msg->transform.rotation.z = q.z();
        lift_tf_msg->transform.rotation.w = q.w();

        tf2_msgs::msg::TFMessage tf_msg;
        tf_msg.transforms.push_back(*lift_tf_msg);
        tf_pub_->publish(tf_msg);
      } else {
        RCLCPP_INFO(get_logger(), "Unable to access time, skipping tf publish");
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

    // Also note that we only process the eStop/Resume button by itself (i.e.
    // if it is used in combination with other buttons, the other buttons are
    // ignored).  For all other buttons we allow multiple to be pressed
    // simultaneously.

    if (msg->buttons.size() < 12) {
      // When coming out of power-up, the joystick node sometimes publishes
      // bogus default messages that don't have enough buttons.  Counteract
      // that by only allowing this joy callback if there are enough buttons.
      return;
    }

    if (msg->buttons[11]) {
      // If we see eStop, set our eStopped_ atomic variable to true.  This will
      // ensure that the pingThread does not start accepting commands while we
      // are eStopped.
      e_stopped_ = true;
      accepting_commands_ = false;

      // eStop the robot, and set the movement to 0.  The latter is so that the
      // robot won't continue moving at its last commanded power when we resume.
      lift_cmd_->eStop();
      hand_cmd_->eStop();

      RCLCPP_INFO(get_logger(), "ESTOP");
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
      e_stopped_ = false;
    } else {
      if (msg->buttons[3]) {
        // Lower Joint UP
        if (!msg->buttons[5] || !accepting_commands_) {
          return;
        }
        lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::lower_arm, relative_pos_to_move_);
      }
      if (msg->buttons[1]) {
        // Lower Joint DOWN
        if (!msg->buttons[5] || !accepting_commands_) {
          return;
        }
        lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::lower_arm, -relative_pos_to_move_);
      }
      if (msg->buttons[0]) {
        // Upper Joint UP
        if (!msg->buttons[5] || !accepting_commands_) {
          return;
        }
        lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::upper_arm, relative_pos_to_move_);
      }
      if (msg->buttons[2]) {
        // Upper Joint DOWN
        if (!msg->buttons[5] || !accepting_commands_) {
          return;
        }
        lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::upper_arm, -relative_pos_to_move_);
      }
      if (msg->buttons[6]) {
        // Increase amount
        relative_pos_to_move_ += 1;
        if (relative_pos_to_move_ > 50) {
          relative_pos_to_move_ = 50;
        }
        RCLCPP_INFO(get_logger(), "Now moving arm by %d", relative_pos_to_move_);
      }
      if (msg->buttons[7]) {
        // Decrease amount
        relative_pos_to_move_ -= 1;
        if (relative_pos_to_move_ < 1) {
          relative_pos_to_move_ = 1;
        }
        RCLCPP_INFO(get_logger(), "Now moving arm by %d", relative_pos_to_move_);
      }
    }
  }

  void pingThread(std::shared_future<void> local_future)
  {
    std::future_status status;

    // The idea behind the ping thread is to make sure that we are talking to
    // the robot; if not, we'll stop accepting commands at this layer to make
    // sure nothing wacky happens.  Any time we get any data from the arm or
    // hand (in armRecvThread() or handRecvThread()), we consider that a "ping",
    // and increment the counter accordingly.  We unconditionally send a ping
    // every interval to ensure that the robot should have responded to
    // something, though in the current usage this is redundant.  After we
    // have waited for kWatchdogIntervalMS, we check to make sure we got at
    // least the number of data that we pinged; if not, we assume we can't
    // talk to the robot right now and stop accepting commands.

    std::chrono::time_point<std::chrono::system_clock> last_watchdog_check = std::chrono::system_clock::now();

    do {
      lift_cmd_->ping();
      hand_cmd_->ping();

      std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
      auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_watchdog_check);
      if (diff_ms.count() > kWatchdogIntervalMS) {
        if (!e_stopped_) {
          if (num_lift_data_recvd_ < kMinPingsExpected
              || num_hand_data_recvd_ < kMinPingsExpected) {
            RCLCPP_INFO(get_logger(), "Stopped accepting commands");
            accepting_commands_ = false;
            // If the arm zero service is running, we need to notify it to wake
            // up and deal with the fact that we stopped accepting commands.
            wakeup_reason_ = ZeroServiceWakeupReason::STOPPED_ACCEPTING_COMMANDS;
            encoder_pos_cv_.notify_one();
          } else {
            if (!accepting_commands_) {
              RCLCPP_INFO(get_logger(), "accepting commands");
            }
            accepting_commands_ = true;
          }
        }
        last_watchdog_check = now;
        num_lift_data_recvd_ = 0;
        num_hand_data_recvd_ = 0;
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
      RCLCPP_INFO(get_logger(), "Couldn't acquire mutex to wait for motor stop");
      return std::cv_status::timeout;
    }

    return arm_zero_service_wait_for_motor_stop_cv_.wait_for(stop_lk,
                                                             std::chrono::milliseconds(2000));
  }

  std::string calibrateLowerArmToCradle()
  {
    RCLCPP_INFO(get_logger(), "Setting motor mode to speed");
    setArmSpeedModeDefaults(ArmJoint::lower_arm);

    static const int NUM_SAME_ENC_COUNTS = 10;
    int num_enc_same = 0;
    std::string error;

    RCLCPP_INFO(get_logger(), "Lowering arm");

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
        RCLCPP_INFO(get_logger(), "No data in %u ms, giving up", kZeroNoDataIntervalMS);
        error += "No data before timeout ";
        break;
      }

      if (!accepting_commands_) {
        error += "Stopped accepting commands (eStopped?)";
        break;
      }

      if (wakeup_reason_ == ZeroServiceWakeupReason::STOPPED_ACCEPTING_COMMANDS) {
        RCLCPP_INFO(get_logger(), "Stopped talking to robot; aborting zero service");
        error += "Stopped talking to robot ";
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
      RCLCPP_INFO(get_logger(), "Timed out waiting for stop at end");
      error += "Timed out waiting for stop at end ";
      num_enc_same = 0;
    }

    if (num_enc_same == NUM_SAME_ENC_COUNTS) {
      RCLCPP_INFO(get_logger(), "Zero encoding position: %ld", last_enc_count);
    } else {
      error += "Bad calibration; arm was moving ";
    }

    setArmPositionModeDefaults(ArmJoint::lower_arm);

    return error;
  }

  void armJointZero(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    RCLCPP_INFO(get_logger(), "Called arm joint zero");

    if (!accepting_commands_) {
      response->success = false;
      response->message = "Not accepting commands (eStopped?)";
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
  const uint32_t kMotorDriverBufferServiceIntervalMS = 20;
  const uint32_t kWatchdogIntervalMS = 200;
  static constexpr double kPingRecvPercentage = 0.8;

  const uint32_t kPubTimerIntervalMS = 100;

  // During the arm zero service, if we didn't get any data in this amount of
  // time, just give up and assume we aren't currently talking to the robot.
  const uint32_t kZeroNoDataIntervalMS = 500;

  // Constants calculated based on the tunable parameters above
  const uint32_t kPingTimerIntervalMS = kMotorDriverBufferServiceIntervalMS * 2;
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
  std::atomic<uint32_t>                                            num_lift_data_recvd_{0};
  std::atomic<uint32_t>                                            num_hand_data_recvd_{0};
  std::atomic<bool>                                                accepting_commands_{false};
  std::mutex                                                       sensor_frame_mutex_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr           tf_pub_;
  rclcpp::Publisher<jaguar4x4_arm_msgs::msg::Lift>::SharedPtr      lift_pub_;
  std::shared_ptr<jaguar4x4_arm_msgs::msg::Lift>                   lift_pub_msg_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr           joy_sub_;
  std::atomic<bool>                                                e_stopped_{true};
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
  rclcpp::callback_group::CallbackGroup::SharedPtr                 joy_cb_grp_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto arm = std::make_shared<Jaguar4x4Arm>("192.168.0.63", 10001, 10002);
  executor.add_node(arm);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
