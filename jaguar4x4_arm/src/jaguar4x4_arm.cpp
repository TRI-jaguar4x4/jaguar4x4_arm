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

    z_pos_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "z_position", std::bind(&Jaguar4x4Arm::liftCallback,
                                this, std::placeholders::_1),
        z_position_qos_profile);

    future_ = exit_signal_.get_future();

    lift_pub_msg_ = std::make_shared<jaguar4x4_arm_msgs::msg::Lift>();
    lift_pub_ = this->create_publisher<jaguar4x4_arm_msgs::msg::Lift>("lift_info");
    lift_pub_thread_ = std::thread(&Jaguar4x4Arm::liftPubThread, this, future_);

    ping_thread_ = std::thread(&Jaguar4x4Arm::pingThread, this, future_);

    lift_cmd_->configure(kPubTimerIntervalMS);
    lift_cmd_->setMotorMode(ArmJoint::lower_arm, ArmMotorMode::closed_loop_count_position);
    lift_cmd_->setMotorMode(ArmJoint::upper_arm, ArmMotorMode::closed_loop_count_position);
    lift_cmd_->setMotorMaxRPM(ArmJoint::lower_arm);
    lift_cmd_->getMotorMaxRPM(ArmJoint::lower_arm);
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
    lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::lower_arm, msg->pose.position.z);
    //lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::upper_arm, msg->pose.position.z);

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

          std::chrono::duration<double, std::milli> diff_ms;
          auto now = std::chrono::high_resolution_clock::now();
          diff_ms = now - last_time;
          //std::cerr << "Diff: " << diff_ms.count() << std::endl;
          last_time = now;

          if (arm_zero_service_running_) {
            //std::unique_lock<std::mutex> lk(encoder_pos_mutex_);
            std::cerr << "Notifying" << std::endl;
            encoder_pos_cv_.notify_one();
          }
        }
      }
      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> last_time;

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

    if (!msg->buttons[10] && !msg->buttons[11]) {
      return;
    }

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
    } else {
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

    lift_cmd_->setMotorMode(ArmJoint::lower_arm, ArmMotorMode::closed_loop_speed);

    std::cerr << "Lowering arm" << std::endl;

    // TODO: is 60 good enough for all positions of the arm?
    //lift_cmd_->moveArmAtSpeed(ArmJoint::lower_arm, 0);
    lift_cmd_->moveArmAtSpeed(ArmJoint::lower_arm, 1000);

    std::unique_lock<std::mutex> lk(encoder_pos_mutex_);
    int num_enc_same = 0;
    int total_count = 0;
    int64_t last_enc_count = current_arm_enc_pos_lower_;
    while (num_enc_same < 20) {
      std::cv_status cv_status = encoder_pos_cv_.wait_for(lk,
                                                          std::chrono::milliseconds(kPubTimerIntervalMS * 25));
      if (cv_status == std::cv_status::timeout) {
        std::cerr << "No data in 100ms???" << std::endl;
        break;
      }
      if (current_arm_enc_pos_lower_ == last_enc_count) {
        num_enc_same++;
      } else {
        num_enc_same = 0;
      }
      total_count++;
      if (total_count > 100) {
        //break;
      }
      last_enc_count = current_arm_enc_pos_lower_;
    }

    std::cerr << "Stopped arm" << std::endl;
    lift_cmd_->moveArmAtSpeed(ArmJoint::lower_arm, 0);

    lift_cmd_->setMotorMode(ArmJoint::lower_arm, ArmMotorMode::closed_loop_count_position);
    std::cerr << "Back in position control" << std::endl;

    arm_zero_service_running_ = false;

    if (num_enc_same != 20) {
      response->success = false;
      response->message = "Error";
    } else {
      response->success = true;
      response->message = "Success";
    }
  }

  // Tunable parameters
  const uint32_t kPubTimerIntervalMS = 20;
  const uint32_t kPingTimerIntervalMS = 20;
  const uint32_t kWatchdogIntervalMS = 500;
  static constexpr double kPingRecvPercentage = 0.6;

  // Constants calculated based on the tunable parameters above
  const uint32_t kPingsPerWatchdogInterval = kWatchdogIntervalMS / kPingTimerIntervalMS;
  const uint32_t kMinPingsExpected = kPingsPerWatchdogInterval * kPingRecvPercentage;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr z_pos_cmd_sub_;
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
  std::mutex                                                       encoder_pos_mutex_;
  std::condition_variable                                          encoder_pos_cv_;
  std::atomic<bool>                                                arm_zero_service_running_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jaguar4x4Arm>("192.168.0.63", 10001, 10002));
  rclcpp::shutdown();
  return 0;
}
