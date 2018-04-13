// Copyright 2018 Toyota Research Institute.  All rights reserved.
#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <jaguar4x4_comms/Communication.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
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
    auto board_1_comm = std::make_shared<Communication>();
    board_1_comm->connect(ip, arm_port);

    auto board_2_comm = std::make_shared<Communication>();
    board_2_comm->connect(ip, hand_port);

    lift_cmd_ = std::make_unique<ArmCommand>(board_1_comm);
    lift_rcv_ = std::make_unique<ArmReceive>(board_1_comm);

    hand_cmd_ = std::make_unique<HandCommand>(board_2_comm);
    hand_rcv_ = std::make_unique<ArmReceive>(board_2_comm);

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
    lift_pub_ = this->create_publisher<jaguar4x4_arm_msgs::msg::Lift>("liftInfo");
    lift_pub_thread_ = std::thread(&Jaguar4x4Arm::liftPubThread, this, future_);

    ping_thread_ = std::thread(&Jaguar4x4Arm::pingThread, this, future_);

    lift_cmd_->configure(kPubTimerIntervalMS);
    lift_cmd_->resume();

    hand_cmd_->configure(kPubTimerIntervalMS);
    hand_cmd_->resume();

    arm_recv_thread_ = std::thread(&Jaguar4x4Arm::armRecvThread, this,
                                   future_);
    hand_recv_thread_ = std::thread(&Jaguar4x4Arm::handRecvThread, this,
                                    future_);
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
    lift_cmd_->moveArmToRelativeEncoderPos(ArmJoint::upper_arm, msg->pose.position.z);

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

  void pingThread(std::shared_future<void> local_future)
  {
    std::future_status status;
    uint32_t num_pings_sent{0};

    do {
      lift_cmd_->ping();
      hand_cmd_->ping();
      num_pings_sent++;

      if (num_pings_sent >= kPingsPerWatchdogInterval) {

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
        num_lift_pings_recvd_ = 0;
        num_hand_pings_recvd_ = 0;
        num_pings_sent = 0;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(kPingTimerIntervalMS));

      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
  }

  // Tunable parameters
  const uint32_t kPubTimerIntervalMS = 100;
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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jaguar4x4Arm>("192.168.0.63", 10001, 10002));
  rclcpp::shutdown();
  return 0;
}
