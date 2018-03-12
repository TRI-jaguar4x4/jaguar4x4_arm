#include <chrono>
#include <future>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jaguar4x4_arm/ArmCommand.h"
#include "jaguar4x4_arm/ArmReceive.h"

#include "jaguar4x4_arm/Communication.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Jaguar4x4Arm : public rclcpp::Node
{
public:
  Jaguar4x4Arm(const std::string& ip, uint16_t port) : Node("jaguar4x4Arm"), recv_thread_()
  {
    lift_cmd_ = std::make_unique<ArmCommand>(&board_1_comm_);
    lift_rcv_ = std::make_unique<ArmReceive>(&board_1_comm_);
    board_1_comm_.connect(ip, port);
    
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
      500ms, std::bind(&Jaguar4x4Arm::timerCallback, this));

    lift_cmd_->resume();

    future_ = exit_signal_.get_future();

    recv_thread_ = std::thread(&Jaguar4x4Arm::recvCallback, this,
			       std::move(future_), std::move(lift_rcv_));
  }

  ~Jaguar4x4Arm()
  {
    exit_signal_.set_value();
    recv_thread_.join();
  }
  
private:
  // clalancette: To test this currently, the following command-line can
  // be used:
  // ros2 topic pub /z_position geometry_msgs/PoseStamped "{header:{stamp:{sec: 4, nanosec: 14}, frame_id: 'frame'}, pose:{position:{x: 1, y: 2, z: 3}, orientation:{x: 4, y: 5, z: 6, w: 7}}}"
  void liftCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    int64_t this_stamp;
    this_stamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

    if (this_stamp < last_stamp_) {
      // An older command came through, just ignore it.  Note that we currently
      // allow equal timestamps to make using the command-line easier.
      return;
    }
    last_stamp_ = this_stamp;
    if (msg->pose.position.z > 0) {
      lift_cmd_->moveArmDown(ArmCommand::Joint::lower_arm);
    }
    else {
      lift_cmd_->moveArmUp(ArmCommand::Joint::lower_arm);
    }
  }

  void recvCallback(std::future<void> local_future,
		    std::unique_ptr<ArmReceive> lift_recv)
  {
    std::cerr << "entered the callback\n";
    std::future_status status;
    do {
      try {
	lift_recv->getAndParseMessage();
      }
      catch (...) {
	std::cerr << "threw\n";
      }
      
      status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
    std::cerr << "no longer in while\n";    
  }
  
  void timerCallback()
  {
    lift_cmd_->ping();
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr z_pos_cmd_sub_;
  Communication board_1_comm_;
  std::unique_ptr<ArmCommand> lift_cmd_;
  std::unique_ptr<ArmReceive> lift_rcv_;
  int64_t last_stamp_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread recv_thread_;
  std::promise<void> exit_signal_;
  std::future<void> future_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jaguar4x4Arm>("192.168.0.63", 10001));
  rclcpp::shutdown();
  return 0;
}
