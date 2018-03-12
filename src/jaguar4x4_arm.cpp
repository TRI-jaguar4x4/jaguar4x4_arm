#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jaguar4x4_arm/ArmCommand.h"
#include "jaguar4x4_arm/Communication.h"
#include "rclcpp/rclcpp.hpp"

class Jaguar4x4Arm : public rclcpp::Node
{
public:
  Jaguar4x4Arm(const std::string& ip, uint16_t port) : Node("jaguar4x4Arm")
  {
    lift_cmd_ = std::make_unique<ArmCommand>(&board_1_comm_);
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

    lift_cmd_->resume();
  }

private:
  void liftCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    lift_cmd_->moveArmUp(ArmCommand::Joint::lower_arm);
  }
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr z_pos_cmd_sub_;
  Communication board_1_comm_;
  std::unique_ptr<ArmCommand> lift_cmd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jaguar4x4Arm>("192.168.0.63",10001));
  rclcpp::shutdown();
  return 0;
}
