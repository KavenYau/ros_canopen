
#ifndef CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_
#define CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_

#include <memory>
#include <map>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <canopen_msgs/msg/debug_publishers.hpp>
#include <canopen_msgs/srv/get_joint_state.hpp>

#include <canopen_402/base.hpp>
#include <canopen_chain_node/ros_chain.hpp>
// #include <canopen_402/motor.hpp>

#include <canopen_motor_node/controller_manager_layer.hpp>
#include <canopen_motor_node/robot_layer.hpp>

namespace canopen {

class MotorChain : public RosChain {
  ClassAllocator<MotorBase> motor_allocator_;
  std::shared_ptr<LayerGroupNoDiag<MotorBase>> motors_;
  RobotLayerSharedPtr robot_layer_;

  std::shared_ptr<ControllerManagerLayer> cm_;

  virtual bool nodeAdded(const NodeSharedPtr &node,
                         const LoggerSharedPtr &logger);

public:
  // MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);
  MotorChain(std::string node_name = "motor_chain_node");

  virtual bool setup_chain();

  // NOTE(sam): optional?
  bool setup_debug_interface(const canopen::NodeSharedPtr &node,
                             MotorBaseSharedPtr motor);

private:
  void handleWrite(LayerStatus &status, const LayerState &current_state);

  void publish_all_debug(HandleLayerBaseSharedPtr handle);
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> state_publishers_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> operation_mode_publishers_;

  rclcpp::Subscription<canopen_msgs::msg::DebugPublishers>::SharedPtr
      set_debug_publishers_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> switch_state_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> set_target_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr>
      switch_operation_mode_subs_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_ros_control_command_sub_;
  rclcpp::Client<canopen_msgs::srv::GetJointState>::SharedPtr get_joint_state_client_;

  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> set_joint_to_zero_srvs_;
};

} // namespace canopen

#endif /* INCLUDE_CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_ */
