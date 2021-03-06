#ifndef CANOPEN_MOTOR_NODE_ROBOT_LAYER_H_
#define CANOPEN_MOTOR_NODE_ROBOT_LAYER_H_

#include <unordered_map>

// #include <hardware_interface/joint_command_interface.h>
// #include <hardware_interface/joint_state_interface.h>
// #include <joint_limits_interface/joint_limits_interface.h>
// #include <hardware_interface/robot_hw.h>
// #include <urdf/urdfdom_compatibility.h>
// #include <urdf/model.h>

#include <rclcpp/rclcpp.hpp>

#include <canopen_402/base.hpp>
#include <canopen_motor_node/handle_layer_base.hpp>

#include <hardware_interface/robot_hardware.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

namespace canopen {

class RobotLayer : public LayerGroupNoDiag<HandleLayerBase>,
                   public hardware_interface::RobotHardware {
  // hardware_interface::JointStateInterface state_interface_;
  // hardware_interface::PositionJointInterface pos_interface_;
  // hardware_interface::VelocityJointInterface vel_interface_;
  // hardware_interface::EffortJointInterface eff_interface_;
  //
  // joint_limits_interface::PositionJointSoftLimitsInterface
  // pos_soft_limits_interface_;
  // joint_limits_interface::PositionJointSaturationInterface
  // pos_saturation_interface_;
  // joint_limits_interface::VelocityJointSoftLimitsInterface
  // vel_soft_limits_interface_;
  // joint_limits_interface::VelocityJointSaturationInterface
  // vel_saturation_interface_;
  // joint_limits_interface::EffortJointSoftLimitsInterface
  // eff_soft_limits_interface_;
  // joint_limits_interface::EffortJointSaturationInterface
  // eff_saturation_interface_;

  // ros::NodeHandle nh_;
  // urdf::Model urdf_;

  struct SwitchData {
    // HandleLayerBaseSharedPtr handle;
    canopen::MotorBase::OperationMode mode;
    bool enforce_limits;
  };
  typedef std::vector<SwitchData> SwitchContainer;
  typedef std::unordered_map<std::string, SwitchContainer> SwitchMap;
  SwitchMap switch_map_;

  std::atomic<bool> first_init_;

  void stopControllers(const std::vector<std::string> controllers);

public:
  typedef std::unordered_map<std::string, HandleLayerBaseSharedPtr> HandleMap;
  HandleMap handles_;
  void add(const std::string &name, HandleLayerBaseSharedPtr handle);
  RobotLayer(rclcpp::Logger logger);
  // RobotLayer(ros::NodeHandle nh);
  // urdf::JointConstSharedPtr getJoint(const std::string &n) const { return "";
  // } //urdf_.getJoint(n); }
  hardware_interface::hardware_interface_ret_t init();
  hardware_interface::hardware_interface_ret_t read();
  hardware_interface::hardware_interface_ret_t write();

  double temp_pos_ = 0.0;
  double temp_vel_ = 0.0;
  double temp_eff_ = 0.0;

  virtual void handleInit(canopen::LayerStatus &status);

  // void enforce(const ros::Duration &period, bool reset);
  // virtual bool prepareSwitch(const
  // std::list<hardware_interface::ControllerInfo> &start_list, const
  // std::list<hardware_interface::ControllerInfo> &stop_list);
  // virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>
  // &start_list, const std::list<hardware_interface::ControllerInfo>
  // &stop_list);

  void doSwitch();
private:
  rclcpp::Logger ros_logger_;
};

typedef std::shared_ptr<RobotLayer> RobotLayerSharedPtr;

} // namespace canopen

#endif
