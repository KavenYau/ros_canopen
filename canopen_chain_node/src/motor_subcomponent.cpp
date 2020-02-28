// Copyright (c) 2019, Samuel Lindgren
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "canopen_chain_node/motor_subcomponent.hpp"

using namespace std::chrono_literals;

namespace canopen_chain_node
{

// TODO(sam): remove/fix this
class RosSettings : public canopen::Settings {
public:
  RosSettings() {}
private:
  virtual bool getRepr(const std::string &n, std::string &repr) const {
    return false;
  }
};

MotorSubcomponent::MotorSubcomponent(
    rclcpp_lifecycle::LifecycleNode *parent_component,
    std::string canopen_node_name,
    canopen::ObjectStorageSharedPtr canopen_object_storage): 
    parent_component_(parent_component),
    canopen_node_name_(canopen_node_name),
    canopen_object_storage_(canopen_object_storage),
    motor_allocator_("canopen_402", "canopen::MotorBase::Allocator")
{
    RCLCPP_INFO(parent_component_->get_logger(), 
            "Creating Motor Profile [402] Subcomponenet for %s",
            canopen_node_name_.c_str());

    // FIXME(sam): try to catch all possible timeout exceptions when reading/writing
    // to object dictionary to prevent program termination in case the canopen node stops responding.

    std::string alloc_name = "canopen_402/Motor402Allocator";
    // TODO(sam): Remove/fix settings?
    RosSettings settings;
    try {
      motor_ = motor_allocator_.allocateInstance(
          alloc_name,
          canopen_node_name + "_motor",
          canopen_object_storage,
          settings);
    } catch (const std::exception &e) {
      std::string info = boost::diagnostic_information(e);
      RCLCPP_ERROR(parent_component_->get_logger(), info);
    }
    
    if (!motor_) {
      RCLCPP_ERROR(parent_component_->get_logger(), "could not allocate motor");
      // return false;
    }

    motor_->registerDefaultModes(canopen_object_storage_);

    motor_state_publisher_ = parent_component_->create_publisher<canopen_msgs::msg::MotorState>(
        canopen_node_name_ + "/motor_state", rclcpp::QoS(rclcpp::SystemDefaultsQoS()));
}

void MotorSubcomponent::activate()
{
  motor_state_publisher_->on_activate();

  auto publish_motor_state_callback = [this]() -> void
  {
    publishMotorState();
  };

  motor_state_publisher_timer_= parent_component_->create_wall_timer(100ms, publish_motor_state_callback);
}

void MotorSubcomponent::publishMotorState()
{
  auto msg = canopen_msgs::msg::MotorState();
  msg.operation_mode = motor_->getModeAsString();
  msg.state_402 = motor_->getStateAsString();
  
  motor_state_publisher_->publish(msg);
}

void MotorSubcomponent::deactivate()
{
  motor_state_publisher_->on_deactivate();
}

bool MotorSubcomponent::switchMode(const canopen::MotorBase::OperationMode &operation_mode)
{
    if (motor_->getMode() != operation_mode) 
    {
        if (motor_->enterModeAndWait(operation_mode)) 
        {
            canopen::LayerStatus layer_status;
            motor_->halt(layer_status);
            return false;
        }
    }
    // NOTE(sam): hmmm
    // motor_->switchState(canopen::State402::Quick_Stop_Active);
    return true;
}

} // namespace canopen_chain_node