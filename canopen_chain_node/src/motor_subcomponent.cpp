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

    declareParameters();
    getParameters();

    initObjectDictionaryEntries();

    createMotorLayer();

    initRosInterface();

    // FIXME(sam): try to catch all possible timeout exceptions when reading/writing
    // to object dictionary to prevent program termination in case the canopen node stops responding.
}

void MotorSubcomponent::declareParameters()
{
    parent_component_->declare_parameter(canopen_node_name_ + ".device_type",
                                         rclcpp::ParameterValue("unknown"));
    parent_component_->declare_parameter(canopen_node_name_ + ".default_operation_mode",
                                         rclcpp::ParameterValue("Profiled Velocity"));
    parent_component_->declare_parameter(canopen_node_name_ + ".motor_to_angular_velocity_scaling_factor",
                                         rclcpp::ParameterValue(1.0));
    parent_component_->declare_parameter(canopen_node_name_ + ".gear_reduction",
                                         rclcpp::ParameterValue(1.0));
    parent_component_->declare_parameter(canopen_node_name_ + ".reverse_motor_direction",
                                         rclcpp::ParameterValue(false));
}

void MotorSubcomponent::getParameters()
{
  parent_component_->get_parameter(canopen_node_name_ + ".device_type",
                                   device_type_);
  RCLCPP_INFO(parent_component_->get_logger(),  
              "%s, device_type: %s", 
              canopen_node_name_.c_str(),
              device_type_.c_str());

  if (device_type_ != "Nanotec-N5-2-2")
  {
    parent_component_->get_parameter(canopen_node_name_ + ".motor_to_angular_velocity_scaling_factor",
                                     motor_to_angular_velocity_scaling_factor_);
    RCLCPP_INFO(parent_component_->get_logger(),  
                "%s, motor_to_angular_velocity_scaling_factor: %f", 
                canopen_node_name_.c_str(), 
                motor_to_angular_velocity_scaling_factor_);
  }

  parent_component_->get_parameter(canopen_node_name_ + ".gear_reduction",
                                   gear_reduction_);
  RCLCPP_INFO(parent_component_->get_logger(),
              "%s, gear reduction: %f",
              canopen_node_name_.c_str(),
              gear_reduction_);

  parent_component_->get_parameter(canopen_node_name_ + ".reverse_motor_direction",
                                   reverse_motor_direction_);
  RCLCPP_INFO(parent_component_->get_logger(),
              "%s, reverse_motor_direction: %s",
              canopen_node_name_.c_str(),
              reverse_motor_direction_ ? "true": "false");
}

void MotorSubcomponent::initObjectDictionaryEntries()
{
    canopen_object_storage_->entry(velocity_actual_value_, 0x606c);
    canopen_object_storage_->entry(profile_acceleration_, 0x6083);
    canopen_object_storage_->entry(profile_deceleration_, 0x6084);

    // Specific for Nanotec N5 motor controller
    canopen_object_storage_->entry(velocity_numerator_, 0x2061);
    canopen_object_storage_->entry(velocity_denominator_, 0x2062);
}

void MotorSubcomponent::createMotorLayer()
{
    std::string alloc_name = "canopen_402/Motor402Allocator";
    // TODO(sam): Remove settings?
    RosSettings settings;
    try {
      motor_ = motor_allocator_.allocateInstance(
          alloc_name,
          canopen_node_name_ + "_motor",
          canopen_object_storage_,
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
}

void MotorSubcomponent::initRosInterface()
{
    motor_state_publisher_ = parent_component_->create_publisher<canopen_msgs::msg::MotorState>(
        canopen_node_name_ + "/motor_state", rclcpp::QoS(rclcpp::SystemDefaultsQoS()));

    velocity_actual_publisher_ = parent_component_->create_publisher<std_msgs::msg::Float32>(
        canopen_node_name_ + "/velocity_actual", rclcpp::QoS(rclcpp::SystemDefaultsQoS()));
}

void MotorSubcomponent::handleSwitch402State(
        const std::shared_ptr<canopen_msgs::srv::Switch402State::Request> request,
        std::shared_ptr<canopen_msgs::srv::Switch402State::Response> response)
{
    RCLCPP_INFO(parent_component_->get_logger(), 
            "Switching 402 State to %s",
            request->state.c_str());

    auto state = motor_->getStateFromString(request->state);
    if (motor_->switchState(state))
    {
      response->success = true;
      response->message = "Switched 402 State";
    } else
    {
      response->success = false;
      response->message = "Failed to switch 402 State";
    }
}

void MotorSubcomponent::handleSwitchOperationMode(
        const std::shared_ptr<canopen_msgs::srv::SwitchMotorOperationMode::Request> request,
        std::shared_ptr<canopen_msgs::srv::SwitchMotorOperationMode::Response> response)
{
    RCLCPP_INFO(parent_component_->get_logger(), 
            "Switching Operation Mode to %s",
            request->operation_mode.c_str());

    auto mode = motor_->getModeFromString(request->operation_mode);
    if (motor_->enterModeAndWait(mode))
    {
      response->success = true;
      response->message = "Switched Operation Mode";
    } else
    {
      response->success = false;
      response->message = "Failed to switch Operation Mode";
    }
}

void MotorSubcomponent::activate()
{
  if (device_type_ == "Nanotec-N5-2-2")
  {
    motor_to_angular_velocity_scaling_factor_ = 2 * M_PI * double(velocity_numerator_.get()) / double(velocity_denominator_.get());
    RCLCPP_INFO(parent_component_->get_logger(),  
                "%s motor_to_angular_velocity_scaling_factor: %f", 
                canopen_node_name_.c_str(), 
                motor_to_angular_velocity_scaling_factor_);
  }
  switch_402_state_srv_ = parent_component_->create_service<canopen_msgs::srv::Switch402State>(
      canopen_node_name_ + "/switch_402_state", 
      std::bind(&MotorSubcomponent::handleSwitch402State, this,
      std::placeholders::_1, std::placeholders::_2));

  switch_operation_mode_srv_ = parent_component_->create_service<canopen_msgs::srv::SwitchMotorOperationMode>(
      canopen_node_name_ + "/switch_operation_mode", 
      std::bind(&MotorSubcomponent::handleSwitchOperationMode, this,
      std::placeholders::_1, std::placeholders::_2));

  motor_state_publisher_->on_activate();
  auto publish_motor_state_callback = [this]() -> void
  {
    publishMotorState();
  };
  motor_state_publisher_timer_= parent_component_->create_wall_timer(100ms, publish_motor_state_callback);

  velocity_actual_publisher_->on_activate();
  auto publish_velocity_actual_callback = [this]() -> void
  {
    publishVelocityActual();
  };
  velocity_actual_publisher_timer_= parent_component_->create_wall_timer(20ms, publish_velocity_actual_callback);

  profiled_velocity_command_subscription_ = parent_component_->create_subscription<canopen_msgs::msg::ProfiledVelocityCommand>(
    canopen_node_name_ + "/profiled_velocity_command",
    10,
    std::bind(&MotorSubcomponent::profiledVelocityCommandCallback, this, std::placeholders::_1));

  if (!motor_->switchState(canopen::State402::Ready_To_Switch_On))
  {
      RCLCPP_ERROR(parent_component_->get_logger(), "Could not set motor to switched on");
  }

  // TODO(sam): Startup mode selectable by param?
  motor_->enterModeAndWait(canopen::Motor402::Profiled_Velocity);

  // FIXME(sam): Should not need to run this two times...
  if (!motor_->switchState(canopen::State402::Operation_Enable))
  {
      RCLCPP_ERROR(parent_component_->get_logger(), "Could not enable motor");
  }
}

void MotorSubcomponent::publishVelocityActual()
{
    float scaling_factor = motor_to_angular_velocity_scaling_factor_/gear_reduction_;
    if (reverse_motor_direction_)
    {
      scaling_factor *= -1;
    }

    auto msg = std_msgs::msg::Float32();
    msg.data = scaling_factor * velocity_actual_value_.get();
    velocity_actual_publisher_->publish(msg);
}

void MotorSubcomponent::profiledVelocityCommandCallback(const canopen_msgs::msg::ProfiledVelocityCommand::SharedPtr msg)
{
    auto scaling_factor = gear_reduction_/motor_to_angular_velocity_scaling_factor_;

    if (reverse_motor_direction_) 
    {
      scaling_factor *= -1;
    }

    profile_acceleration_.set(scaling_factor*msg->profile_acceleration);
    profile_deceleration_.set(scaling_factor*msg->profile_deceleration);

    motor_->setTarget(scaling_factor*msg->target_velocity);
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
  velocity_actual_publisher_->on_deactivate();
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

    motor_->switchState(canopen::State402::Quick_Stop_Active);
    return true;
}

} // namespace canopen_chain_node