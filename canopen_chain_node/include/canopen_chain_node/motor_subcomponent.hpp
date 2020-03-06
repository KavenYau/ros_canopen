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

#ifndef CANOPEN_CHAIN_NODE__MOTOR_SUBCOMPONENT_HPP_
#define CANOPEN_CHAIN_NODE__MOTOR_SUBCOMPONENT_HPP_

#include <canopen_master/canopen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <canopen_402/base.hpp>
#include <canopen_402/motor.hpp>
#include <pluginlib/class_loader.hpp>

#include <canopen_msgs/srv/switch_motor_operation_mode.hpp>
#include <canopen_msgs/srv/switch402_state.hpp>
#include <canopen_msgs/msg/motor_state.hpp>
#include <canopen_msgs/msg/profiled_velocity_command.hpp>
#include <std_msgs/msg/float32.hpp>

#include "canopen_chain_helpers.hpp"

namespace canopen_chain_node
{

class MotorSubcomponent
{
public:
    MotorSubcomponent(
        rclcpp_lifecycle::LifecycleNode *parent_component,
        std::string canopen_node_name,
        canopen::ObjectStorageSharedPtr canopen_object_storage);

    void activate();
    void deactivate();

    canopen::MotorBaseSharedPtr getMotor() { return motor_; };

private:
    rclcpp_lifecycle::LifecycleNode *parent_component_;

    rclcpp::Service<canopen_msgs::srv::SwitchMotorOperationMode>::SharedPtr switch_operation_mode_srv_;
    rclcpp::Service<canopen_msgs::srv::Switch402State>::SharedPtr switch_402_state_srv_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<canopen_msgs::msg::MotorState>> motor_state_publisher_;
    rclcpp::TimerBase::SharedPtr motor_state_publisher_timer_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>> velocity_actual_publisher_;
    rclcpp::TimerBase::SharedPtr velocity_actual_publisher_timer_;

    std::shared_ptr<rclcpp::Subscription<canopen_msgs::msg::ProfiledVelocityCommand>> profiled_velocity_command_subscription_;

    std::string canopen_node_name_;
    canopen::ObjectStorageSharedPtr canopen_object_storage_;

    ClassAllocator<canopen::MotorBase> motor_allocator_;
    canopen::MotorBaseSharedPtr motor_;

    canopen::ObjectStorage::Entry<uint32_t> velocity_numerator_;
    canopen::ObjectStorage::Entry<uint32_t> velocity_denominator_;
    canopen::ObjectStorage::Entry<int32_t> velocity_actual_value_;
    canopen::ObjectStorage::Entry<uint32_t> profile_acceleration_;
    canopen::ObjectStorage::Entry<uint32_t> profile_deceleration_;

    std::string device_type_;
    std::string default_operation_mode_;
    float motor_to_angular_velocity_scaling_factor_;
    float gear_reduction_;
    bool reverse_motor_direction_;

    void declareParameters();
    void getParameters();
    void initObjectDictionaryEntries();
    void createMotorLayer();
    void initRosInterface();

    bool switchMode(const canopen::MotorBase::OperationMode &m);

    void publishMotorState();
    void publishVelocityActual();

    void handleSwitch402State(
            const std::shared_ptr<canopen_msgs::srv::Switch402State::Request> request,
            std::shared_ptr<canopen_msgs::srv::Switch402State::Response> response);
    void handleSwitchOperationMode(
            const std::shared_ptr<canopen_msgs::srv::SwitchMotorOperationMode::Request> request,
            std::shared_ptr<canopen_msgs::srv::SwitchMotorOperationMode::Response> response);

    void profiledVelocityCommandCallback(const canopen_msgs::msg::ProfiledVelocityCommand::SharedPtr msg);
};

}  // namespace canopen_chain_node

#endif // CANOPEN_CHAIN_NODE__MOTOR_SUBCOMPONENT_HPP_