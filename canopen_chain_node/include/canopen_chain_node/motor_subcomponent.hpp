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

    std::string canopen_node_name_;
    canopen::ObjectStorageSharedPtr canopen_object_storage_;

    ClassAllocator<canopen::MotorBase> motor_allocator_;
    canopen::MotorBaseSharedPtr motor_;

    bool switchMode(const canopen::MotorBase::OperationMode &m);

    void publishMotorState();

    void handleSwitch402State(
            const std::shared_ptr<canopen_msgs::srv::Switch402State::Request> request,
            std::shared_ptr<canopen_msgs::srv::Switch402State::Response> response);
};

}  // namespace canopen_chain_node

#endif // CANOPEN_CHAIN_NODE__MOTOR_SUBCOMPONENT_HPP_