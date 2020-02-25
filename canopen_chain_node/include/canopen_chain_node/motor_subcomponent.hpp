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

private:
    rclcpp_lifecycle::LifecycleNode *parent_component_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr lathing_not_working_for_web_ui_temporary_timer_;

    std::string canopen_node_name_;
    canopen::ObjectStorageSharedPtr canopen_object_storage_;

};

}  // namespace canopen_chain_node

#endif // CANOPEN_CHAIN_NODE__MOTOR_SUBCOMPONENT_HPP_