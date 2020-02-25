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

MotorSubcomponent::MotorSubcomponent(
    rclcpp_lifecycle::LifecycleNode *parent_component,
    std::string canopen_node_name,
    canopen::ObjectStorageSharedPtr canopen_object_storage): 
    parent_component_(parent_component),
    canopen_node_name_(canopen_node_name),
    canopen_object_storage_(canopen_object_storage)
{
    RCLCPP_INFO(parent_component_->get_logger(), 
            "Creating Motor Profile [402] Subcomponenet for %s",
            canopen_node_name_.c_str());

    // FIXME(sam): try to catch all possible timeout exceptions when reading/writing
    // to object dictionary to prevent program termination in case the canopen node stops responding.
}

void MotorSubcomponent::activate()
{
}

void MotorSubcomponent::deactivate()
{
}

} // namespace canopen_chain_node