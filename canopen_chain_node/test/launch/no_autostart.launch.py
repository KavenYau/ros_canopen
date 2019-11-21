# Copyright 2019 Dyno Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Laucher for canopen_chain"""

import os

import launch

from ament_index_python import get_package_share_directory
from launch_ros.actions import LifecycleNode


def generate_launch_description():

    test_params = os.path.join(
      get_package_share_directory('canopen_chain_node'),
      'test/config/chain_node_params.yaml'
    )

    ld = launch.LaunchDescription()

    chain_node = LifecycleNode(
      node_name='canopen_chain',
      package='canopen_chain_node',
      node_executable='manual_composition',
      output='screen',
      parameters=[
        test_params
      ]
    )

    ld.add_action(chain_node)

    return ld
