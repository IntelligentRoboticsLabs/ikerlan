# Copyright (c) 2023 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License atimage.png
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    bumpgo_cmd = Node(package='fsm_bumpgo',
                      executable='fsm_bumpgo',
                      output='screen',
                      parameters=[{
                          'use_sim_time': True
                      }],
                      remappings=[
                          ('input_bumper', '/event_bumper'),
                          ('output_vel', '/nav_vel')
                      ])
    

    ld = LaunchDescription()

    ld.add_action(bumpgo_cmd)

    return ld