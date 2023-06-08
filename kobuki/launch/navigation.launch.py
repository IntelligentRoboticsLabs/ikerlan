# Copyright (c) 2023 Intelligent Robotics Lab (URJC)
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

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    kobuki_dir = get_package_share_directory('kobuki')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    config = os.path.join(kobuki_dir, 'config', 'params.yaml')
    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir,
                                   'rviz',
                                   'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="false",
        description="Whether to start the SLAM or the Map Localization",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(kobuki_dir, "maps", conf['kobuki']['map'] + '.yaml'),
        description="Full path to map yaml file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(kobuki_dir, "params", 'nav_params.yaml'),
        description="Full path to map yaml file to load",
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_dir, "/localization_launch.py"]
        ),
        launch_arguments={
            "params_file": params_file,
            "map": map_yaml_file,
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('slam'))
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, "/slam_launch.py"]),
        launch_arguments={"params_file": params_file}.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_dir, "/navigation_launch.py"]
        ),
        launch_arguments={"params_file": params_file}.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                   'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'rviz_config': rviz_config_file}.items())

    ld = LaunchDescription()
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(localization_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)

    return ld
