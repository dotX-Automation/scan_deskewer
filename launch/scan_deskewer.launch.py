"""
Scan Deskewer app sample launch file.

Giorgio Manca <giorgio.manca.97@gmail.com>

July 4, 2025
"""

# Copyright 2025 dotX Automation s.r.l.
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
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('scan_deskewer'),
        'config',
        'scan_deskewer.yaml')

    # Declare launch arguments
    nm = LaunchConfiguration('name')
    ns = LaunchConfiguration('namespace')
    cf = LaunchConfiguration('cf')
    nm_launch_arg = DeclareLaunchArgument(
        'name',
        default_value='scan_deskewer')
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='')
    cf_launch_arg = DeclareLaunchArgument(
        'cf',
        default_value=config)
    ld.add_action(nm_launch_arg)
    ld.add_action(ns_launch_arg)
    ld.add_action(cf_launch_arg)

    # Launch dua_tf_server node
    tf_server = Node(
        package='dua_tf_server',
        executable='dua_tf_server_app',
        namespace=ns,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config]
    )
    ld.add_action(tf_server)

    # Create node launch description
    node = Node(
        package='scan_deskewer',
        executable='scan_deskewer_app',
        name=nm,
        namespace=ns,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[
            cf
        ],
        remappings=[
            ('/dua_tf_server/get_transform', '/dua_tf_server/get_transform'),
            ('/input',                       '/input'),
            ('/imu',                         '/imu'),
            ('/twist',                       '/twist'),
            ('/odometry',                    '/odometry'),
            ('/scan_deskewer/output',        '/output'),
        ]
    )
    ld.add_action(node)

    return ld
