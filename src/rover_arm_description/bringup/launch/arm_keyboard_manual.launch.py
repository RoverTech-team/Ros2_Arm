# Copyright 2023 ros2_control Development Team
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

"""Arm-only manual control: mock hardware, controllers and manual teleop.

Loads just the arm (no rover base, no Gazebo, no MoveIt), starts the
joint_state_broadcaster, arm_controller and gripper_controller, and starts
manual teleop for jogging every joint by hand:

  - keyboard teleop in its own xterm window (ros2 launch does not forward
    stdin to its processes)
  - gamepad teleop subscribing to /joy. joy_source selects who publishes it:
      auto   - joy_node on Linux (Jetson/Ubuntu, /dev/input/js0),
               pygame publisher elsewhere (macOS)
      ros    - standard `joy` package node (Linux only)
      pygame - bundled arm_joy_publisher.py (any platform with pygame)
      none   - no gamepad support

RViz shows the arm. A full HTML guide lives in
doc/arm_keyboard_manual_guide.html of this package.
"""

import shutil
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    gui = LaunchConfiguration('gui').perform(context)
    with_keyboard = LaunchConfiguration('teleop').perform(context)
    joy_source = LaunchConfiguration('joy_source').perform(context)

    if joy_source == 'auto':
        joy_source = 'ros' if sys.platform.startswith('linux') else 'pygame'

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('rover_arm_description'),
                    'urdf',
                    'arm.urdf.xacro',
                ]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('rover_arm_description'),
            'bringup',
            'config',
            'arm_controller.yaml',
        ]
    )

    nodes = [
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, robot_controllers],
            remappings=[('~/robot_description', '/robot_description')],
            output='both',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[robot_description],
            remappings=[('joint_states', '/joint_states_mimic')],
        ),
        Node(
            package='rover_arm_description',
            executable='gripper_mimic_bridge.py',
            name='gripper_mimic_bridge',
            parameters=[{
                'input_topic': '/joint_states',
                'output_topic': '/joint_states_mimic',
            }],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
                '--param-file', robot_controllers,
            ],
        ),
        LogInfo(msg=[
            'HTML control guide: ',
            PathJoinSubstitution(
                [FindPackageShare('rover_arm_description'),
                 'doc', 'arm_keyboard_manual_guide.html']),
        ]),
    ]

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', robot_controllers,
        ],
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', robot_controllers,
        ],
    )

    on_jsb_exit = [arm_controller_spawner, gripper_controller_spawner]
    if gui == 'true':
        on_jsb_exit.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ))

    # teleop starts only once the arm controller is active
    on_arm_ctrl_exit = []

    if with_keyboard == 'true':
        if shutil.which('xterm') is not None:
            # ros2 launch does not forward stdin, so the keyboard teleop
            # needs its own terminal (xterm ships with XQuartz on macOS)
            on_arm_ctrl_exit.append(Node(
                package='rover_arm_description',
                executable='arm_keyboard_teleop.py',
                name='arm_keyboard_teleop',
                prefix='xterm -e',
                parameters=[{
                    'step': ParameterValue(
                        LaunchConfiguration('step'), value_type=float),
                }],
                output='screen',
            ))
        else:
            nodes.append(LogInfo(msg=[
                'xterm not found: start the keyboard teleop manually in a ',
                'second terminal with\n  ros2 run rover_arm_description ',
                'arm_keyboard_teleop.py',
            ]))

    if joy_source != 'none':
        if joy_source == 'ros':
            nodes.append(Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{
                    'device_id': 0,      # /dev/input/js0
                    'deadzone': 0.05,
                    'autorepeat_rate': 30.0,
                }],
                output='screen',
            ))
            joy_profile = 'joydev'
        elif joy_source == 'pygame':
            nodes.append(Node(
                package='rover_arm_description',
                executable='arm_joy_publisher.py',
                name='arm_joy_publisher',
                output='screen',
            ))
            joy_profile = 'sdl'
        else:
            raise RuntimeError(
                f"invalid joy_source '{joy_source}' "
                '(expected auto, ros, pygame or none)')

        on_arm_ctrl_exit.append(Node(
            package='rover_arm_description',
            executable='arm_joy_teleop.py',
            name='arm_joy_teleop',
            parameters=[{
                'profile': ParameterValue(joy_profile, value_type=str),
            }],
            output='screen',
        ))

    # nodes[2] is now gripper_mimic_bridge (was spawner), keep spawner index
    # joint_state_broadcaster spawner is nodes[3]
    nodes.append(RegisterEventHandler(OnProcessExit(
        target_action=nodes[3], on_exit=on_jsb_exit)))
    if on_arm_ctrl_exit:
        nodes.append(RegisterEventHandler(OnProcessExit(
            target_action=arm_controller_spawner, on_exit=on_arm_ctrl_exit)))

    return nodes


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='true',
            description='Open the keyboard teleop in a new xterm window.',
        ),
        DeclareLaunchArgument(
            'joy_source',
            default_value='auto',
            choices=['auto', 'ros', 'pygame', 'none'],
            description='Gamepad input source: auto (joy_node on Linux, '
                        'pygame elsewhere), ros (joy_node), pygame '
                        '(arm_joy_publisher.py) or none.',
        ),
        DeclareLaunchArgument(
            'step',
            default_value='0.05',
            description='Keyboard rotation step per key press [rad] '
                        '(gripper step is always 1/10 of this in metres).',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('rover_arm_description'),
                 'rviz', 'urdf_config.rviz']
            ),
            description='RViz configuration file.',
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=_launch_setup)])
