#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


#gait script lives in the source tree (it is not installed as an executable)
SCRIPT=os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "hexa_description", "hexa_description", "cmd_vel_fixed_heading.py"))
if not os.path.exists(SCRIPT):
    SCRIPT=os.path.expanduser("~/hexapod_ws/src/hexa_description/hexa_description/cmd_vel_fixed_heading.py")


def generate_launch_description():

    # Packages
    sim_pkg = get_package_share_directory("hexa_simulation")
    desc_pkg = get_package_share_directory("hexa_description")

    # World
    world = os.path.join(sim_pkg, "worlds", "world.sdf")

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r -v 4 {world}",
        }.items(),
    )

    # Robot launch
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                desc_pkg,
                "launch",
                "spawn_hexa.launch.py",
            )
        )
    )
#using gz odom via bridge temorarily
    bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="odom_bridge",  # just a label for this node, can be anything
                arguments=[
                    "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                    "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                ],
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    #new terminals: gait node (shows its menu) and keyboard teleop, started after the bridge is up
    gait=ExecuteProcess(cmd=["gnome-terminal", "--", "python3", SCRIPT])
    teleop=ExecuteProcess(cmd=["gnome-terminal", "--", "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])
    terminals=TimerAction(period=8.0, actions=[gait, teleop])

    return LaunchDescription([
        gazebo,
        robot,
        bridge,
        terminals
    ])