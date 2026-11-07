#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


#workspace root = 3 levels above this launch folder (hexapod_ws/src/hexa_simulation/launch)
#realpath only lands in src/ with --symlink-install, otherwise fall back to the known path
WS_ROOT=os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", ".."))
if not os.path.isdir(os.path.join(WS_ROOT, "src")):
    WS_ROOT=os.path.expanduser("~/hexapod_ws")

#virtual environment lives in the workspace root
VENV=os.path.join(WS_ROOT, ".venv")
VENV_PY=os.path.join(VENV, "bin", "python")
PIP_PACKAGES="numpy rich"

#gait and teleop scripts live in the source tree (they are not installed as executables)
SCRIPT_DIR=os.path.join(WS_ROOT, "src", "hexa_description", "hexa_description")
SCRIPT=os.path.join(SCRIPT_DIR, "cmd_vel_fixed_heading.py")
TELEOP=os.path.join(SCRIPT_DIR, "pretty_teleop.py")


def in_terminal(script):
    #runs the script with the venv python, exec bash keeps the window open if it crashes
    return ExecuteProcess(cmd=["gnome-terminal", "--", "bash", "-c", f"'{VENV_PY}' '{script}'; exec bash"])


def after_setup(event, context):
    #only open the terminals if the venv setup succeeded
    if event.returncode!=0:
        return [LogInfo(msg="Venv setup failed, gait and teleop terminals NOT started (see messages above)")]
    #small delay so the gz bridge (started at 5 s) is up before the gait node starts
    return [TimerAction(period=5.0, actions=[in_terminal(SCRIPT), in_terminal(TELEOP)])]


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

    #venv setup: create it only if missing (--system-site-packages keeps rclpy and the ROS msgs importable), then pip install
    #pip skips packages that are already installed, so re-running is harmless
    setup_cmd=(
        f"if [ ! -x '{VENV_PY}' ]; then "
        f"echo 'Creating venv in {VENV}'; "
        f"python3 -m venv --system-site-packages '{VENV}' || {{ echo 'venv failed, try: sudo apt install python3-venv'; exit 1; }}; "
        f"fi; "
        f"'{VENV_PY}' -m pip install {PIP_PACKAGES}"
    )
    setup=ExecuteProcess(cmd=["bash", "-c", setup_cmd], output="screen")
    on_setup_done=RegisterEventHandler(OnProcessExit(target_action=setup, on_exit=after_setup))

    return LaunchDescription([
        gazebo,
        robot,
        bridge,
        setup,
        on_setup_done
    ])