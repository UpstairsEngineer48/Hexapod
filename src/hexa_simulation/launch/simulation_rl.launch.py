#!/usr/bin/env python3
#
# ORBIT / TEST COPY -- based on simulation.launch.py
# Difference: loads world2.sdf (robot included statically) and
# hexa_to_launch.py (no runtime spawn node) instead of the originals.
#
# Safe to delete once testing is done -- original simulation.launch.py
# and spawn_hexa.launch.py are untouched.

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    sim_pkg = get_package_share_directory("hexa_simulation")
    desc_pkg = get_package_share_directory("hexa_description")
    desc_prefix = get_package_prefix("hexa_description")

    # NOTE: points at world2.sdf, not world.sdf.
    # Place world2.sdf in the same worlds/ directory as world.sdf,
    # e.g. hexa_simulation/share/hexa_simulation/worlds/world2.sdf
    world = os.path.join(sim_pkg, "worlds", "world2.sdf")

    # FIX: world2.sdf's <include><uri>model://hexapod_test_model</uri></include>
    # is resolved by Gazebo at world-load time, so IGN_GAZEBO_RESOURCE_PATH
    # must be set BEFORE the gazebo IncludeLaunchDescription runs -- setting
    # it only inside hexa_to_launch.py (which runs after gazebo starts) is
    # too late and causes "Unable to find uri[model://hexapod_test_model]".
    ign_res = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ign_res += f":{desc_pkg}/models"
    ign_plg = os.environ.get('IGN_GAZEBO_PLUGIN_PATH', '')
    ign_plg += f":{desc_prefix}/lib"
    ign_plg += ":/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins"

    set_ign_resource = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=ign_res)
    set_ign_plugin = SetEnvironmentVariable(name='IGN_GAZEBO_PLUGIN_PATH', value=ign_plg)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            # ORIGINAL (with GUI/rendering window) -- use for visual debugging.
            "gz_args": f"-r {world}",

            # RL-FAST (headless, server-only, no GUI window) -- uncomment this
            # line and comment out the one above when running PPO training.
            # "-s" = server only, no ogre2 render window, much lower CPU/GPU load.
            # This is the single biggest RTF win available before touching
            # sensors/xacro at all.
           #"gz_args": f"-s -r {world}",
        }.items(),
    )

    # NOTE: points at hexa_to_launch.py, not spawn_hexa.launch.py.
    # Place hexa_to_launch.py in the same launch/ directory as
    # spawn_hexa.launch.py, e.g.
    # hexa_description/share/hexa_description/launch/hexa_to_launch.py
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                desc_pkg,
                "launch",
                "hexa_to_launch.py",
            )
        )
    )

    bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="odom_bridge",
                arguments=[
                    "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                ],
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    return LaunchDescription([
        set_ign_resource,
        set_ign_plugin,
        gazebo,
        robot,
        bridge
    ])