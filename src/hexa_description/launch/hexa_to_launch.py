#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ORBIT / TEST COPY -- based on spawn_hexa.launch.py
# Difference: does NOT spawn the robot at runtime via ros_gz_sim create.
# Assumes the robot is already loaded into the world via world2.sdf's
# <include> block, so this file only starts robot_state_publisher,
# the ros2_control spawners, and the sensor/topic bridges.
#
# Safe to delete once testing is done -- original spawn_hexa.launch.py
# is untouched.

import os
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'hexa_description'
    pkg_rover_share = get_package_share_directory(pkg_name)
    pkg_rover_prefix = get_package_prefix(pkg_name)

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock'
    )

    # Same Ignition resource/plugin path setup as original --
    # resource path additionally needs to find the models/ dir
    # where hexapod_test_model.sdf will live, so world2.sdf's
    # <include><uri>model://hexapod_test_model</uri></include> resolves.
    ign_res = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ign_res += f":{pkg_rover_share}/models"
    ign_plg = os.environ.get('IGN_GAZEBO_PLUGIN_PATH', '')
    ign_plg += f":{pkg_rover_prefix}/lib"
    ign_plg += ":/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins"
    set_ign_resource = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=ign_res)
    set_ign_plugin = SetEnvironmentVariable(name='IGN_GAZEBO_PLUGIN_PATH', value=ign_plg)

    xacro_file = os.path.join(pkg_rover_share, 'urdf', 'hexapod.urdf.xacro')
    controller_yaml = os.path.join(pkg_rover_share, 'config', 'controller.yaml')

    robot_desc = xacro.process_file(
        xacro_file,
        mappings={'prefix': 'hexa_', 'config_dir': pkg_rover_share}
    ).toxml()

    # Robot State Publisher -- still needed for TF, even though the
    # robot itself is now loaded via world2.sdf, not spawned here.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_desc,
        }]
    )

    # NOTE: spawn_hexa node intentionally removed here.
    # The robot is expected to already exist in the world via
    # world2.sdf's <include> block pointing at hexapod_test_model.sdf.

    bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="rover_gz_bridge",
                arguments=[
                    #"/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
                    #"/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                    #"/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                    #"/camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
                    "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",

                    "/world/default/model/hexa/link/hexa_Tibia_1/sensor/contact_sensor_1/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
                    "/world/default/model/hexa/link/hexa_Tibia_2/sensor/contact_sensor_2/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
                    "/world/default/model/hexa/link/hexa_Tibia_3/sensor/contact_sensor_3/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
                    "/world/default/model/hexa/link/hexa_Tibia_4/sensor/contact_sensor_4/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
                    "/world/default/model/hexa/link/hexa_Tibia_5/sensor/contact_sensor_5/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
                    "/world/default/model/hexa/link/hexa_Tibia_6/sensor/contact_sensor_6/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
                ],
                remappings=[
                    ("/camera/image", "/camera/image_raw"),
                    ("/camera/depth_image", "/camera/depth/image_raw"),
                    ("/camera/camera_info", "/camera/camera_info"),
                    ("/camera/points", "/camera/depth/points"),
                ],
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    camera_info_relay = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='camera_info_relay',
                arguments=['/camera/camera_info', '/camera/depth/camera_info'],
                output='screen',
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawner_joint_state_broadcaster',
                arguments=['joint_state_broadcaster', '--param-file', controller_yaml],
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    hexapod_joint_trajectory_controller_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawner_hexapod_joint_trajectory_controller',
                arguments=['hexapod_joint_trajectory_controller', '--param-file', controller_yaml],
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        set_ign_resource,
        set_ign_plugin,
        bridge,
        rsp_node,
    #    camera_info_relay,
        joint_state_broadcaster_spawner,
        hexapod_joint_trajectory_controller_spawner,
    ])
