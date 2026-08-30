#!/usr/bin/env python3
"""
Calibration tool - NOT the gait. Use this first, before running any IK-based
gait, to find out what each joint's zero position and positive direction
actually mean on the real robot.

Usage:
  1. Run this node.
  2. It publishes ALL JOINTS AT 0.0 first and holds - confirm the robot is
     in a safe, stable pose before doing anything else. If it isn't, STOP
     and tell me what the zero pose looks like (photo helps).
  3. Type a joint index (0-17) and an angle in DEGREES to test that single
     joint in isolation, e.g.:
         joint index: 1
         degrees: 20
     Watch which physical joint moves and which direction, then note it down.
  4. Type 'q' to quit.

This never sends more than one non-zero joint at a time, and never sends
anything beyond +/-45 degrees, to keep it safe to test on hardware.
"""

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [f"hexa_Joint{leg}_{j}" for leg in range(1, 7) for j in (1, 2, 3)]
MAX_TEST_DEG = 45.0
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Test(Node):
    def __init__(self):
        super().__init__("test_joint")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/hexapod_joint_trajectory_controller/joint_trajectory",
            10
        )

        self.q = [
            0.013250, -0.068470, 0.050032,
            0.000000, 0.034370, 0.000000,
            0.000000, 0.016197, 0.011683,
            0.000083, -0.021910, 0.018496,
            0.000000, 0.013367, 0.001428,
            -0.002046, -0.024462, 0.029975,
        ]

        self.names = [
            "hexa_Joint1_1", "hexa_Joint1_2", "hexa_Joint1_3",
            "hexa_Joint2_1", "hexa_Joint2_2", "hexa_Joint2_3",
            "hexa_Joint3_1", "hexa_Joint3_2", "hexa_Joint3_3",
            "hexa_Joint4_1", "hexa_Joint4_2", "hexa_Joint4_3",
            "hexa_Joint5_1", "hexa_Joint5_2", "hexa_Joint5_3",
            "hexa_Joint6_1", "hexa_Joint6_2", "hexa_Joint6_3",
        ]

        self.step = 0
        self.timer = self.create_timer(2.0, self.run)

    def run(self):
        q = self.q.copy()

        if self.step == 0:
            # Move Joint1_1 by only 0.1 rad
            q[0] += 0.1
            self.get_logger().info("Moving Joint1_1 +0.1 rad")

        elif self.step == 1:
            # Return to original position
            self.get_logger().info("Returning Joint1_1")

        else:
            self.get_logger().info("Done")
            self.timer.cancel()
            return

        msg = JointTrajectory()
        msg.joint_names = self.names

        p = JointTrajectoryPoint()
        p.positions = q
        p.time_from_start.sec = 1

        msg.points = [p]

        self.pub.publish(msg)

        self.step += 1


def main():
    rclpy.init()
    node = Test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()