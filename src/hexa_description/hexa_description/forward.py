#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ik import ik

GRP1 = [['hexa_Joint1_1', 'hexa_Joint1_2', 'hexa_Joint1_3'],
        ['hexa_Joint3_1', 'hexa_Joint3_2', 'hexa_Joint3_3'],
        ['hexa_Joint5_1', 'hexa_Joint5_2', 'hexa_Joint5_3']]

GRP2 = [['hexa_Joint2_1', 'hexa_Joint2_2', 'hexa_Joint2_3'],
        ['hexa_Joint4_1', 'hexa_Joint4_2', 'hexa_Joint4_3'],
        ['hexa_Joint6_1', 'hexa_Joint6_2', 'hexa_Joint6_3']]

JOINT_NAMES = ['hexa_Joint1_1', 'hexa_Joint1_2', 'hexa_Joint1_3',
               'hexa_Joint2_1', 'hexa_Joint2_2', 'hexa_Joint2_3',
               'hexa_Joint3_1', 'hexa_Joint3_2', 'hexa_Joint3_3',
               'hexa_Joint4_1', 'hexa_Joint4_2', 'hexa_Joint4_3',
               'hexa_Joint5_1', 'hexa_Joint5_2', 'hexa_Joint5_3',
               'hexa_Joint6_1', 'hexa_Joint6_2', 'hexa_Joint6_3']

ALL_LEGS = [1, 2, 3, 4, 5, 6]
TRIPOD_GROUP = {1: 'A', 2: 'B', 3: 'A', 4: 'B', 5: 'A', 6: 'B'}


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))



class LegMount:
    """
    Fixed geometry describing where/how a leg is bolted onto the body.
    mount_xy: (x, y) offset of this leg's Joint 1 from body center, in
              the BODY frame, measured with the robot at rest.
    mount_yaw: the angle (radians) this leg's own local x-axis is rotated
               relative to the body's forward (+x) axis. e.g. a leg
               pointing straight out to the side would have mount_yaw
               = +-90 deg; a leg pointing forward-diagonally might be
               +-30 deg, etc. Measure this from your CAD/URDF.
    """
    def __init__(self, mount_xy, mount_yaw):
        self.mount_xy = mount_xy
        self.mount_yaw = mount_yaw


class TripodGait(Node):
    def __init__(self):
        super().__init__("tripod_gait")

        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            "/hexapod_joint_trajectory_controller/joint_trajectory",
            10
        )

        self.dt = 0.01
        self.cycle_time = 1.0
        self.step_height = 0.08
        self.L2 = 0.08
        self.L3 = 0.15


        self.body_vx = 0.1   # m/s in body's forward direction
        self.body_vy = 0.0    # m/s in body's lateral direction

        # Stride length derived from body velocity and cycle time -how far the body (hence each foot's stance sweep) travels per full gait cycle.
        self.stride_x = self.body_vx * self.cycle_time
        self.stride_y = self.body_vy * self.cycle_time

        # ---- Leg mount geometry: TUNE THESE to your actual robot ----
        # Angles/offsets below are placeholders - replace with your
        # hexapod's real CAD/URDF values.
        self.mounts = {
            1: LegMount((0.067149, 0.11215), 2.3578),
            2: LegMount((0.08925, 0.0), 1.6011),
            3: LegMount((0.067149, -0.11215), 0.80016),
            4: LegMount((-0.067149, -0.11215), -0.7472),
            5: LegMount((-0.08925, 0.0), -1.5753),
            6: LegMount((-0.067149, 0.11215), -2.5044),
        }

        # Neutral foot height/reach (in each leg's own local frame,
        # relative to that leg's Joint 2), independent of gait motion.
        self.neutral_reach = 0.18   # how far out the foot rests, meters
        self.neutral_z = -0.155      # how low the foot rests, meters

        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.update)

    def body_frame_foot_offset(self, leg_id, t):

        half_cycle = self.cycle_time / 2

        # Tripod A: legs 1,3,5
        if leg_id == 1 or leg_id == 3 or leg_id == 5:

            if t < half_cycle:
                # Swing
                st = t / half_cycle

                dx = self.stride_x * (st - 0.5)
                dy = self.stride_y * (st - 0.5)
                dz = self.step_height * math.sin(math.pi * st)

            else:
                # Stance
                st = (t - half_cycle) / half_cycle

                dx = self.stride_x * (0.5 - st)
                dy = self.stride_y * (0.5 - st)
                dz = 0.0

        # Tripod B: legs 2,4,6
        else:

            if t < half_cycle:
                # Stance
                st = t / half_cycle

                dx = self.stride_x * (0.5 - st)
                dy = self.stride_y * (0.5 - st)
                dz = 0.0

            else:
                # Swing
                st = (t - half_cycle) / half_cycle

                dx = self.stride_x * (st - 0.5)
                dy = self.stride_y * (st - 0.5)
                dz = self.step_height * math.sin(math.pi * st)

        return dx, dy, dz

    def leg_local_target(self, leg_id, t):
      
        mount = self.mounts[leg_id]
        dx_body, dy_body, dz = self.body_frame_foot_offset(leg_id, t)

        # Rotate the body-frame offset into the leg's local frame by
        # applying the inverse of the mount rotation.
        cos_a = math.cos(-mount.mount_yaw)
        sin_a = math.sin(-mount.mount_yaw)
        dx_leg = dx_body * cos_a - dy_body * sin_a
        dy_leg = dx_body * sin_a + dy_body * cos_a

        # Add the neutral standing offset (foot's resting position in
        # this leg's local frame) on top of the gait-driven delta.
        x = self.neutral_reach + dx_leg
        y = dy_leg
        z = self.neutral_z + dz

        return x, y, z

    def update(self):
        positions = [0.0] * len(JOINT_NAMES)

        for leg_id in ALL_LEGS:
            x, y, z = self.leg_local_target(leg_id, self.t)
            if leg_id == 81:
                L3 = self.L3 + 0.09
            else:
                L3 = self.L3

            theta1_deg, theta2_deg, theta3_deg = ik(
                [0.0, 0.0, 0.0],
                [x, y, z],
                L2=self.L2,
                L3=L3
            )
            theta1 = math.radians(theta1_deg)
            theta2 = math.radians(theta2_deg)
            theta3 = math.radians(theta3_deg)

            idx = (leg_id - 1) * 3
            positions[idx + 0] = theta1
            positions[idx + 1] = theta2
            positions[idx + 2] = theta3

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        msg.points = [point]
        self.cmd_pub.publish(msg)

        self.t += self.dt
        if self.t >= self.cycle_time:
            self.t -= self.cycle_time


def main(args=None):
    rclpy.init(args=args)
    node = TripodGait()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()