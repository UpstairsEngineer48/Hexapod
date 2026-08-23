#!/usr/bin/env python3


import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

GRP1=[['hexa_Joint1_1', 'hexa_Joint1_2', 'hexa_Joint1_3'], ['hexa_Joint3_1', 'hexa_Joint3_2', 'hexa_Joint3_3'], ['hexa_Joint5_1', 'hexa_Joint5_2', 'hexa_Joint5_3']]
GRP2=[['hexa_Joint2_1', 'hexa_Joint2_2', 'hexa_Joint2_3'], ['hexa_Joint4_1', 'hexa_Joint4_2', 'hexa_Joint4_3'], ['hexa_Joint6_1', 'hexa_Joint6_2', 'hexa_Joint6_3']]
JOINT_NAMES = ['hexa_Joint1_1', 'hexa_Joint1_2', 'hexa_Joint1_3', 'hexa_Joint2_1', 'hexa_Joint2_2', 'hexa_Joint2_3', 'hexa_Joint3_1', 'hexa_Joint3_2', 'hexa_Joint3_3', 'hexa_Joint4_1', 'hexa_Joint4_2', 'hexa_Joint4_3', 'hexa_Joint5_1', 'hexa_Joint5_2', 'hexa_Joint5_3', 'hexa_Joint6_1', 'hexa_Joint6_2', 'hexa_Joint6_3']
#[['hexa_Joint1_1', 'hexa_Joint1_2', 'hexa_Joint1_3'], ['hexa_Joint2_1', 'hexa_Joint2_2', 'hexa_Joint2_3'], ['hexa_Joint3_1', 'hexa_Joint3_2', 'hexa_Joint3_3'], ['hexa_Joint4_1', 'hexa_Joint4_2', 'hexa_Joint4_3'], ['hexa_Joint5_1', 'hexa_Joint5_2', 'hexa_Joint5_3'], ['hexa_Joint6_1', 'hexa_Joint6_2', 'hexa_Joint6_3']]
#for leg in range(1, 7):
#   JOINT_NAMES += [f"hexa_Joint{leg}_1", f"hexa_Joint{leg}_2", f"hexa_Joint{leg}_3"]


class LegTest(Node):
    def __init__(self):
        super().__init__("leg_test")

        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            "/hexapod_joint_trajectory_controller/joint_trajectory",
            10
        )

        self.timer = self.create_timer(
            0.01,
            self.swing
        )
        self.t=0
        self.dt=0.01
        self.step=0
    def swing(self):
        positions = [0.0] * len(JOINT_NAMES)
        trajectory=math.sin(2*math.pi*self.t)


        if self.step%2==0:
            #swing
            alpha=-50*trajectory
            positions[4]=positions[10]=positions[16]=math.radians(alpha)
            #stance
            gamma=-90*trajectory
            positions[0]=math.radians(gamma)
            positions[6]=positions[12]=math.radians(-0.5*gamma)
        else:
            #swing
            alpha=-50*trajectory
            positions[1]=positions[7]=positions[13]=math.radians(alpha)
            #stance
            gamma=-90*trajectory
            positions[3]=math.radians(gamma)
            positions[9]=positions[15]=math.radians(-0.5*gamma)


        #publishing
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        msg.points = [point]
        self.cmd_pub.publish(msg)


        #updations
        self.t += self.dt
        if self.t >= 1.0:
            self.t -= 1.0  # reset for next cycle
            self.step += 1
def main(args=None):
    rclpy.init(args=args)
    node = LegTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()