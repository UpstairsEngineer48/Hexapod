#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ik import ik

JOINT_NAMES=['hexa_Joint1_1', 'hexa_Joint1_2', 'hexa_Joint1_3',
             'hexa_Joint2_1', 'hexa_Joint2_2', 'hexa_Joint2_3',
             'hexa_Joint3_1', 'hexa_Joint3_2', 'hexa_Joint3_3',
             'hexa_Joint4_1', 'hexa_Joint4_2', 'hexa_Joint4_3',
             'hexa_Joint5_1', 'hexa_Joint5_2', 'hexa_Joint5_3',
             'hexa_Joint6_1', 'hexa_Joint6_2', 'hexa_Joint6_3']
ALL_LEGS=[1, 2, 3, 4, 5, 6]
TRIPOD_B=[2, 4, 6]
MOUNT_YAW={1: 2.3578, 2: 1.6011, 3: 0.80016, 4: -0.7472, 5: -1.5753, 6: -2.5044}

class SidewaysGait(Node):
    def __init__(self):
        super().__init__("sideways_gait")

        self.cmd_pub=self.create_publisher(
            JointTrajectory,
            "/hexapod_joint_trajectory_controller/joint_trajectory",
            10
        )

        self.dt=0.01
        self.L2=0.08
        self.L3=0.15

        #Values that can be tuned for gait movement
        #+ is body-left, - is body-right
        self.body_vx=0.0
        self.body_vy=0.08
        self.neutral_reach=0.18
        self.neutral_z=-0.155
        self.cycle_time=1.0
        self.step_height=0.08

        self.stride_x=self.body_vx*self.cycle_time
        self.stride_y=self.body_vy*self.cycle_time

        self.t=0.0
        self.timer=self.create_timer(self.dt, self.update)

    def body_frame_foot_offset(self, leg_id, t):
        half=self.cycle_time/2
        #tripod B runs half a cycle behind tripod A
        if leg_id in TRIPOD_B:
            t=(t+half)%self.cycle_time

        if t<half:
            p=t/half
            s=p-0.5
            dz=self.step_height*math.sin(math.pi*p)
        else:
            p=(t-half)/half
            s=0.5-p
            dz=0.0

        return self.stride_x*s, self.stride_y*s, dz

    def leg_local_target(self, leg_id, t):
        dx,dy,dz=self.body_frame_foot_offset(leg_id, t)
        a=-MOUNT_YAW[leg_id]
        dx_leg=dx*math.cos(a)-dy*math.sin(a)
        dy_leg=dx*math.sin(a)+dy*math.cos(a)

        return self.neutral_reach+dx_leg, dy_leg, self.neutral_z+dz

    def update(self):
        positions=[0.0]*len(JOINT_NAMES)
        for leg_id in ALL_LEGS:
            x,y,z=self.leg_local_target(leg_id, self.t)
            t1,t2,t3=ik([0.0, 0.0, 0.0],[x, y, z],L2=self.L2,L3=self.L3)
            idx=(leg_id-1)*3
            positions[idx]=math.radians(t1)
            positions[idx+1]=math.radians(t2)
            positions[idx+2]=math.radians(t3)

        msg=JointTrajectory()
        msg.joint_names=JOINT_NAMES
        point=JointTrajectoryPoint()
        point.positions=positions
        msg.points=[point]
        self.cmd_pub.publish(msg)
        #reset the timer for the next cycle
        self.t=(self.t+self.dt)%self.cycle_time


def main(args=None):
    rclpy.init(args=args)
    node=SidewaysGait()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()