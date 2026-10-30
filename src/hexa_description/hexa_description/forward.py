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

class LegMount:
    def __init__(self, mount_xy, mount_yaw):
        self.mount_xy=mount_xy
        self.mount_yaw=mount_yaw

class TripodGait(Node):
    def __init__(self):
        super().__init__("tripod_gait")

        self.cmd_pub=self.create_publisher(
            JointTrajectory,
            "/hexapod_joint_trajectory_controller/joint_trajectory",
            10
        )

        self.dt=0.01
        self.L2=0.08
        self.L3=0.15

        #Values that can be tuned for gait movement 
        self.body_vx=0.1
        self.body_vy=0.0
        self.neutral_reach=0.18
        self.neutral_z=-0.155
        self.cycle_time=1.0
        self.step_height=0.08

        self.stride_x=self.body_vx*self.cycle_time
        self.stride_y=self.body_vy*self.cycle_time

        self.mounts={
            1: LegMount((0.067149, 0.11215), 2.3578),
            2: LegMount((0.08925, 0.0), 1.6011),
            3: LegMount((0.067149, -0.11215), 0.80016),
            4: LegMount((-0.067149, -0.11215), -0.7472),
            5: LegMount((-0.08925, 0.0), -1.5753),
            6: LegMount((-0.067149, 0.11215), -2.5044),
        }

        self.t=0.0
        self.timer=self.create_timer(self.dt, self.update)

    def body_frame_foot_offset(self, leg_id, t):
        half_cycle=self.cycle_time/2
        if leg_id in (1,3,5):
            if t<half_cycle:
                st=t/half_cycle
                dx=self.stride_x*(st-0.5)
                dy=self.stride_y*(st-0.5)
                dz=self.step_height*math.sin(math.pi*st)
            else:
                st=(t-half_cycle)/half_cycle
                dx=self.stride_x*(0.5-st)
                dy=self.stride_y*(0.5-st)
                dz=0.0
        else:
            if t<half_cycle:
                st=t/half_cycle
                dx=self.stride_x*(0.5-st)
                dy=self.stride_y*(0.5-st)
                dz=0.0
            else:
                st=(t-half_cycle)/half_cycle
                dx=self.stride_x*(st-0.5)
                dy=self.stride_y*(st-0.5)
                dz=self.step_height*math.sin(math.pi*st)

        return dx, dy, dz

    def leg_local_target(self, leg_id, t):
        mount=self.mounts[leg_id]
        dx_body,dy_body,dz=self.body_frame_foot_offset(leg_id, t)

        cos_a=math.cos(-mount.mount_yaw)
        sin_a=math.sin(-mount.mount_yaw)
        dx_leg=dx_body*cos_a-dy_body*sin_a
        dy_leg=dx_body*sin_a+dy_body*cos_a

        return self.neutral_reach+dx_leg, dy_leg, self.neutral_z+dz

    def update(self):
        positions=[0.0]*len(JOINT_NAMES)
        for leg_id in ALL_LEGS:
            x,y,z=self.leg_local_target(leg_id,self.t)
            if leg_id==5:
                L3=self.L3+0.09
            else:
                L3=self.L3
            t1,t2,t3=ik([0.0, 0.0, 0.0],[x, y, z],L2=self.L2, L3=L3)
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
        self.t+=self.dt
        if self.t>=self.cycle_time:
            self.t-=self.cycle_time


def main(args=None):
    rclpy.init(args=args)
    node=TripodGait()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()