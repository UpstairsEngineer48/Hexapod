#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
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
MOUNT_XY={1: (0.067149, 0.11215), 2: (0.08925, 0.0), 3: (0.067149, -0.11215),
          4: (-0.067149, -0.11215), 5: (-0.08925, 0.0), 6: (-0.067149, 0.11215)}

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))

def quat_to_yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))

class SidewaysGait(Node):
    def __init__(self):
        super().__init__("sideways_gait")

        self.cmd_pub=self.create_publisher(
            JointTrajectory,
            "/hexapod_joint_trajectory_controller/joint_trajectory",
            10
        )

        self.imu_topic="/imu"
        self.imu_sub=self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos_profile_sensor_data)

        self.dt=0.01
        self.L2=0.08
        self.L3=0.15

        #Values that can be tuned for gait movement
        #+ is body-left, - is body-right
        self.yaw_target=0.0
        self.body_vx=0.0
        self.body_vy=0.08
        self.neutral_reach=0.18
        self.neutral_z=-0.155
        self.cycle_time=1.0
        self.step_height=0.08

        #Values that can be tuned for yaw hold
        self.kp=1.5
        self.kd=0.15
        self.ki=0.3
        self.i_max=0.2
        self.max_wz=0.5
        self.dband=0.01

        self.yaw=None
        self.gyro_z=0.0
        self.wz_cmd=0.0
        self.yi=0.0

        self.t=0.0
        self.timer=self.create_timer(self.dt, self.update)

    def imu_cb(self, msg):
        self.yaw=quat_to_yaw(msg.orientation)
        self.gyro_z=msg.angular_velocity.z

    def compute_yaw_correction(self):
        if self.yaw is None:
            return 0.0
        err=wrap_pi(self.yaw_target-self.yaw)
        if abs(err)<self.dband:
            err=0.0
        self.yi=clamp(self.yi+self.ki*err*self.dt, -self.i_max, self.i_max)
        wz=self.kp*err+self.yi-self.kd*self.gyro_z
        return clamp(wz, -self.max_wz, self.max_wz)

    def leg_stride(self, leg_id):
        #stride=(body velocity+wz x foot position)*cycle_time
        mx,my=MOUNT_XY[leg_id]
        rx=mx+self.neutral_reach*math.cos(MOUNT_YAW[leg_id])
        ry=my+self.neutral_reach*math.sin(MOUNT_YAW[leg_id])
        vx=self.body_vx-self.wz_cmd*ry
        vy=self.body_vy+self.wz_cmd*rx
        return vx*self.cycle_time, vy*self.cycle_time

    def body_frame_foot_offset(self, leg_id, t):
        half=self.cycle_time/2
        stride_x,stride_y=self.leg_stride(leg_id)
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

        return stride_x*s, stride_y*s, dz

    def leg_local_target(self, leg_id, t):
        dx,dy,dz=self.body_frame_foot_offset(leg_id, t)
        a=-MOUNT_YAW[leg_id]
        dx_leg=dx*math.cos(a)-dy*math.sin(a)
        dy_leg=dx*math.sin(a)+dy*math.cos(a)

        return self.neutral_reach+dx_leg,dy_leg,self.neutral_z+dz

    def update(self):
        self.wz_cmd=self.compute_yaw_correction()
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