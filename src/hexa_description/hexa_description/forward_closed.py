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

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))

def quat_to_yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))

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

        self.imu_topic="/imu"
        self.imu_sub=self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos_profile_sensor_data)

        self.dt=0.01
        self.L2=0.08
        self.L3=0.15

        #Values that can be tuned for gait movement
        self.yaw_target=0.0
        self.body_vx=0.1
        self.body_vy=0.0
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

    def imu_cb(self, msg):
        self.yaw=quat_to_yaw(msg.orientation)
        self.gyro_z=msg.angular_velocity.z
        if self.yaw_target is None:
            self.yaw_target=self.yaw
            self.get_logger().info(f"Yaw hold engaged, target={math.degrees(self.yaw_target):.1f} deg")

    def compute_yaw_correction(self):
        if self.yaw is None or self.yaw_target is None:
            return 0.0
        err=wrap_pi(self.yaw_target-self.yaw)
        if abs(err)<self.dband:
            err=0.0
        self.yi=clamp(self.yi+self.ki*err*self.dt, -self.i_max, self.i_max)
        wz=self.kp*err+self.yi-self.kd*self.gyro_z
        return clamp(wz, -self.max_wz, self.max_wz)

    def leg_stride(self, leg_id):
        #Per-leg stride=(body velocity+wz x r_foot)*cycle_time, so outer legs sweep more and inner legs less
        m=self.mounts[leg_id]
        rx=m.mount_xy[0]+self.neutral_reach*math.cos(m.mount_yaw)
        ry=m.mount_xy[1]+self.neutral_reach*math.sin(m.mount_yaw)
        vx=self.body_vx-self.wz_cmd*ry
        vy=self.body_vy+self.wz_cmd*rx
        return vx*self.cycle_time, vy*self.cycle_time

    def body_frame_foot_offset(self, leg_id, t):
        half_cycle=self.cycle_time/2
        stride_x,stride_y=self.leg_stride(leg_id)
        swinging=(t<half_cycle) if leg_id in (1,3,5) else (t>=half_cycle)
        st=(t%half_cycle)/half_cycle

        if swinging:
            dx=stride_x*(st-0.5)
            dy=stride_y*(st-0.5)
            dz=self.step_height*math.sin(math.pi*st)
        else:
            dx=stride_x*(0.5-st)
            dy=stride_y*(0.5-st)
            dz=0.0

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
        self.wz_cmd=self.compute_yaw_correction()
        positions=[0.0]*len(JOINT_NAMES)
        for leg_id in ALL_LEGS:
            x,y,z=self.leg_local_target(leg_id, self.t)
            L3=self.L3+0.09 if leg_id==5 else self.L3
            t1,t2,t3=ik([0.0, 0.0, 0.0],[x, y, z],L2=self.L2,L3=L3)
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