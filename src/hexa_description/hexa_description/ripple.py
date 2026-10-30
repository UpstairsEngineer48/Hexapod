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
MOUNT_YAW={1: 2.3578, 2: 1.6011, 3: 0.80016, 4: -0.7472, 5: -1.5753, 6: -2.5044}
#swing order 1,3,5,2,6,4, 1/6 of a cycle apart -> 2 legs airborne at a time, never adjacent
PHASE={1: 0.0, 3: 1/6, 5: 2/6, 2: 3/6, 6: 4/6, 4: 5/6}

class RippleGait(Node):
    def __init__(self):
        super().__init__("ripple_gait")

        self.cmd_pub=self.create_publisher(
            JointTrajectory,
            "/hexapod_joint_trajectory_controller/joint_trajectory",
            10
        )

        self.dt=0.01
        self.L2=0.08
        self.L3=0.15

        #Values that can be tuned for gait movement
        #body velocity is scaled down automatically if the legs cannot sweep that far
        self.body_vx=0.12
        self.body_vy=0.0
        #neutral_reach must leave room to sweep (L2+L3=0.23)
        self.neutral_reach=0.10
        self.neutral_z=-0.155
        self.cycle_time=1.2
        self.step_height=0.06
        self.swing_frac=1/3
        self.reach_margin=0.97

        self.stance_time=self.cycle_time*(1-self.swing_frac)
        self.stride_x,self.stride_y=self.fit_stride()

        self.t=0.0
        self.timer=self.create_timer(self.dt, self.update)

    def fit_stride(self):
        #largest stride (<= requested) every leg can sweep without ik() clamping, shared by all legs
        R=self.reach_margin*(self.L2+self.L3)
        disc=R*R-self.neutral_z**2
        if disc<=0:
            self.get_logger().error("neutral_z is out of reach, lower |neutral_z|")
            return 0.0,0.0
        Rh=math.sqrt(disc)

        speed=math.hypot(self.body_vx, self.body_vy)
        want=speed*self.stance_time
        if want<1e-9:
            return 0.0,0.0
        dirx,diry=self.body_vx/speed,self.body_vy/speed

        c=self.neutral_reach
        limit=1e9
        for leg_id in ALL_LEGS:
            a=-MOUNT_YAW[leg_id]
            ux=dirx*math.cos(a)-diry*math.sin(a)
            cu=abs(c*ux)
            d=cu*cu-c*c+Rh*Rh
            if d<=0:
                limit=0.0
                break
            limit=min(limit, 2*(math.sqrt(d)-cu))

        stride=min(want, limit)
        self.get_logger().info(f"stride requested {want:.3f} m, max reachable {limit:.3f} m, using {stride:.3f} m -> body speed {stride/self.stance_time:.3f} m/s")
        return stride*dirx,stride*diry

    def body_frame_foot_offset(self, leg_id, t):
        u=(t/self.cycle_time-PHASE[leg_id])%1.0

        if u<self.swing_frac:
            #cosine ease: ~zero horizontal speed at lift-off and touch-down
            p=u/self.swing_frac
            s=-0.5+0.5*(1-math.cos(math.pi*p))
            dz=self.step_height*math.sin(math.pi*p)
        else:
            #constant speed so all grounded feet agree
            p=(u-self.swing_frac)/(1-self.swing_frac)
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
            L3=self.L3+0.04 if leg_id in [2, 5] else self.L3
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
        self.t=(self.t+self.dt)%self.cycle_time


def main(args=None):
    rclpy.init(args=args)
    node=RippleGait()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()