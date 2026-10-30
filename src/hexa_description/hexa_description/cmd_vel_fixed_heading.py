#!/usr/bin/env python3
"""
Omnidirectional hexapod gait driven by /cmd_vel (geometry_msgs/Twist).
  linear.x  -> forward/back  (m/s)
  linear.y  -> left/right    (m/s, + = left)
  angular.z -> turn          (rad/s, + = CCW)

At start-up it shows a menu asking which gait (tripod / ripple) and which
command frame (world / body). Type 0 or 1, or just press Enter for the default
(tripod, world). Without a terminal it also takes the defaults.

Frames:
  body  : x/y are relative to where the robot is facing.
  world : x/y are fixed room directions, defined by the robot's heading at
          start-up (x = the way it faced when the node got its first IMU msg).
Yaw hold keeps the heading while angular.z == 0.
"""
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
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
MOUNT_YAW={1: 2.3578, 2: 1.6011, 3: 0.80016, 4: -0.7472, 5: -1.5753, 6: -2.5044}
MOUNT_XY={1: (0.067149, 0.11215), 2: (0.08925, 0.0), 3: (0.067149, -0.11215),
          4: (-0.067149, -0.11215), 5: (-0.08925, 0.0), 6: (-0.067149, 0.11215)}

#swing_frac=fraction of the cycle a leg is in the air, phase=when each leg starts its swing (0..1)
#ease=cosine-eased swing (zero foot speed at lift-off/touch-down)
GAITS={
    "tripod": {"cycle_time": 1.0, "swing_frac": 0.5, "step_height": 0.08, "ease": False, "phase": {1: 0.0, 3: 0.0, 5: 0.0, 2: 0.5, 4: 0.5, 6: 0.5}},
    "ripple": {"cycle_time": 1.2, "swing_frac": 1/3, "step_height": 0.06, "ease": True, "phase": {1: 0.0, 3: 1/6, 5: 2/6, 2: 3/6, 6: 4/6, 4: 5/6}},
}

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))

def quat_to_yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))

def slew(cur, target, max_step):
    return cur+clamp(target-cur, -max_step, max_step)

def ask_choice(question, options, default):
    #terminal menu, falls back to default if there is no terminal (e.g. launch file)
    if not sys.stdin.isatty():
        return default
    keys=[k for k,_ in options]
    print(f"\n{question}")
    for i,(k,desc) in enumerate(options):
        print(f"  {i}) {k:<7} - {desc}")
    while True:
        try:
            ans=input(f"Choose 0-{len(options)-1} (Enter = {default}): ").strip().lower()
        except EOFError:
            return default
        if ans=="":
            return default
        if ans.isdigit() and int(ans)<len(options):
            return keys[int(ans)]
        if ans in keys:
            return ans
        print("  Not a valid choice.")

class CmdVelGait(Node):
    def __init__(self):
        super().__init__("cmd_vel_gait")

        self.cmd_pub=self.create_publisher(
            JointTrajectory,
            "/hexapod_joint_trajectory_controller/joint_trajectory",
            10
        )

        gait_opts=[("tripod", "faster, 3 legs on the ground"), ("ripple", "slower, steadier, 4+ legs on the ground")]
        frame_opts=[("world", "x/y fixed in the room (start-up heading = x)"), ("body", "x/y relative to where the robot faces")]
        gait=ask_choice("Which gait?", gait_opts, "tripod")
        frame=ask_choice("Which command frame?", frame_opts, "world")
        self.world_frame=(frame=="world")

        g=GAITS[gait]
        self.cycle_time=g["cycle_time"]
        self.swing_frac=g["swing_frac"]
        self.step_height=g["step_height"]
        self.ease=g["ease"]
        self.phase=g["phase"]
        self.stance_time=self.cycle_time*(1-self.swing_frac)
        self.get_logger().info(f"Gait={gait}, frame={frame}")

        self.imu_topic="/imu"
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos_profile_sensor_data)

        self.dt=0.01
        self.L2=0.08
        self.L3=0.15

        #Values that can be tuned for gait movement
        #neutral_reach must leave room to sweep (L2+L3=0.23)
        self.neutral_reach=0.10
        self.neutral_z=-0.155
        self.reach_margin=0.97
        self.max_vx=0.15
        self.max_vy=0.10
        self.max_wz=0.8
        self.lin_acc=0.3
        self.ang_acc=1.5
        self.timeout=0.5
        self.debug=True

        #Values that can be tuned for yaw hold
        self.kp=1.5
        self.kd=0.15
        self.ki=0.3
        self.i_max=0.2
        self.max_corr=0.5
        self.dband=0.01

        R=self.reach_margin*(self.L2+self.L3)
        self.Rh=math.sqrt(R*R-self.neutral_z**2)

        self.cmd_vx=self.cmd_vy=self.cmd_wz=0.0
        self.vx=self.vy=self.wz=0.0
        self.last_cmd_time=self.get_clock().now()

        self.yaw=None
        self.yaw_ref=None
        self.gyro_z=0.0
        self.yaw_target=None
        self.wz_hold=0.0
        self.yi=0.0

        self.strides={l: (0.0, 0.0) for l in ALL_LEGS}
        self.lift_scale=0.0

        self.t=0.0
        self.create_timer(self.dt, self.update)

    def cmd_cb(self, msg):
        self.cmd_vx=msg.linear.x
        self.cmd_vy=msg.linear.y
        self.cmd_wz=msg.angular.z
        self.last_cmd_time=self.get_clock().now()

    def imu_cb(self, msg):
        self.yaw=quat_to_yaw(msg.orientation)
        self.gyro_z=msg.angular_velocity.z
        if self.yaw_target is None:
            self.yaw_target=self.yaw
            self.yaw_ref=self.yaw
            self.get_logger().info(f"IMU OK, yaw hold target={math.degrees(self.yaw_target):.1f} deg")

    def yaw_hold_correction(self):
        if self.yaw is None or self.yaw_target is None:
            return 0.0
        err=wrap_pi(self.yaw_target-self.yaw)
        if abs(err)<self.dband:
            err=0.0
        self.yi=clamp(self.yi+self.ki*err*self.dt, -self.i_max, self.i_max)
        wz=self.kp*err+self.yi-self.kd*self.gyro_z
        return clamp(wz, -self.max_corr, self.max_corr)

    def reach_limit(self, leg_id, sx, sy):
        #longest stride along (sx,sy) this leg can sweep without ik() clamping
        L=math.hypot(sx, sy)
        if L<1e-9:
            return 1e9
        a=-MOUNT_YAW[leg_id]
        ux=(sx*math.cos(a)-sy*math.sin(a))/L
        c=self.neutral_reach
        cu=abs(c*ux)
        d=cu*cu-c*c+self.Rh*self.Rh
        if d<=0:
            return 0.0
        return 2*(math.sqrt(d)-cu)

    def update_strides(self, vx, vy, wz):
        #stride=(body velocity+wz x r_foot)*stance_time, uniformly scaled down if any leg would be pushed out of reach
        raw={}
        k=1.0
        for l in ALL_LEGS:
            mx,my=MOUNT_XY[l]
            yaw=MOUNT_YAW[l]
            rx=mx+self.neutral_reach*math.cos(yaw)
            ry=my+self.neutral_reach*math.sin(yaw)
            sx=(vx-wz*ry)*self.stance_time
            sy=(vy+wz*rx)*self.stance_time
            raw[l]=(sx,sy)
            L=math.hypot(sx, sy)
            if L>1e-9:
                k=min(k, self.reach_limit(l, sx, sy)/L)
        self.strides={l: (sx*k, sy*k) for l,(sx,sy) in raw.items()}
        biggest=max(math.hypot(*s) for s in self.strides.values())
        #fade the foot lift in/out so a stopped robot stands still instead of marching
        self.lift_scale=clamp(biggest/0.01, 0.0, 1.0)

    def foot_offset(self, leg_id, t):
        sx,sy=self.strides[leg_id]
        u=(t/self.cycle_time-self.phase[leg_id])%1.0

        if u<self.swing_frac:
            p=u/self.swing_frac
            s=-0.5+(0.5*(1-math.cos(math.pi*p)) if self.ease else p)
            dz=self.step_height*math.sin(math.pi*p)*self.lift_scale
        else:
            #constant speed so all grounded feet agree
            p=(u-self.swing_frac)/(1-self.swing_frac)
            s=0.5-p
            dz=0.0
        return sx*s, sy*s, dz

    def leg_local_target(self, leg_id, t):
        dx,dy,dz=self.foot_offset(leg_id, t)
        a=-MOUNT_YAW[leg_id]
        dx_leg=dx*math.cos(a)-dy*math.sin(a)
        dy_leg=dx*math.sin(a)+dy*math.cos(a)
        return self.neutral_reach+dx_leg, dy_leg, self.neutral_z+dz

    def update(self):
        #watchdog
        age=(self.get_clock().now()-self.last_cmd_time).nanoseconds*1e-9
        if age>self.timeout:
            self.cmd_vx=self.cmd_vy=self.cmd_wz=0.0

        #world frame -> body frame (rotate by how far the robot has turned since start)
        cvx,cvy=self.cmd_vx,self.cmd_vy
        if self.world_frame and self.yaw is not None and self.yaw_ref is not None:
            a=wrap_pi(self.yaw-self.yaw_ref)
            c,s=math.cos(a),math.sin(a)
            cvx=c*self.cmd_vx+s*self.cmd_vy
            cvy=-s*self.cmd_vx+c*self.cmd_vy

        #scale vx/vy together so over-limit commands keep their direction
        k=max(1.0, abs(cvx)/self.max_vx, abs(cvy)/self.max_vy)
        tvx=cvx/k
        tvy=cvy/k
        twz=clamp(self.cmd_wz, -self.max_wz, self.max_wz)

        #smooth the command (feet are position-driven, so a velocity step = a foot jump)
        la=self.lin_acc*self.dt
        aa=self.ang_acc*self.dt
        self.vx=slew(self.vx, tvx, la)
        self.vy=slew(self.vy, tvy, la)
        self.wz=slew(self.wz, twz, aa)

        #yaw hold only when no turn is requested, re-latch the target when a turn ends
        if abs(self.cmd_wz)>1e-3:
            self.yaw_target=self.yaw
            self.wz_hold=0.0
            self.yi=0.0
        else:
            self.wz_hold=self.yaw_hold_correction()

        self.update_strides(self.vx, self.vy, self.wz+self.wz_hold)

        if self.yaw is None:
            warn="No IMU data received -> yaw hold is OFF"+(" and world frame is acting as body frame" if self.world_frame else "")+" (check imu_topic)"
            self.get_logger().warn(warn, throttle_duration_sec=2.0)
        elif self.debug and (abs(self.vx)+abs(self.vy))>1e-3:
            err=math.degrees(wrap_pi(self.yaw_target-self.yaw))
            info=f"yaw={math.degrees(self.yaw):.1f} target={math.degrees(self.yaw_target):.1f} err={err:.1f} deg  wz_hold={self.wz_hold:.3f} rad/s"
            self.get_logger().info(info, throttle_duration_sec=1.0)

        positions=[0.0]*len(JOINT_NAMES)
        for leg_id in ALL_LEGS:
            x,y,z=self.leg_local_target(leg_id, self.t)
            L3=self.L3+0.04 if leg_id==55 else self.L3
            t1,t2,t3=ik([0.0, 0.0, 0.0],[x, y, z],L2=self.L2,L3=L3)
            idx=(leg_id-1)*3
            positions[idx]=math.radians(t1)
            positions[idx+1]=math.radians(t2)
            positions[idx+2]=math.radians(t3)

        msg=JointTrajectory()
        msg.joint_names=JOINT_NAMES
        pt=JointTrajectoryPoint()
        pt.positions=positions
        msg.points=[pt]
        self.cmd_pub.publish(msg)
        #reset the timer for the next cycle
        self.t=(self.t+self.dt)%self.cycle_time


def main(args=None):
    rclpy.init(args=args)
    node=CmdVelGait()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()