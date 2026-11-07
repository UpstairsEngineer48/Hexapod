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

Switching gait while running: publish the gait name on /gait_cmd (std_msgs/String,
"tripod" or "ripple"), e.g. press 1 / 2 in pretty_teleop.py, or
  ros2 topic pub --once /gait_cmd std_msgs/msg/String "{data: ripple}"
The robot slows to a stop, blends its stance into the new gait, then carries on
with whatever /cmd_vel is being sent. The current gait is published on /gait_state.

Needs: pip install rich
"""
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rich import box
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from ik import ik
import ik as ik_mod

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
#neutral_reach=foot distance from the hip at mid-stance (must leave room to sweep, L2+L3=0.23)
#l3_extra=per-leg extra shank length used in ik() (empirical fix, copied from the *_closed.py gaits)
#yaw_gain=scales the yaw-hold gains (1.0 = kp 1.5, kd 0.15, ki 0.3, i_max 0.2, max_corr 0.5 rad/s).
#ik_offsets=True uses the offset angles in ik.py (bot stands at the proper height, slower), False uses the raw angles (bot hugs the ground, faster)
#stride_cap=max stride in m. None=use the geometric reach limit (ok for ripple). The tripod uses neutral_reach 0.18,
#which is already beyond the geometric limit (~0.16), so the geometric limit would collapse its strides to 0/negative.
#0.10 is the stride the working open-loop forward.py used.
GAITS={
    "tripod": {"cycle_time": 1.0, "swing_frac": 0.5, "step_height": 0.08, "ease": False,
               "phase": {1: 0.0, 3: 0.0, 5: 0.0, 2: 0.5, 4: 0.5, 6: 0.5},
               "neutral_reach": 0.18, "l3_extra": {5: 0.09}, "yaw_gain": 1.0, "stride_cap": 0.10, "ik_offsets": False},
    "ripple": {"cycle_time": 1.2, "swing_frac": 1/3, "step_height": 0.06, "ease": True,
               "phase": {1: 0.0, 3: 1/6, 5: 2/6, 2: 3/6, 6: 4/6, 4: 5/6},
               "neutral_reach": 0.10, "l3_extra": {2: 0.04, 5: 0.04}, "yaw_gain": 1.0, "stride_cap": None, "ik_offsets": True},
}

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))

def quat_to_yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))

def slew(cur, target, max_step):
    return cur+clamp(target-cur, -max_step, max_step)

def ask_choice(title, options, default):
    #pretty terminal menu, falls back to default if there is no terminal (e.g. launch file)
    if not sys.stdin.isatty():
        return default
    console=Console()
    keys=[k for k,_ in options]
    table=Table(box=box.SIMPLE_HEAD, header_style="bold cyan")
    table.add_column("#", justify="center")
    table.add_column("Option", style="bold")
    table.add_column("Description")
    for i,(k,desc) in enumerate(options):
        table.add_row(str(i), k+("  ★" if k==default else ""), desc)
    console.print(Panel(table, title=f"[bold]{title}[/]", border_style="bright_blue", expand=False))
    while True:
        try:
            ans=console.input(f"[cyan]Choose 0-{len(keys)-1}[/] [dim](Enter = {default})[/]: ").strip().lower()
        except EOFError:
            return default
        if ans=="":
            return default
        if ans.isdigit() and int(ans)<len(keys):
            return keys[int(ans)]
        if ans in keys:
            return ans
        console.print("[red]Not a valid choice.[/]")

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

        self.apply_gait(gait)
        self.get_logger().info(f"Gait={gait}, frame={frame}")

        self.imu_topic="/imu"
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(String,"/gait_cmd",self.gait_cb,10)
        state_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub=self.create_publisher(String,"/gait_state",state_qos)

        self.dt=0.01
        self.L2=0.08
        self.L3=0.15

        #Values that can be tuned for gait movement
        #(neutral_reach, l3_extra and yaw_gain are per gait, see GAITS above)
        self.neutral_z=-0.155
        self.reach_margin=0.97
        self.max_vx=0.15
        self.max_vy=0.10
        self.max_wz=0.8
        self.lin_acc=0.3
        self.ang_acc=1.5
        self.timeout=0.5
        self.debug=True

        #Yaw hold gains (kp, kd, ki, i_max, max_corr) are set per gait in apply_gait()
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

        #gait switching: idle -> stopping -> blending -> idle
        self.switch_state="idle"
        self.pending_gait=None
        self.blend_time=1.0
        self.blend_alpha=1.0
        self.blend_from=None
        self.last_positions=None

        self.t=0.0
        self.create_timer(self.dt, self.update)
        self.state_pub.publish(String(data=self.gait_name))

    def apply_gait(self, name):
        #load one gait's parameters (used at start-up and on every switch)
        g=GAITS[name]
        self.gait_name=name
        self.cycle_time=g["cycle_time"]
        self.swing_frac=g["swing_frac"]
        self.step_height=g["step_height"]
        self.ease=g["ease"]
        self.phase=g["phase"]
        self.neutral_reach=g["neutral_reach"]
        self.l3_extra=g["l3_extra"]
        self.stride_cap=g["stride_cap"]
        ik_mod.set_offsets(g["ik_offsets"])
        yg=g["yaw_gain"]
        self.stance_time=self.cycle_time*(1-self.swing_frac)
        #Values that can be tuned for yaw hold (scaled by the gait's yaw_gain)
        self.kp=1.5*yg
        self.kd=0.15*yg
        self.ki=0.3*yg
        self.i_max=0.2*yg
        self.max_corr=0.5*yg

    def gait_cb(self, msg):
        name=msg.data.strip().lower()
        if name not in GAITS:
            self.get_logger().warn(f"Unknown gait '{msg.data}', choose from {list(GAITS)}")
            return
        if self.switch_state=="idle":
            if name==self.gait_name:
                return
            self.switch_state="stopping"
            self.get_logger().info(f"Switching {self.gait_name} -> {name}: slowing to a stop first")
        #latest request wins, also while a switch is already in progress
        self.pending_gait=name

    def begin_blend(self):
        #robot is standing still: swap the gait parameters and blend the stance
        new=self.pending_gait
        self.pending_gait=None
        if new is None or new==self.gait_name:
            self.switch_state="idle"
            return
        self.apply_gait(new)
        self.t=0.0
        self.yi=0.0
        if self.yaw is not None:
            self.yaw_target=self.yaw
        self.state_pub.publish(String(data=new))
        self.get_logger().info(f"Gait={new}, blending stance")
        if self.last_positions is None:
            self.switch_state="idle"
            return
        self.blend_from=self.last_positions
        self.blend_alpha=0.0
        self.switch_state="blending"

    def finish_blend(self):
        self.blend_from=None
        if self.pending_gait is not None and self.pending_gait!=self.gait_name:
            #another change was requested during the blend
            self.switch_state="stopping"
        else:
            self.pending_gait=None
            self.switch_state="idle"
            self.get_logger().info(f"Gait {self.gait_name} ready")

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
        return max(0.0, 2*(math.sqrt(d)-cu))

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
                lim=self.stride_cap if self.stride_cap is not None else self.reach_limit(l, sx, sy)
                k=min(k, lim/L)
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

        #while the gait is changing, ask for zero velocity so the robot stops first
        if self.switch_state!="idle":
            tvx=tvy=twz=0.0

        #smooth the command (feet are position-driven, so a velocity step = a foot jump)
        la=self.lin_acc*self.dt
        aa=self.ang_acc*self.dt
        self.vx=slew(self.vx, tvx, la)
        self.vy=slew(self.vy, tvy, la)
        self.wz=slew(self.wz, twz, aa)

        #stopped: swap to the new gait and start blending the stance
        if self.switch_state=="stopping" and max(abs(self.vx),abs(self.vy),abs(self.wz))<1e-3:
            self.vx=self.vy=self.wz=0.0
            self.begin_blend()

        #yaw hold only when no turn is requested, re-latch the target when a turn ends
        #(off while the gait is changing)
        if self.switch_state!="idle":
            self.wz_hold=0.0
        elif abs(self.cmd_wz)>1e-3:
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
            sat=" SATURATED" if abs(self.wz_hold)>=self.max_corr-1e-3 else ""
            info=f"yaw={math.degrees(self.yaw):.1f} target={math.degrees(self.yaw_target):.1f} err={err:.1f} deg  wz_hold={self.wz_hold:.3f}/{self.max_corr:.2f} rad/s{sat}"
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

        #blend from the old gait's stance to the new one (joint space, smoothstep)
        if self.switch_state=="blending":
            self.blend_alpha=min(1.0, self.blend_alpha+self.dt/self.blend_time)
            a=self.blend_alpha
            a=a*a*(3-2*a)
            positions=[f+(n-f)*a for f,n in zip(self.blend_from,positions)]
            if self.blend_alpha>=1.0:
                self.finish_blend()
        self.last_positions=positions

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