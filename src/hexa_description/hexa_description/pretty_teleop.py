#!/usr/bin/env python3
"""
Pretty arrow-key teleop for cmd_vel_fixed_heading.py  (pip install rich)

  Up / Down            walk forward / backward
  Shift+Up / Shift+Down  speed +10% / -10%
  Left / Right         turn left / right (constant turn rate)
  A / D                strafe left / right (hexapod is omnidirectional)
  1 / 2                switch gait: tripod / ripple (works while walking)
  Space                stop
  Q                    quit

Terminals have no "key released" event, so a key counts as held while its
auto-repeat keeps arriving. The first press is given a longer grace period to
bridge the OS key-repeat delay. Only the most recently pressed motion key is
active (terminals only repeat the last key).
"""
import re
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rich import box
from rich.console import Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

# ---- tunables -------------------------------------------------------------
START_SPEED = 0.08     # m/s
MIN_SPEED = 0.01
MAX_SPEED = 0.15       # keep <= max_vx in cmd_vel_fixed_heading.py
SPEED_STEP = 0.10      # 10 % per Shift+Up/Down
TURN_RATE = 0.5        # rad/s, constant (keep <= max_wz = 0.8)
FIRST_HOLD = 0.60      # s, grace after first press (covers key-repeat delay)
REPEAT_HOLD = 0.15     # s, grace once auto-repeat is flowing
PUB_HZ = 20.0
# ---------------------------------------------------------------------------

# \x1b[A  \x1b[1;2A  \x1bOA  \x1b[a(rxvt shift)  or a plain character
KEY_RE = re.compile(r"\x1b(?:\[1;(\d+)|\[|O)([A-Da-d])|(.)", re.S)
ARROW = {"A": "up", "B": "down", "C": "right", "D": "left"}


def parse_keys(data):
    """-> list of (name, shift) from a raw byte string."""
    out = []
    for m in KEY_RE.finditer(data):
        mod, letter, ch = m.groups()
        if letter:
            shift = bool((int(mod) - 1) & 1) if mod else letter.islower()
            out.append((ARROW[letter.upper()], shift))
        elif ch:
            out.append((ch.lower(), False))
    return out


class PrettyTeleop(Node):
    def __init__(self):
        super().__init__("pretty_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.speed = START_SPEED
        self.held = None          # [action, count, last_time]
        self.gait_pub=self.create_publisher(String,"/gait_cmd",10)
        state_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String,"/gait_state",self.gait_state_cb,state_qos)
        self.gait=None            # gait the robot reports it is using
        self.requested=None       # gait we asked for, until the robot confirms

    def gait_state_cb(self,msg):
        self.gait=msg.data
        if self.requested==self.gait:
            self.requested=None

    # ---- input ------------------------------------------------------------
    def handle_key(self, name, shift, now):
        if name in ("up", "down") and shift:
            f = 1 + SPEED_STEP if name == "up" else 1 - SPEED_STEP
            self.speed = min(MAX_SPEED, max(MIN_SPEED, self.speed * f))
            return True
        if name in ("left", "right") and shift:
            return True           # shift+left/right intentionally unused
        if name == " ":
            self.held = None
            return True
        if name == "q":
            return False
        if name in ("1","2"):
            gait={"1":"tripod","2":"ripple"}[name]
            self.gait_pub.publish(String(data=gait))
            self.requested=None if gait==self.gait else gait
            return True
        action = {"up": "fwd", "down": "back", "left": "left", "right": "right",
                  "a": "slide_l", "d": "slide_r"}.get(name)
        if action:
            if self.held and self.held[0] == action and now - self.held[2] < REPEAT_HOLD * 3:
                self.held[1] += 1
                self.held[2] = now
            else:
                self.held = [action, 1, now]
        return True

    def active_action(self, now):
        if not self.held:
            return None
        action, count, last = self.held
        hold = FIRST_HOLD if count < 2 else REPEAT_HOLD
        if now - last > hold:
            self.held = None
            return None
        return action

    # ---- output -----------------------------------------------------------
    def twist_for(self, action):
        vx = vy = wz = 0.0
        if action == "fwd":
            vx = self.speed
        elif action == "back":
            vx = -self.speed
        elif action == "slide_l":
            vy = self.speed
        elif action == "slide_r":
            vy = -self.speed
        elif action == "left":
            wz = TURN_RATE
        elif action == "right":
            wz = -TURN_RATE
        return vx, vy, wz

    def publish(self, vx, vy, wz):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
        self.pub.publish(m)

    def render(self, action, vx, vy, wz):
        def key(label, on):
            return Text(f" {label} ", style="bold black on bright_green" if on
                        else "white on grey23")

        pad = Table.grid(padding=(0, 1))
        pad.add_column(justify="center")
        pad.add_column(justify="center")
        pad.add_column(justify="center")
        pad.add_row(Text(""), key("▲", action == "fwd"), Text(""))
        pad.add_row(key("◀", action == "left"), key("▼", action == "back"),
                    key("▶", action == "right"))

        n = 28
        frac = (self.speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED)
        filled = round(frac * n)
        bar = Text("█" * filled, style="green") + Text("░" * (n - filled), style="grey37")

        labels = {None: ("STOPPED", "red"), "fwd": ("FORWARD", "green"),
                  "back": ("BACKWARD", "yellow"), "left": ("TURNING LEFT", "cyan"),
                  "right": ("TURNING RIGHT", "cyan"), "slide_l": ("STRAFE LEFT", "magenta"),
                  "slide_r": ("STRAFE RIGHT", "magenta")}
        txt, col = labels[action]
        status = Text(f"● {txt}", style=f"bold {col}")

        speed_row = Text.assemble(("speed  ", "bold"), bar,
                                  (f"  {self.speed:.3f} m/s", "bold white"))
        if self.requested:
            gait_row=Text.assemble(("gait   ","bold"),(f"{self.gait or '?'} → {self.requested} ...","bold yellow"))
        else:
            gait_row=Text.assemble(("gait   ","bold"),(self.gait or "waiting for gait node","bold cyan"))
        cmd = Text(f"cmd_vel   x={vx:+.3f}   y={vy:+.3f}   ωz={wz:+.2f}", style="dim")

        legend = Table(box=box.SIMPLE_HEAD, show_edge=False, pad_edge=False)
        legend.add_column("Key", style="bold cyan")
        legend.add_column("Action")
        legend.add_row("↑ / ↓", "forward / backward")
        legend.add_row("⇧↑ / ⇧↓", "speed +10% / −10%")
        legend.add_row("← / →", "turn left / right")
        legend.add_row("A / D", "strafe left / right")
        legend.add_row("1 / 2", "tripod / ripple gait")
        legend.add_row("Space / Q", "stop / quit")

        body = Group(Text(""), pad, Text(""), status, Text(""), speed_row, gait_row, cmd,
                     Text(""), legend)
        return Panel(body, title="[bold]🕷  HEXAPOD TELEOP[/]",
                     subtitle="[dim]/cmd_vel[/]", border_style="bright_blue",
                     box=box.ROUNDED, padding=(0, 2), expand=False)

    # ---- main loop --------------------------------------------------------
    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            period = 1.0 / PUB_HZ
            with Live(self.render(None, 0, 0, 0), refresh_per_second=PUB_HZ,
                      screen=True) as live:
                running = True
                while running and rclpy.ok():
                    if select.select([fd], [], [], period)[0]:
                        raw = sys.stdin.buffer.raw.read(64).decode(errors="ignore")
                        now = time.monotonic()
                        for name, shift in parse_keys(raw):
                            if not self.handle_key(name, shift, now):
                                running = False
                    action = self.active_action(time.monotonic())
                    vx, vy, wz = self.twist_for(action)
                    self.publish(vx, vy, wz)
                    rclpy.spin_once(self,timeout_sec=0.0)
                    live.update(self.render(action, vx, vy, wz))
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self.publish(0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = PrettyTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()