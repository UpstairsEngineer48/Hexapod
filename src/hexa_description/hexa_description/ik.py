import math
import os
import sys

#Angle offsets for the return at the bottom of ik():
#  offsets ON  -> calibrated angles, the bot stands at the proper height (slower). Used for ripple.
#  offsets OFF -> raw angles, the bot hugs the ground (faster). Used for tripod and the other gaits.
#  None (default) -> automatic: ON only when the script calling ik() has one of the OFFSET_SCRIPTS words in its file name.
#A node that switches gait at run time sets it explicitly with set_offsets(True/False).

#Standalone scripts that use the offsets (matched against the file name, case-insensitive). Edit this to change it:
#  ("ripple",)            -> ripple.py, ripple_closed.py
#  ("ripple","forward")   -> also forward.py, forward_closed.py
#  ("ripple","forward_closed") -> also only forward_closed.py
OFFSET_SCRIPTS=("ripple",)

#print one line the first time each script calls ik(), so you can see which mode it got
VERBOSE=True

_offsets=None
_reported=set()

def set_offsets(on):
    global _offsets
    _offsets=on
    if VERBOSE:
        print(f"[ik] offsets {'auto' if on is None else 'ON' if on else 'OFF'} (set by the running node)")

def ik(base, target, L1=0.07, L2=0.08, L3=0.15):
    """
    base:   [x1, y1, z1] - Joint 1 position (origin of the arm's frame)
    target: [x0, y0, z0] - desired end-effector position, world coordinates

    L1 is accepted as a parameter for completeness / documentation, but
    matching the derivation exactly, it does not appear in theta2/theta3.
    """
    x1, y1, z1 = base
    xt, yt, zt = target

    X0 = xt - x1
    Y0 = yt - y1
    Z0 = zt - z1

    def clamp(v, lo=-1.0, hi=1.0):
        return max(lo, min(hi, v))

    rho_xy = math.sqrt(X0**2 + Y0**2)
    dist_to_target = math.sqrt(X0**2 + Y0**2 + Z0**2)

    theta1 = math.atan2(Y0, X0)

    cos_term_2 = (-L3**2 + L2**2 + X0**2 + Y0**2 + Z0**2) / (2 * L2 * dist_to_target)
    theta2 = math.acos(clamp(cos_term_2)) + math.atan2(Z0, rho_xy)

    cos_term_3 = (X0**2 + Y0**2 + Z0**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta3 = -math.acos(clamp(cos_term_3))

    use_offsets=_offsets
    if use_offsets is None:
        name=os.path.basename(sys._getframe(1).f_code.co_filename).lower()
        use_offsets=any(k.lower() in name for k in OFFSET_SCRIPTS)
        if VERBOSE and name not in _reported:
            _reported.add(name)
            print(f"[ik] {name}: offsets {'ON' if use_offsets else 'OFF'} (auto)")
    if use_offsets:
        return (math.degrees(theta1),math.degrees(theta2-0.143),math.degrees(theta3+math.pi/2-0.5))
    return (math.degrees(theta1),math.degrees(theta2),math.degrees(theta3))