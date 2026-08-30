import math

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

    return (math.degrees(theta1), math.degrees(theta2), math.degrees(theta3))