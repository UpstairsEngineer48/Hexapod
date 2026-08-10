import math

def ik(A, C, L_coxa, L1, L2):
    """
    A: [x1, y1, z1] - coxa joint position (base of the leg)
    C: [x3, y3, z3] - target foot position
    L_coxa: coxa link length
    L1: femur link length
    L2: tibia link length

    Returns: (gamma, alpha, beta) in degrees
        gamma - coxa (yaw) angle
        alpha - femur angle
        beta  - tibia (knee) angle
    """
    x1, y1, z1 = A
    x3, y3, z3 = C

    dx = x3 - x1
    dy = y3 - y1
    dz = z3 - z1

    # 1) Coxa yaw angle - depends only on horizontal direction
    gamma = math.atan2(dy, dx)

    # 2) Horizontal distance to target, minus the coxa link already "used up"
    r = math.sqrt(dx**2 + dy**2)
    r_prime = r - L_coxa

    # 3) Effective distance from femur joint to target (2D problem now)
    D = math.sqrt(r_prime**2 + dz**2)

    # Clamp helper to keep acos domain safe against float error / unreachable targets
    def clamp(v, lo=-1.0, hi=1.0):
        return max(lo, min(hi, v))

    # 4) Law of cosines for knee angle (beta)
    cos_beta = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    beta = math.acos(clamp(cos_beta))

    # 5) Femur angle (alpha) = triangle-internal angle minus elevation angle
    cos_alpha_tri = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    alpha_tri = math.acos(clamp(cos_alpha_tri))
    elevation = math.atan2(dz, r_prime)
    alpha = alpha_tri - elevation

    return (math.degrees(alpha), math.degrees(beta), math.degrees(gamma))