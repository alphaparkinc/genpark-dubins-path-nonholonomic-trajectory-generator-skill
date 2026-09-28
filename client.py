"""Dubins Path Trajectory Generator for Nonholonomic Vehicles.
100% Python Standard Library.
"""

import math

class DubinsPathGenerator:
    """Computes shortest path between two poses (x, y, theta) for fixed turning radius."""
    @staticmethod
    def mod2pi(theta):
        return theta % (2.0 * math.pi)

    @staticmethod
    def compute_lsl(alpha, beta, d):
        p_sq = 2.0 + d**2 - (2.0 * math.cos(alpha - beta)) + (2.0 * d * (math.sin(alpha) - math.sin(beta)))
        if p_sq < 0:
            return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(math.cos(beta) - math.cos(alpha), d + math.sin(alpha) - math.sin(beta))
        t = DubinsPathGenerator.mod2pi(-alpha + tmp)
        q = DubinsPathGenerator.mod2pi(beta - tmp)
        return (t, p, q, "LSL", t + p + q)

    @staticmethod
    def compute_rsr(alpha, beta, d):
        p_sq = 2.0 + d**2 - (2.0 * math.cos(alpha - beta)) - (2.0 * d * (math.sin(alpha) - math.sin(beta)))
        if p_sq < 0:
            return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta))
        t = DubinsPathGenerator.mod2pi(alpha - tmp)
        q = DubinsPathGenerator.mod2pi(-beta + tmp)
        return (t, p, q, "RSR", t + p + q)

    @staticmethod
    def compute_lsr(alpha, beta, d):
        p_sq = -2.0 + d**2 + (2.0 * math.cos(alpha - beta)) + (2.0 * d * (math.sin(alpha) + math.sin(beta)))
        if p_sq < 0:
            return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(-math.cos(alpha) - math.cos(beta), d + math.sin(alpha) + math.sin(beta)) - math.atan2(-2.0, p)
        t = DubinsPathGenerator.mod2pi(-alpha + tmp)
        q = DubinsPathGenerator.mod2pi(-DubinsPathGenerator.mod2pi(beta) + tmp)
        return (t, p, q, "LSR", t + p + q)

    @staticmethod
    def compute_rsl(alpha, beta, d):
        p_sq = -2.0 + d**2 + (2.0 * math.cos(alpha - beta)) - (2.0 * d * (math.sin(alpha) + math.sin(beta)))
        if p_sq < 0:
            return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(math.cos(alpha) + math.cos(beta), d - math.sin(alpha) - math.sin(beta)) - math.atan2(2.0, p)
        t = DubinsPathGenerator.mod2pi(alpha - tmp)
        q = DubinsPathGenerator.mod2pi(beta - tmp)
        return (t, p, q, "RSL", t + p + q)

    @staticmethod
    def shortest_path(start, goal, r_min=1.0):
        """start, goal: (x, y, theta_rad). Returns dictionary with trajectory parameters."""
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        d = math.hypot(dx, dy) / r_min
        if d < 1e-6:
            return {"type": "NONE", "total_length": 0.0, "r_min": r_min}
        
        phi = math.atan2(dy, dx)
        alpha = DubinsPathGenerator.mod2pi(start[2] - phi)
        beta = DubinsPathGenerator.mod2pi(goal[2] - phi)
        
        candidates = [
            DubinsPathGenerator.compute_lsl(alpha, beta, d),
            DubinsPathGenerator.compute_rsr(alpha, beta, d),
            DubinsPathGenerator.compute_lsr(alpha, beta, d),
            DubinsPathGenerator.compute_rsl(alpha, beta, d)
        ]
        valid = [c for c in candidates if c is not None]
        if not valid:
            return None
        best = min(valid, key=lambda c: c[4])
        return {
            "type": best[3],
            "total_length": round(best[4] * r_min, 4),
            "segment_lengths": (round(best[0] * r_min, 4), round(best[1] * r_min, 4), round(best[2] * r_min, 4)),
            "r_min": r_min
        }
