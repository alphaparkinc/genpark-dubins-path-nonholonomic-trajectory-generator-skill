"""Example evaluating Dubins vehicle path."""
import math
from client import DubinsPathGenerator

def main():
    start = (0.0, 0.0, 0.0)
    goal = (10.0, 10.0, math.pi / 2)
    r_min = 2.5
    print(f"Computing Dubins path from {start} to {goal} (r_min={r_min}):")
    path = DubinsPathGenerator.shortest_path(start, goal, r_min=r_min)
    print(f"  Optimal Word: {path['type']}")
    print(f"  Total Length: {path['total_length']}")
    print(f"  Segments: {path['segment_lengths']}")

if __name__ == "__main__":
    main()
