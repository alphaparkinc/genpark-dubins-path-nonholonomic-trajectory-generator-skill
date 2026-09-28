# Dubins Path Nonholonomic Trajectory Generator Skill

Analytical shortest path solver for wheeled mobile robots subject to nonholonomic curvature constraints.

```mermaid
flowchart LR
    Start["Start Pose (x0, y0, θ0)"] --> Words["Dubins Word Analysis (LSL, RSR, LSR, RSL)"]
    Goal["Goal Pose (x1, y1, θ1)"] --> Words
    Radius["Min Turn Radius r_min"] --> Words
    Words --> Best["Min Length Trajectory Selection"]
    Best --> Trajectory["Trajectory Output (Type, Length, Arc Waypoints)"]
```

## Features
- **100% Python Standard Library**: Analytical tangent circle mathematics.
- **Dubins Words**: LSL, RSR, LSR, RSL candidate path evaluations.
- **Fast Deterministic Solvers**: Instantaneous path generation for autonomous navigation.
