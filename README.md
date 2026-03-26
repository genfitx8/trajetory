# trajetory

A Python library for simulating **3D projectile trajectories** with optional aerodynamic drag.

## Features

- 3D vector math (`Vector3`) — add, subtract, scale, dot/cross product, normalize
- Physics-based trajectory simulation using numerical integration (Euler method)
- Optional quadratic aerodynamic drag force
- Launch from any position, speed, elevation angle and azimuth heading
- Query max height, range, time of flight and landing position

## Files

| File | Description |
|------|-------------|
| `trajectory.py` | Core library — `Vector3`, `TrajectoryParams`, `Trajectory`, `from_angle` |
| `example.py` | Demonstrates several launch scenarios and prints statistics |
| `test_trajectory.py` | Unit test suite (Python `unittest`) |

## Quick start

```python
from trajectory import from_angle

# 45° launch at 30 m/s, no drag
traj = from_angle(speed=30, elevation_deg=45)
traj.simulate()

print(f"Range     : {traj.range_distance():.1f} m")
print(f"Max height: {traj.max_height():.1f} m")
print(f"Flight    : {traj.time_of_flight():.2f} s")
```

### 3-D launch with azimuth

```python
traj = from_angle(speed=20, elevation_deg=30, azimuth_deg=45)
traj.simulate()
print(traj.landing_position())   # Vector3(x, y, z)
```

### With air drag

```python
traj = from_angle(
    speed=30,
    elevation_deg=45,
    drag_coefficient=0.47,   # sphere
    mass=0.145,              # kg  (baseball)
)
traj.simulate()
```

### Custom parameters

```python
from trajectory import Trajectory, TrajectoryParams, Vector3

params = TrajectoryParams(
    initial_position=Vector3(0, 10, 0),  # 10 m above ground
    initial_velocity=Vector3(15, 5, 8),
    gravity=9.81,
    drag_coefficient=0.1,
    mass=0.5,
    dt=0.005,
)
traj = Trajectory(params)
traj.simulate()
```

## Running the tests

```bash
python -m unittest test_trajectory -v
```

## Physics

The equation of motion integrated at each time step `dt`:

```
a = (F_gravity + F_drag) / m
v += a * dt
x += v * dt
```

where:

- `F_gravity = (0, -m*g, 0)`
- `F_drag = -0.5 * ρ * Cd * A * |v|² * v̂`  (quadratic drag, opposes velocity)

