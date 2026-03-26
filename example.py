"""
Example usage of the 3D trajectory simulation.

Demonstrates:
  1. Basic projectile launched at an angle
  2. Comparing trajectories with and without air drag
  3. Printing key statistics
"""

from trajectory import Trajectory, TrajectoryParams, Vector3, from_angle


def print_stats(label: str, traj: Trajectory) -> None:
    pts = traj.simulate()
    print(f"\n=== {label} ===")
    print(f"  Time of flight : {traj.time_of_flight():.3f} s")
    print(f"  Max height     : {traj.max_height():.3f} m")
    print(f"  Range          : {traj.range_distance():.3f} m")
    print(f"  Landing pos    : {traj.landing_position()}")
    print(f"  Points logged  : {len(pts)}")


def main() -> None:
    # ------------------------------------------------------------------ #
    # 1. Classic 45° launch — maximum range (no drag)                     #
    # ------------------------------------------------------------------ #
    traj_no_drag = from_angle(speed=30, elevation_deg=45, drag_coefficient=0.0)
    print_stats("45° launch, no drag (v=30 m/s)", traj_no_drag)

    # ------------------------------------------------------------------ #
    # 2. Same launch with air drag                                        #
    # ------------------------------------------------------------------ #
    traj_drag = from_angle(
        speed=30,
        elevation_deg=45,
        drag_coefficient=0.47,   # sphere
        mass=0.145,              # baseball ~145 g
    )
    print_stats("45° launch, with drag (baseball)", traj_drag)

    # ------------------------------------------------------------------ #
    # 3. 3-D trajectory: launch at azimuth 30° from north                #
    # ------------------------------------------------------------------ #
    traj_3d = from_angle(
        speed=20,
        elevation_deg=30,
        azimuth_deg=30,
        drag_coefficient=0.0,
    )
    print_stats("3-D launch (elev=30°, azim=30°)", traj_3d)

    # ------------------------------------------------------------------ #
    # 4. Custom parameters via TrajectoryParams                          #
    # ------------------------------------------------------------------ #
    params = TrajectoryParams(
        initial_position=Vector3(0, 10, 0),   # starts 10 m above ground
        initial_velocity=Vector3(15, 5, 8),
        gravity=9.81,
        drag_coefficient=0.1,
        mass=0.5,
        dt=0.005,
    )
    traj_custom = Trajectory(params)
    print_stats("Custom params (elevated launch)", traj_custom)


if __name__ == "__main__":
    main()
