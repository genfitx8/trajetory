"""
Unit tests for the 3D trajectory simulation module.
"""

import math
import unittest

from trajectory import (
    Trajectory,
    TrajectoryParams,
    TrajectoryPoint,
    Vector3,
    from_angle,
)


class TestVector3(unittest.TestCase):
    def test_addition(self):
        a = Vector3(1, 2, 3)
        b = Vector3(4, 5, 6)
        result = a + b
        self.assertAlmostEqual(result.x, 5)
        self.assertAlmostEqual(result.y, 7)
        self.assertAlmostEqual(result.z, 9)

    def test_subtraction(self):
        a = Vector3(4, 5, 6)
        b = Vector3(1, 2, 3)
        result = a - b
        self.assertAlmostEqual(result.x, 3)
        self.assertAlmostEqual(result.y, 3)
        self.assertAlmostEqual(result.z, 3)

    def test_scalar_multiplication(self):
        v = Vector3(1, 2, 3)
        result = v * 2
        self.assertAlmostEqual(result.x, 2)
        self.assertAlmostEqual(result.y, 4)
        self.assertAlmostEqual(result.z, 6)

    def test_rmul(self):
        v = Vector3(1, 2, 3)
        result = 3 * v
        self.assertAlmostEqual(result.x, 3)
        self.assertAlmostEqual(result.y, 6)
        self.assertAlmostEqual(result.z, 9)

    def test_division(self):
        v = Vector3(2, 4, 6)
        result = v / 2
        self.assertAlmostEqual(result.x, 1)
        self.assertAlmostEqual(result.y, 2)
        self.assertAlmostEqual(result.z, 3)

    def test_magnitude(self):
        v = Vector3(3, 4, 0)
        self.assertAlmostEqual(v.magnitude(), 5.0)

    def test_normalize(self):
        v = Vector3(0, 5, 0)
        n = v.normalize()
        self.assertAlmostEqual(n.magnitude(), 1.0)
        self.assertAlmostEqual(n.y, 1.0)

    def test_normalize_zero_vector(self):
        v = Vector3(0, 0, 0)
        n = v.normalize()
        self.assertAlmostEqual(n.magnitude(), 0.0)

    def test_dot_product(self):
        a = Vector3(1, 0, 0)
        b = Vector3(0, 1, 0)
        self.assertAlmostEqual(a.dot(b), 0.0)
        self.assertAlmostEqual(a.dot(a), 1.0)

    def test_cross_product(self):
        a = Vector3(1, 0, 0)
        b = Vector3(0, 1, 0)
        c = a.cross(b)
        self.assertAlmostEqual(c.x, 0)
        self.assertAlmostEqual(c.y, 0)
        self.assertAlmostEqual(c.z, 1)


class TestTrajectory(unittest.TestCase):
    def _basic_params(self, **kwargs) -> TrajectoryParams:
        defaults = dict(
            initial_velocity=Vector3(10, 10, 0),
            gravity=9.81,
            drag_coefficient=0.0,
            dt=0.01,
            max_time=20.0,
            ground_y=0.0,
        )
        defaults.update(kwargs)
        return TrajectoryParams(**defaults)

    def test_simulate_returns_points(self):
        traj = Trajectory(self._basic_params())
        pts = traj.simulate()
        self.assertGreater(len(pts), 0)

    def test_first_point_is_initial_condition(self):
        params = self._basic_params()
        traj = Trajectory(params)
        traj.simulate()
        first = traj.points[0]
        self.assertAlmostEqual(first.time, 0.0)
        self.assertAlmostEqual(first.position.x, params.initial_position.x)
        self.assertAlmostEqual(first.position.y, params.initial_position.y)
        self.assertAlmostEqual(first.velocity.x, params.initial_velocity.x)
        self.assertAlmostEqual(first.velocity.y, params.initial_velocity.y)

    def test_max_height_greater_than_launch(self):
        traj = Trajectory(self._basic_params())
        traj.simulate()
        self.assertGreater(traj.max_height(), 0.0)

    def test_landing_y_near_ground(self):
        traj = Trajectory(self._basic_params())
        traj.simulate()
        # The simulation stops at or below ground_y = 0
        self.assertLessEqual(traj.landing_position().y, 0.1)

    def test_range_is_positive(self):
        traj = Trajectory(self._basic_params())
        traj.simulate()
        self.assertGreater(traj.range_distance(), 0.0)

    def test_no_drag_vs_drag(self):
        # Drag should reduce the range
        no_drag = from_angle(speed=30, elevation_deg=45, drag_coefficient=0.0)
        with_drag = from_angle(speed=30, elevation_deg=45, drag_coefficient=0.47, mass=0.145)
        no_drag.simulate()
        with_drag.simulate()
        self.assertGreater(no_drag.range_distance(), with_drag.range_distance())

    def test_90_degree_launch_max_height(self):
        """Straight-up launch: range should be ~0, height should be significant."""
        traj = from_angle(speed=20, elevation_deg=90, drag_coefficient=0.0)
        traj.simulate()
        self.assertLess(traj.range_distance(), 1.0)  # negligible horizontal drift
        self.assertGreater(traj.max_height(), 10.0)

    def test_3d_launch_has_z_component(self):
        traj = from_angle(speed=20, elevation_deg=30, azimuth_deg=90, drag_coefficient=0.0)
        traj.simulate()
        # With azimuth = 90°, all horizontal motion is in z-direction
        self.assertAlmostEqual(traj.landing_position().x, 0.0, delta=0.5)
        self.assertGreater(abs(traj.landing_position().z), 1.0)

    def test_query_before_simulate_raises(self):
        traj = Trajectory(self._basic_params())
        with self.assertRaises(RuntimeError):
            traj.max_height()
        with self.assertRaises(RuntimeError):
            traj.range_distance()
        with self.assertRaises(RuntimeError):
            traj.time_of_flight()
        with self.assertRaises(RuntimeError):
            traj.landing_position()

    def test_time_of_flight_positive(self):
        traj = from_angle(speed=20, elevation_deg=45)
        traj.simulate()
        self.assertGreater(traj.time_of_flight(), 0.0)

    def test_trajectory_point_speed(self):
        pt = TrajectoryPoint(
            time=0.0,
            position=Vector3(0, 0, 0),
            velocity=Vector3(3, 4, 0),
        )
        self.assertAlmostEqual(pt.speed, 5.0)

    def test_elevated_start(self):
        """Projectile starting above the ground should travel farther."""
        low_start = from_angle(speed=20, elevation_deg=30)
        low_start.simulate()

        params = TrajectoryParams(
            initial_position=Vector3(0, 10, 0),
            initial_velocity=Vector3(
                20 * math.cos(math.radians(30)),
                20 * math.sin(math.radians(30)),
                0,
            ),
            gravity=9.81,
            drag_coefficient=0.0,
        )
        high_start = Trajectory(params)
        high_start.simulate()

        self.assertGreater(high_start.range_distance(), low_start.range_distance())


if __name__ == "__main__":
    unittest.main()
