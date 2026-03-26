"""Tests for the 3D trajectory simulation module."""

import math
import unittest

from trajectory import Trajectory3D


class TestTrajectory3DBasic(unittest.TestCase):
    """Basic physics tests for Trajectory3D."""

    def test_stationary_object_falls(self):
        """An object with zero horizontal velocity falls straight down."""
        t = Trajectory3D(x0=0, y0=0, z0=100, vx0=0, vy0=0, vz0=0, dt=0.01)
        points = t.simulate()
        _, x_final, y_final, _ = points[-1]
        self.assertAlmostEqual(x_final, 0.0, places=5)
        self.assertAlmostEqual(y_final, 0.0, places=5)

    def test_range_increases_with_speed(self):
        """Faster launch produces greater horizontal range."""
        t1 = Trajectory3D.from_angle(speed=10, elevation_deg=45)
        t2 = Trajectory3D.from_angle(speed=20, elevation_deg=45)
        self.assertGreater(t2.range_xy(), t1.range_xy())

    def test_max_height_at_45_degrees(self):
        """Max height for 45-degree launch is positive."""
        t = Trajectory3D.from_angle(speed=20, elevation_deg=45, z0=0)
        self.assertGreater(t.max_height(), 0)

    def test_flight_time_positive(self):
        """Flight time is always positive."""
        t = Trajectory3D.from_angle(speed=15, elevation_deg=30, z0=5)
        self.assertGreater(t.flight_time(), 0)

    def test_no_drag_vs_drag(self):
        """Drag reduces the range compared to no drag."""
        t_no_drag = Trajectory3D.from_angle(speed=30, elevation_deg=45,
                                            drag_coefficient=0.0)
        t_drag = Trajectory3D.from_angle(speed=30, elevation_deg=45,
                                         drag_coefficient=0.1)
        self.assertGreater(t_no_drag.range_xy(), t_drag.range_xy())

    def test_higher_elevation_initial_z(self):
        """Object launched from higher z takes longer to land."""
        t_low = Trajectory3D.from_angle(speed=20, elevation_deg=30, z0=0)
        t_high = Trajectory3D.from_angle(speed=20, elevation_deg=30, z0=50)
        self.assertGreater(t_high.flight_time(), t_low.flight_time())

    def test_from_angle_velocity_components(self):
        """from_angle correctly decomposes velocity into components."""
        speed = 10.0
        t = Trajectory3D.from_angle(speed=speed, elevation_deg=90, azimuth_deg=0)
        # At 90 degrees elevation, all velocity should be in Z
        self.assertAlmostEqual(t.vz0, speed, places=5)
        self.assertAlmostEqual(t.vx0, 0.0, places=5)
        self.assertAlmostEqual(t.vy0, 0.0, places=5)

    def test_azimuth_splits_horizontal_velocity(self):
        """45-degree azimuth splits horizontal velocity equally in X and Y."""
        t = Trajectory3D.from_angle(speed=10, elevation_deg=0, azimuth_deg=45)
        self.assertAlmostEqual(t.vx0, t.vy0, places=5)

    def test_landing_z_near_zero(self):
        """The trajectory ends near z=0 when starting from z=0."""
        t = Trajectory3D.from_angle(speed=20, elevation_deg=30, z0=0, dt=0.001)
        points = t.simulate()
        _, _, _, z_final = points[-1]
        self.assertLessEqual(z_final, 0.1)

    def test_simulate_returns_list(self):
        """simulate() returns a non-empty list of tuples."""
        t = Trajectory3D.from_angle(speed=10, elevation_deg=45)
        points = t.simulate()
        self.assertIsInstance(points, list)
        self.assertGreater(len(points), 1)
        self.assertEqual(len(points[0]), 4)  # (t, x, y, z)


if __name__ == "__main__":
    unittest.main()
