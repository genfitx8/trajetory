"""
3D Trajectory Simulation Module

Simulates projectile motion in 3D space with optional air resistance.
"""

import math


class Trajectory3D:
    """Simulates a 3D trajectory under gravity and optional drag."""

    GRAVITY = 9.81  # m/s^2
    # Decimal places used when rounding accumulated time to avoid floating-point drift
    TIME_PRECISION = 10

    def __init__(self, x0=0.0, y0=0.0, z0=0.0,
                 vx0=0.0, vy0=0.0, vz0=0.0,
                 mass=1.0, drag_coefficient=0.0, dt=0.01):
        """
        Initialize the 3D trajectory.

        Args:
            x0, y0, z0: Initial position (meters)
            vx0, vy0, vz0: Initial velocity (m/s)
            mass: Object mass (kg)
            drag_coefficient: Air drag coefficient (0 = no drag)
            dt: Time step (seconds)
        """
        self.x0 = x0
        self.y0 = y0
        self.z0 = z0
        self.vx0 = vx0
        self.vy0 = vy0
        self.vz0 = vz0
        self.mass = mass
        self.drag_coefficient = drag_coefficient
        self.dt = dt
        self._points = None

    def simulate(self, max_time=10.0):
        """
        Run the simulation until the object hits the ground (z <= 0)
        or max_time is reached. Results are cached for repeated queries.

        Returns:
            list of (t, x, y, z) tuples representing the trajectory
        """
        if self._points is not None:
            return self._points

        t = 0.0
        x, y, z = self.x0, self.y0, self.z0
        vx, vy, vz = self.vx0, self.vy0, self.vz0
        points = [(t, x, y, z)]

        while t < max_time and z >= 0:
            speed = math.sqrt(vx**2 + vy**2 + vz**2)
            if speed > 0 and self.drag_coefficient > 0:
                drag = self.drag_coefficient * speed / self.mass
            else:
                drag = 0.0

            ax = -drag * vx
            ay = -drag * vy
            az = -self.GRAVITY - drag * vz

            vx += ax * self.dt
            vy += ay * self.dt
            vz += az * self.dt

            x += vx * self.dt
            y += vy * self.dt
            z += vz * self.dt

            t = round(t + self.dt, self.TIME_PRECISION)
            points.append((t, x, y, z))

        self._points = points
        return self._points

    def range_xy(self):
        """Return the horizontal range (distance in XY plane) at landing."""
        points = self.simulate()
        if len(points) < 2:
            return 0.0
        _, x, y, _ = points[-1]
        return math.sqrt((x - self.x0)**2 + (y - self.y0)**2)

    def max_height(self):
        """Return the maximum Z height reached during the trajectory."""
        points = self.simulate()
        return max(z for _, _, _, z in points)

    def flight_time(self):
        """Return the total flight time."""
        points = self.simulate()
        return points[-1][0]

    @classmethod
    def from_angle(cls, speed, elevation_deg, azimuth_deg=0.0,
                   x0=0.0, y0=0.0, z0=0.0, **kwargs):
        """
        Create a Trajectory3D from launch speed and angles.

        Args:
            speed: Launch speed (m/s)
            elevation_deg: Elevation angle above horizontal (degrees)
            azimuth_deg: Horizontal direction angle (degrees, 0 = +X axis)
        """
        elev = math.radians(elevation_deg)
        azim = math.radians(azimuth_deg)
        vz0 = speed * math.sin(elev)
        vh = speed * math.cos(elev)
        vx0 = vh * math.cos(azim)
        vy0 = vh * math.sin(azim)
        return cls(x0=x0, y0=y0, z0=z0, vx0=vx0, vy0=vy0, vz0=vz0, **kwargs)
