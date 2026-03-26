"""
3D Trajectory Simulation Module

Calculates the trajectory of a projectile in 3D space with optional
drag force (air resistance) and gravity.
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class Vector3:
    """A 3D vector with x, y, z components."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vector3":
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> "Vector3":
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> "Vector3":
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def magnitude(self) -> float:
        """Return the Euclidean length of the vector."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self) -> "Vector3":
        """Return a unit vector in the same direction."""
        mag = self.magnitude()
        if mag == 0:
            return Vector3(0.0, 0.0, 0.0)
        return self / mag

    def dot(self, other: "Vector3") -> float:
        """Return the dot product with another vector."""
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vector3") -> "Vector3":
        """Return the cross product with another vector."""
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def __repr__(self) -> str:
        return f"Vector3({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"


@dataclass
class TrajectoryPoint:
    """A single point along a 3D trajectory."""

    time: float
    position: Vector3
    velocity: Vector3

    @property
    def speed(self) -> float:
        """Scalar speed at this point."""
        return self.velocity.magnitude()

    def __repr__(self) -> str:
        return (
            f"TrajectoryPoint(t={self.time:.3f}s, "
            f"pos={self.position}, "
            f"speed={self.speed:.3f} m/s)"
        )


@dataclass
class TrajectoryParams:
    """Parameters that define a 3D trajectory simulation."""

    # Initial conditions
    initial_position: Vector3 = field(default_factory=lambda: Vector3(0, 0, 0))
    initial_velocity: Vector3 = field(default_factory=lambda: Vector3(10, 10, 0))

    # Physical constants
    gravity: float = 9.81          # m/s², downward (applied to y-axis)
    mass: float = 1.0              # kg
    drag_coefficient: float = 0.0  # dimensionless (0 = no drag)
    cross_section_area: float = 0.01  # m² (used for drag calculation)
    air_density: float = 1.225     # kg/m³ (sea level, 15 °C)

    # Simulation settings
    dt: float = 0.01              # time step in seconds
    max_time: float = 10.0        # maximum simulation duration in seconds
    ground_y: float = 0.0         # y-coordinate of the ground plane


class Trajectory:
    """
    Simulates a 3D projectile trajectory using numerical integration
    (Euler method).

    Gravity acts in the negative y-direction. Optional quadratic drag
    force opposes the velocity direction.
    """

    def __init__(self, params: TrajectoryParams):
        self.params = params
        self._points: List[TrajectoryPoint] = []

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def simulate(self) -> List[TrajectoryPoint]:
        """
        Run the simulation and return the list of trajectory points.

        The simulation stops when the projectile hits the ground
        (y <= ground_y after the first step) or max_time is reached.
        """
        p = self.params
        pos = Vector3(p.initial_position.x, p.initial_position.y, p.initial_position.z)
        vel = Vector3(p.initial_velocity.x, p.initial_velocity.y, p.initial_velocity.z)
        t = 0.0
        self._points = [TrajectoryPoint(t, Vector3(pos.x, pos.y, pos.z), Vector3(vel.x, vel.y, vel.z))]

        while t < p.max_time:
            acc = self._acceleration(vel)
            pos = pos + vel * p.dt
            vel = vel + acc * p.dt
            t += p.dt

            self._points.append(
                TrajectoryPoint(t, Vector3(pos.x, pos.y, pos.z), Vector3(vel.x, vel.y, vel.z))
            )

            if pos.y <= p.ground_y and t > p.dt:
                break

        return self._points

    @property
    def points(self) -> List[TrajectoryPoint]:
        """Return cached trajectory points (run simulate() first)."""
        return self._points

    def max_height(self) -> float:
        """Return the maximum y-coordinate reached during the trajectory."""
        if not self._points:
            raise RuntimeError("Call simulate() before querying results.")
        return max(pt.position.y for pt in self._points)

    def range_distance(self) -> float:
        """
        Return the horizontal distance (xz-plane) from launch to landing.
        """
        if not self._points:
            raise RuntimeError("Call simulate() before querying results.")
        start = self._points[0].position
        end = self._points[-1].position
        dx = end.x - start.x
        dz = end.z - start.z
        return math.sqrt(dx**2 + dz**2)

    def time_of_flight(self) -> float:
        """Return the total simulated time."""
        if not self._points:
            raise RuntimeError("Call simulate() before querying results.")
        return self._points[-1].time

    def landing_position(self) -> Vector3:
        """Return the position of the last simulated point."""
        if not self._points:
            raise RuntimeError("Call simulate() before querying results.")
        return self._points[-1].position

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _drag_force(self, velocity: Vector3) -> Vector3:
        """
        Compute the quadratic drag force vector.

        F_drag = -0.5 * rho * Cd * A * |v|^2 * v_hat
        """
        p = self.params
        speed = velocity.magnitude()
        if speed == 0 or p.drag_coefficient == 0:
            return Vector3(0.0, 0.0, 0.0)
        force_magnitude = (
            0.5 * p.air_density * p.drag_coefficient * p.cross_section_area * speed**2
        )
        direction = velocity.normalize() * (-1)
        return direction * force_magnitude

    def _acceleration(self, velocity: Vector3) -> Vector3:
        """Return the net acceleration vector at a given velocity."""
        gravity_force = Vector3(0.0, -self.params.mass * self.params.gravity, 0.0)
        drag = self._drag_force(velocity)
        net_force = gravity_force + drag
        return net_force / self.params.mass


# ---------------------------------------------------------------------------
# Convenience factory functions
# ---------------------------------------------------------------------------

def from_angle(
    speed: float,
    elevation_deg: float,
    azimuth_deg: float = 0.0,
    *,
    drag_coefficient: float = 0.0,
    mass: float = 1.0,
    dt: float = 0.01,
) -> Trajectory:
    """
    Create a :class:`Trajectory` from launch speed and angles.

    Parameters
    ----------
    speed : float
        Initial speed in m/s.
    elevation_deg : float
        Elevation angle above the horizontal plane, in degrees.
        0° is horizontal, 90° is straight up.
    azimuth_deg : float
        Compass heading in degrees. 0° points along the +x axis,
        90° points along the +z axis.
    drag_coefficient : float
        Aerodynamic drag coefficient (0 = no drag).
    mass : float
        Projectile mass in kg.
    dt : float
        Simulation time step in seconds.
    """
    elev = math.radians(elevation_deg)
    azim = math.radians(azimuth_deg)

    vx = speed * math.cos(elev) * math.cos(azim)
    vy = speed * math.sin(elev)
    vz = speed * math.cos(elev) * math.sin(azim)

    params = TrajectoryParams(
        initial_velocity=Vector3(vx, vy, vz),
        drag_coefficient=drag_coefficient,
        mass=mass,
        dt=dt,
    )
    return Trajectory(params)
