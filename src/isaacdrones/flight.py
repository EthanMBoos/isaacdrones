"""A position controller for the rigid-body drone, and the flight it follows.

The drone is a rigid body that takes a thrust along its own z axis and a
torque, with no rotors. This controller turns a target position into that
thrust and torque. It is here so that the simulated sensors have a flight to
record, and it is not a model of a flight controller.

The body frame is x forward, y left, z up. Quaternions are wxyz and rotate
body vectors into the world. Pure Python, so it runs without Isaac Sim.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

GRAVITY_MPS2 = 9.81

Vec = tuple[float, float, float]


@dataclass(frozen=True)
class Drone:
    """A box of uniform density. Isaac Sim derives the same inertia from the
    mass and the box, so the controller and the simulator agree.
    """

    mass_kg: float = 1.0
    size_m: Vec = (0.3, 0.3, 0.08)

    @property
    def inertia_kgm2(self) -> Vec:
        x, y, z = self.size_m
        k = self.mass_kg / 12.0
        return (k * (y * y + z * z), k * (x * x + z * z), k * (x * x + y * y))


@dataclass(frozen=True)
class State:
    position_m: Vec
    quat_wxyz: tuple[float, float, float, float]
    velocity_mps: Vec  # world frame
    angular_velocity_radps: Vec  # world frame


def rotation(quat_wxyz) -> list[Vec]:
    """Rotation matrix whose columns are the body axes in the world."""
    w, x, y, z = quat_wxyz
    return [
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    ]


def _cross(a: Vec, b: Vec) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a: Vec) -> Vec:
    norm = math.sqrt(sum(c * c for c in a))
    return (a[0] / norm, a[1] / norm, a[2] / norm)


def wrench(
    drone: Drone,
    state: State,
    target_m: Vec,
    target_velocity_mps: Vec = (0.0, 0.0, 0.0),
    *,
    position_gain: float = 4.0,
    velocity_gain: float = 3.0,
    attitude_gain: float = 100.0,
    rate_gain: float = 20.0,
) -> tuple[float, Vec]:
    """Thrust along body z in newtons, and torque in the body frame in N m.

    The position error sets an acceleration. The drone can only push along
    its z axis, so the thrust is the part of that acceleration along z, and
    the torque turns z toward the rest. The nose is kept pointing along
    world x.
    """
    accel = tuple(
        position_gain * (target_m[i] - state.position_m[i])
        + velocity_gain * (target_velocity_mps[i] - state.velocity_mps[i])
        for i in range(3)
    )
    accel = (accel[0], accel[1], accel[2] + GRAVITY_MPS2)

    r = rotation(state.quat_wxyz)
    body_x, body_y, body_z = ([r[row][col] for row in range(3)] for col in range(3))
    thrust_n = max(0.0, drone.mass_kg * sum(accel[i] * body_z[i] for i in range(3)))

    want_z = _unit(accel)
    want_y = _unit(_cross(want_z, (1.0, 0.0, 0.0)))
    want_x = _cross(want_y, want_z)
    # The rotation error, as a vector in the body frame: half of what is left
    # of Rd^T R - R^T Rd.
    wanted = (want_x, want_y, want_z)
    body = (body_x, body_y, body_z)

    def m(i: int, j: int) -> float:  # (Rd^T R)[i][j]
        return sum(wanted[i][k] * body[j][k] for k in range(3))

    error = (0.5 * (m(2, 1) - m(1, 2)), 0.5 * (m(0, 2) - m(2, 0)), 0.5 * (m(1, 0) - m(0, 1)))
    rate = tuple(
        sum(body[i][k] * state.angular_velocity_radps[k] for k in range(3)) for i in range(3)
    )
    inertia = drone.inertia_kgm2
    torque_nm = tuple(
        inertia[i] * (-attitude_gain * error[i] - rate_gain * rate[i]) for i in range(3)
    )
    return thrust_n, torque_nm


# The flight: hover, then fly at the cube and stop just short of it.
START_M: Vec = (-6.0, 0.0, 1.5)
HOVER_S = 1.0
SPEED_MPS = 1.0
STOP_X_M = -1.0


def target(time_s: float) -> tuple[Vec, Vec]:
    """Target position and velocity `time_s` after the flight starts. Before
    it starts, at negative times, the target is the start.
    """
    x = START_M[0] + SPEED_MPS * max(0.0, time_s - HOVER_S)
    if x >= STOP_X_M:
        return (STOP_X_M, START_M[1], START_M[2]), (0.0, 0.0, 0.0)
    moving = SPEED_MPS if time_s > HOVER_S else 0.0
    return (x, START_M[1], START_M[2]), (moving, 0.0, 0.0)
