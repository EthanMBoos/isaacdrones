import ast
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

import isaacdrones
from isaacdrones.app import parse_args

SRC = Path(isaacdrones.__file__).parent
SIM_ONLY = {"isaacsim", "omni", "pxr"}


def test_importable_without_isaac_sim():
    import isaacdrones.app
    import isaacdrones.sim  # noqa: F401

    assert isaacdrones.__version__
    assert not SIM_ONLY & {name.split(".")[0] for name in sys.modules}


def test_parse_args():
    args = parse_args(["--headless", "--steps", "10"])
    assert args.headless and args.steps == 10
    assert args.stream is None and args.scene is None
    assert parse_args(["--scene", "falling-cube"]).scene == "falling-cube"
    assert parse_args(["--stream"]).stream == "127.0.0.1"
    assert parse_args(["--stream", "10.0.0.5"]).stream == "10.0.0.5"
    assert parse_args(["--ros"]).ros and not args.ros
    assert args.resolution_hw == (480, 640) and args.image_hz == 60
    assert parse_args(["--resolution", "1280x720"]).resolution_hw == (720, 1280)


def test_isaac_sim_only_imported_in_sim_package():
    offenders = []
    for path in SRC.rglob("*.py"):
        if SRC / "sim" in path.parents:
            continue
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            else:
                continue
            if SIM_ONLY & {name.split(".")[0] for name in names}:
                offenders.append(f"{path.relative_to(SRC)}:{node.lineno}")
    assert not offenders, offenders


@pytest.mark.sim
def test_boot_headless():
    pytest.importorskip("isaacsim")
    # Closing Isaac Sim exits the process, so it runs in a subprocess.
    steps = 10
    result = subprocess.run(
        [sys.executable, "-m", "isaacdrones.app", "--headless", "--steps", str(steps)],
        capture_output=True,
        text=True,
        timeout=900,
    )
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    match = re.search(
        r"isaacdrones: frames=(\d+) elapsed=(\S+) sim_time=(\S+) dt=(\S+) "
        r"physics_steps=(\d+) engine=(\S+)",
        result.stdout,
    )
    assert match, result.stdout[-3000:]
    print(match[0])
    frames, elapsed, dt, engine = int(match[1]), float(match[2]), float(match[4]), match[6]
    assert frames == steps
    assert engine == "physx"
    # The default run stays light: the ROS 2 bridge loads only with --ros.
    assert "isaacsim.ros2" not in result.stdout
    # One physics step per frame.
    assert abs(elapsed - steps * dt) <= dt * 0.01


@pytest.mark.sim
def test_camera_renders(tmp_path):
    pytest.importorskip("isaacsim")
    png = tmp_path / "frame.png"
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "isaacdrones.app",
            "--headless",
            "--steps",
            "30",
            "--camera",
            str(png),
        ],
        capture_output=True,
        text=True,
        timeout=900,
    )
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    match = re.search(
        r"isaacdrones: camera shape=\((\d+), (\d+), 3\) mean=(\S+) spread=(\S+).*", result.stdout
    )
    assert match, result.stdout[-3000:]
    print(match[0])
    assert (int(match[1]), int(match[2])) == (480, 640)
    assert png.stat().st_size > 0
    # An orange cube on a ground plane is not a flat image.
    assert float(match[4]) > 5.0


@pytest.mark.sim
def test_ros_publishes_clock_and_camera():
    pytest.importorskip("isaacsim")
    steps = 20
    result = subprocess.run(
        [sys.executable, "-m", "isaacdrones.app", "--headless", "--ros", "--steps", str(steps)],
        capture_output=True,
        text=True,
        timeout=900,
    )
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    match = re.search(r"isaacdrones: ros clock=(\d+) images=(\d+)", result.stdout)
    assert match, result.stdout[-3000:]
    print(match[0])
    # The clock is published while the renderer warms up, and the camera only
    # after it, so the clock leads and the images are one per counted frame.
    assert int(match[1]) > int(match[2]) == steps


@pytest.mark.sim
def test_drone_flies_to_the_cube():
    pytest.importorskip("isaacsim")
    from isaacdrones import flight

    # Long enough to finish the flight and settle.
    route = flight.CUBE_FLIGHT
    seconds = route.duration_s + 3.0
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "isaacdrones.app",
            "--headless",
            "--scene",
            "drone",
            "--steps",
            str(round(seconds * 60)),
        ],
        capture_output=True,
        text=True,
        timeout=900,
    )
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    match = re.search(
        r"isaacdrones: drone position=(\S+) (\S+) (\S+) quat_wxyz=(\S+) .* speed=(\S+)",
        result.stdout,
    )
    assert match, result.stdout[-3000:]
    print(match[0])
    x, y, z, w, speed = (float(match[i]) for i in range(1, 6))
    assert abs(x - route.end_m[0]) < 0.05 and abs(y) < 0.05
    assert abs(z - route.end_m[2]) < 0.05
    assert abs(w) > 0.999 and speed < 0.05


@pytest.mark.sim
def test_drone_flies_through_the_warehouse():
    # Downloads NVIDIA's warehouse on first use, so it needs the internet.
    pytest.importorskip("isaacsim")
    from isaacdrones import flight

    route = flight.WAREHOUSE_FLIGHT
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "isaacdrones.app",
            "--headless",
            "--scene",
            "warehouse",
            "--steps",
            str(round((route.duration_s + 3.0) * 60)),
            "--camera",
            "warehouse.png",
        ],
        capture_output=True,
        text=True,
        timeout=1800,
        cwd=os.environ.get("TMPDIR", "/tmp"),
    )
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    match = re.search(r"isaacdrones: drone position=(\S+) (\S+) (\S+) ", result.stdout)
    assert match, result.stdout[-3000:]
    print(match[0])
    assert max(abs(float(match[i + 1]) - route.end_m[i]) for i in range(3)) < 0.05
    # Racks, boxes and lights are far from a flat image.
    match = re.search(r"isaacdrones: camera shape=.* spread=(\S+)", result.stdout)
    assert match and float(match[1]) > 20.0, result.stdout[-3000:]


RESET_SCRIPT = """
from isaacdrones.sim import Simulation

sim = Simulation(headless=True)
sim.add_ground_and_light()
sim.play()
start = sim.time
for _ in range(5):
    sim.step()
before = sim.time
sim.reset()
after = sim.time
for _ in range(5):
    sim.step()
end = sim.time
times = f"start={start:.6f} before={before:.6f} after={after:.6f} end={end:.6f}"
print(f"isaacdrones: reset {times}", flush=True)
sim.close()
"""


@pytest.mark.sim
def test_reset_restarts_time():
    pytest.importorskip("isaacsim")
    result = subprocess.run(
        [sys.executable, "-c", RESET_SCRIPT], capture_output=True, text=True, timeout=900
    )
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    match = re.search(
        r"isaacdrones: reset start=(\S+) before=(\S+) after=(\S+) end=(\S+)", result.stdout
    )
    assert match, result.stdout[-3000:]
    print(match[0])
    start, before, after, end = (float(match[i]) for i in range(1, 5))
    assert before > start
    assert abs(after - start) < 1e-4
    assert abs(end - before) < 1e-4


POSE_SCRIPT = """
from isaacdrones.sim import Simulation

sim = Simulation(headless=True)
sim.add_ground_and_light()
sim.add_cube(falling=True)
sim.play()
position_m, quat_wxyz = sim.cube_pose()
print("isaacdrones: pose start", type(position_m).__name__, *position_m, *quat_wxyz, flush=True)
for _ in range(240):
    sim.step()
position_m, quat_wxyz = sim.cube_pose()
print("isaacdrones: pose end", type(position_m).__name__, *position_m, *quat_wxyz, flush=True)
sim.close()
"""


@pytest.mark.sim
def test_cube_pose_falls_and_lands():
    pytest.importorskip("isaacsim")
    result = subprocess.run(
        [sys.executable, "-c", POSE_SCRIPT], capture_output=True, text=True, timeout=900
    )
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    poses = {}
    for when, kind, numbers in re.findall(r"isaacdrones: pose (\w+) (\w+) (.*)", result.stdout):
        assert kind == "ndarray"
        poses[when] = [float(n) for n in numbers.split()]
    print(poses)
    start, end = poses["start"], poses["end"]
    # Starts 2 m up, tilted 45 degrees (w = cos 22.5 degrees). play() takes
    # two warm-up steps, so it has already moved slightly.
    assert 1.9 < start[2] <= 2.0
    assert abs(start[3] - 0.9239) < 0.05
    # After 4 s it has moved along +X and lies flat, center half a side up.
    assert end[0] > start[0] + 0.5
    assert abs(end[2] - 0.3) < 0.02
    assert abs(sum(q * q for q in end[3:]) - 1.0) < 1e-3


def _fly(seconds, start, goal_at, quat=(1.0, 0.0, 0.0, 0.0)):
    """Integrate a rigid body under the controller, without Isaac Sim, at the
    simulator's rate. `goal_at(t)` gives the arguments of `wrench` after the state.
    Returns the positions, and the final velocity and quaternion.
    """
    from isaacdrones.flight import GRAVITY_MPS2, Drone, State, rotation, wrench

    drone, dt = Drone(), 1.0 / 60.0
    position, velocity = list(start), [0.0, 0.0, 0.0]
    quat, rate = list(quat), [0.0, 0.0, 0.0]  # rate is in the body frame
    inertia = drone.inertia_kgm2
    path = []
    for step in range(round(seconds / dt)):
        r = rotation(quat)
        world_rate = tuple(sum(r[i][k] * rate[k] for k in range(3)) for i in range(3))
        state = State(tuple(position), tuple(quat), tuple(velocity), world_rate)
        thrust, torque = wrench(drone, state, *goal_at(step * dt))
        accel = [thrust / drone.mass_kg * r[i][2] for i in range(3)]
        accel[2] -= GRAVITY_MPS2
        gyro = (
            (inertia[1] - inertia[2]) * rate[1] * rate[2],
            (inertia[2] - inertia[0]) * rate[2] * rate[0],
            (inertia[0] - inertia[1]) * rate[0] * rate[1],
        )
        for i in range(3):
            velocity[i] += accel[i] * dt
            position[i] += velocity[i] * dt
            rate[i] += (torque[i] + gyro[i]) / inertia[i] * dt
        w, x, y, z = quat
        p, q, s = rate
        quat = [
            w + 0.5 * dt * (-x * p - y * q - z * s),
            x + 0.5 * dt * (w * p + y * s - z * q),
            y + 0.5 * dt * (w * q + z * p - x * s),
            z + 0.5 * dt * (w * s + x * q - y * p),
        ]
        norm = sum(c * c for c in quat) ** 0.5
        quat = [c / norm for c in quat]
        path.append(tuple(position))
    return path, velocity, quat


def test_controller_flies_a_rigid_body_to_its_target():
    goal = (2.0, -1.0, 2.0)
    path, velocity, quat = _fly(10.0, (0.0, 0.0, 1.0), lambda t: (goal,))
    assert max(abs(path[-1][i] - goal[i]) for i in range(3)) < 0.02
    assert max(abs(v) for v in velocity) < 0.02
    # Level, with the nose along world x.
    assert abs(quat[0]) > 0.999


def test_flight_stops_short_of_the_cube():
    from isaacdrones import flight

    route = flight.CUBE_FLIGHT
    path, velocity, _ = _fly(route.duration_s + 3.0, route.start_m, route.target)
    assert abs(path[-1][0] - route.end_m[0]) < 0.02 and max(abs(v) for v in velocity) < 0.02
    # The camera is 0.2 m ahead of center and the cube's face is at -0.5 m.
    assert max(x for x, _, _ in path) + 0.2 < -0.5 - 0.2
    assert max(abs(z - route.start_m[2]) for _, _, z in path) < 0.05


def test_flight_follows_a_line_in_any_direction():
    from isaacdrones.flight import Flight

    # Along -y and climbing, so the nose must turn a quarter turn to the right.
    route = Flight(start_m=(1.0, 4.0, 1.0), end_m=(1.0, -2.0, 2.0))
    assert abs(route.yaw_rad + 3.14159265 / 2) < 1e-6
    path, velocity, quat = _fly(route.duration_s + 3.0, route.start_m, route.target)
    assert max(abs(path[-1][i] - route.end_m[i]) for i in range(3)) < 0.02
    assert max(abs(v) for v in velocity) < 0.02
    assert max(abs(x - 1.0) for x, _, _ in path) < 0.05
    # Level, turned -90 degrees about z.
    assert abs(abs(quat[0]) - 0.7071) < 0.01 and abs(abs(quat[3]) - 0.7071) < 0.01
