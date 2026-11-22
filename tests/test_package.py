import ast
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
    # Publishing starts before the renderer has an image, so the clock leads.
    assert int(match[1]) >= int(match[2]) >= steps


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
