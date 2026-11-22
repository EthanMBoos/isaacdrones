"""Command-line entry point. Boots Isaac Sim with a ground plane and steps it.

This module does not import Isaac Sim; `isaacdrones.sim` does.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import traceback

# Limit on the wait for the renderer's first camera image.
CAMERA_WAIT_S = 180


# What --scene can put in the world besides the ground and lights. Every mode
# (plain, --camera, --video, --stream) uses this table.
SCENES = {
    "empty": lambda sim: None,
    "cube": lambda sim: sim.add_cube(),
    "falling-cube": lambda sim: sim.add_cube(falling=True),
    "drone": lambda sim: _add_drone(sim),
}


def _add_drone(sim) -> None:
    """A cube, and a drone that hovers until `sim.start_flight()` and then
    flies at the cube. A camera, if there is one, rides on the drone.
    """
    from isaacdrones import flight

    drone = flight.Drone()
    sim.add_cube()
    sim.add_drone(position_m=flight.START_M, mass_kg=drone.mass_kg, size_m=drone.size_m)
    started_at = [None]

    def fly() -> None:
        elapsed = -1.0 if started_at[0] is None else sim.time - started_at[0]
        state = flight.State(*sim.drone_state())
        sim.apply_drone_wrench(*flight.wrench(drone, state, *flight.target(elapsed)))

    def start_flight() -> None:
        started_at[0] = sim.time

    sim.before_step = fly
    sim.start_flight = start_flight


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="isaacdrones")
    parser.add_argument("--headless", action="store_true", help="Run without the UI window.")
    parser.add_argument(
        "--steps",
        type=int,
        default=0,
        help="Kit frames to run, one physics step each. Default: until closed, or 30 "
        "with --camera and 120 with --video.",
    )
    parser.add_argument(
        "--camera",
        metavar="PNG",
        help="Add a cube and a camera, and save the camera's last image to this file.",
    )
    parser.add_argument(
        "--video",
        metavar="DIR",
        help="Add a falling cube and a camera, and save one PNG per frame of --steps "
        "(default 120) into this folder.",
    )
    parser.add_argument(
        "--scene",
        choices=sorted(SCENES),
        help="What is in the world besides the ground and lights. drone adds a cube "
        "and a drone that flies at it, carrying the camera. Default: empty, or cube "
        "with --camera or --ros, or falling-cube with --video.",
    )
    parser.add_argument(
        "--stream",
        nargs="?",
        const="127.0.0.1",
        metavar="IP",
        help="Show the scene in the Isaac Sim GUI, streamed to the Isaac Sim WebRTC "
        "Streaming Client. It starts paused; press Play in the client. IP is the "
        "address the client connects to (default 127.0.0.1).",
    )
    parser.add_argument(
        "--ros",
        action="store_true",
        help="Add a cube and a camera, and publish /clock, /camera/image_raw and "
        "/camera/camera_info over ROS 2, stamped with simulation time.",
    )
    return parser.parse_args(argv)


def _restart_with_ros_libraries() -> None:
    """Isaac Sim's ROS 2 libraries load only if their folder was on
    LD_LIBRARY_PATH when the process started, so start again with it there.
    """
    from isaacdrones.sim.ros import library_path

    path = library_path()
    current = os.environ.get("LD_LIBRARY_PATH", "")
    if path is None or str(path) in current.split(":"):
        return
    os.environ["LD_LIBRARY_PATH"] = f"{current}:{path}" if current else str(path)
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.execv(sys.executable, [sys.executable, "-m", "isaacdrones.app", *sys.argv[1:]])


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    if args.ros and argv is None:
        _restart_with_ros_libraries()

    from isaacdrones.sim import Simulation

    sim = Simulation(headless=args.headless, stream_ip=args.stream)
    exit_code = 1
    try:
        _run(sim, args)
        exit_code = 0
    except Exception:
        # close() exits the process, so print the error before it does.
        traceback.print_exc()
    finally:
        sim.close(exit_code)


def _run(sim, args: argparse.Namespace) -> None:
    sim.add_ground_and_light()
    has_camera = bool(args.video or args.camera or args.ros)
    scene = args.scene or ("falling-cube" if args.video else "cube" if has_camera else "empty")
    SCENES[scene](sim)
    if args.stream:
        _stream(sim)
        return
    if has_camera:
        sim.add_camera(on_drone=scene == "drone")
    if args.ros:
        sim.enable_ros()
    sim.play()
    if args.ros:
        # Wait for the renderer, so that every counted frame has an image.
        _wait_for_camera(sim)
    if scene == "drone":
        sim.start_flight()
    if args.video:
        _save_video(sim, args.video, args.steps or 120)
        return

    # play() takes two warm-up physics steps, so time does not start at zero.
    start_time = sim.time
    frames = 0
    # --camera is a one-shot, so without --steps it must still end.
    steps = args.steps or (30 if args.camera else 0)
    while sim.is_running() and (steps == 0 or frames < steps):
        sim.step()
        frames += 1

    # The sim test parses this line.
    print(
        f"isaacdrones: frames={frames} elapsed={sim.time - start_time:.6f} "
        f"sim_time={sim.time:.6f} dt={sim.dt:.6f} "
        f"physics_steps={sim.num_physics_steps} engine={sim.physics_engine}",
        flush=True,
    )
    if scene == "drone":
        position, quat, velocity, _ = sim.drone_state()
        print(
            "isaacdrones: drone position="
            + " ".join(f"{c:.3f}" for c in position)
            + " quat_wxyz="
            + " ".join(f"{c:.3f}" for c in quat)
            + " speed="
            + f"{sum(c * c for c in velocity) ** 0.5:.3f}",
            flush=True,
        )
    if args.ros:
        clock, images = sim.ros_counts
        print(f"isaacdrones: ros clock={clock} images={images}", flush=True)
    if args.camera:
        _save_camera_image(sim, args.camera)


def _stream(sim) -> None:
    # Left paused, so whoever connects sees the scene from its start.
    print("isaacdrones: stream ready", flush=True)
    while sim.is_running():
        # Paced to real time for the viewer, hence the wall clock.
        frame_start = time.monotonic()
        sim.step()
        time.sleep(max(0.0, sim.dt - (time.monotonic() - frame_start)))


def _wait_for_camera(sim):
    """Step until the camera has an image. Returns it (or None) and the frames run."""
    # The first image arrives when the renderer has loaded its shaders, which
    # takes seconds regardless of frames: 300 frames have passed in a few
    # seconds with no image. The limit is therefore on the wall clock.
    deadline = time.monotonic() + CAMERA_WAIT_S
    rgb = sim.camera_rgb()
    frames = 0
    while rgb is None and time.monotonic() < deadline:
        time.sleep(0.02)
        sim.step()
        frames += 1
        rgb = sim.camera_rgb()
    if rgb is None:
        print(f"isaacdrones: camera produced no image in {CAMERA_WAIT_S} s", flush=True)
        return None, frames

    # The first image is grainy; it settles within about 30 frames.
    for _ in range(30):
        sim.step()
    later = sim.camera_rgb()
    return (rgb if later is None else later), frames


def _save_camera_image(sim, path: str) -> None:
    rgb, extra_frames = _wait_for_camera(sim)
    if rgb is None:
        return

    from pathlib import Path

    from PIL import Image  # comes with Isaac Sim

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rgb).save(path)
    # A black or flat image has a spread near zero.
    print(
        f"isaacdrones: camera shape={rgb.shape} mean={rgb.mean():.1f} spread={rgb.std():.1f} "
        f"extra_frames={extra_frames} saved={path}",
        flush=True,
    )


def _save_video(sim, folder: str, steps: int) -> None:
    from pathlib import Path

    # Wait for the renderer, then reset so the frames cover the whole fall.
    rgb, _ = _wait_for_camera(sim)
    if rgb is None:
        return
    sim.reset()
    if hasattr(sim, "start_flight"):
        sim.start_flight()
    # The image lags the scene by a few frames, so the first images after a
    # reset show the scene before it. Skip them.
    for _ in range(3):
        sim.step()

    from PIL import Image  # comes with Isaac Sim

    out = Path(folder)
    out.mkdir(parents=True, exist_ok=True)
    saved = 0
    for frame in range(steps):
        sim.step()
        rgb = sim.camera_rgb()
        if rgb is not None:
            Image.fromarray(rgb).save(out / f"frame_{frame:04d}.png")
            saved += 1
    print(f"isaacdrones: video frames={saved} of {steps} sim_time={sim.time:.6f} saved={folder}")


if __name__ == "__main__":
    main()
