# isaacdrones

isaacdrones is a small base for simulating aerial drones in [NVIDIA Isaac
Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/) 6.2. It starts Isaac
Sim in under 10 s, advances physics by a fixed timestep, and flies a drone that
carries an RTX camera, whose images are published over ROS 2. The drone is a
rigid body moved by a thrust and a torque, with no rotors.

The goal is to find out how realistic a camera feed Isaac Sim can give a drone.
The default scene is a cube on a ground plane and stays light. The next steps
put the drone in a detailed scene, first NVIDIA's warehouse and then real places
streamed as 3D tiles. ROS 2 is the only integration: no autopilot or flight
stack is connected. [docs/TODO.md](docs/TODO.md) has the steps.

Isaac Sim needs Linux or Windows 11 on x86_64 and an NVIDIA RTX GPU. macOS
cannot run the simulator, but the package installs there and the tests that
need no simulator run. At Georgia Tech it runs on PACE ICE; see
[docs/PACE.md](docs/PACE.md).

## Setup

Setup requires [uv](https://docs.astral.sh/uv/).

```bash
uv sync                 # no simulator; works anywhere
uv sync --extra sim     # adds Isaac Sim 6.2 and torch; Linux or Windows
```

The first launch compiles shaders and takes a few minutes.

## Use

```bash
uv run isaacdrones                              # a window and a ground plane
uv run isaacdrones --headless --steps 100       # no window, 100 frames
uv run isaacdrones --headless --camera out.png  # one image of the cube
uv run isaacdrones --headless --ros --scene drone --steps 600
                                                # a flight at the cube, with the
                                                # drone's camera over ROS 2
uv run isaacdrones --help                       # every flag
```

```bash
uv run pytest -m "not sim"   # the tests that need no simulator
uv run pytest                # adds the tests that boot Isaac Sim
```
