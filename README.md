# isaacdrones

isaacdrones is a small base for simulating aerial drones in [NVIDIA Isaac
Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/) 6.2. It starts Isaac
Sim in under 10 s, advances physics by a fixed timestep, and renders an RTX
camera. The only scene is a cube on a ground plane, at rest or falling. No drone
is modeled yet.

The goal is to find out how good Isaac Sim's simulated sensors are for
developing software that will run on a real drone. The first step is to put a
camera and an IMU on a simulated drone, pass their data to a ROS 2 robot, and
look at what arrives. ROS 2 is the only integration: no autopilot or flight
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
uv run isaacdrones --help                       # every flag
```

```bash
uv run pytest -m "not sim"   # the tests that need no simulator
uv run pytest                # adds the tests that boot Isaac Sim
```
