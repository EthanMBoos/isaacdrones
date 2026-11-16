# isaacdrones

isaacdrones is a small base for simulating aerial drones in [NVIDIA Isaac
Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/) 6.2. It starts Isaac
Sim in under 10 s, advances physics by a fixed timestep, and renders an RTX
camera. The only scene is a cube on a ground plane, at rest or falling. No
drone is modelled yet.

The code was the simulator half of
[splatsim](https://github.com/EthanMBoos/splatsim), which drew Gaussian splats
in Isaac Sim until 2026-11-16. The splat code stayed in splatsim, and
isaacdrones is what was underneath it.

Isaac Sim needs Linux or Windows 11 on x86_64 and an NVIDIA RTX GPU. On Linux
it needs Ubuntu 24.04, since 22.04 is deprecated as of 6.2. NVIDIA does not
support a GPU without RT cores, such as an H100; what we measured on one is in
[docs/PACE.md](docs/PACE.md). macOS cannot run the simulator, but the rest of
the package installs there and the tests that need no simulator run.

## Setup

Setup requires [uv](https://docs.astral.sh/uv/). Python is pinned to 3.12,
the only version Isaac Sim 6.x supports.

```bash
uv sync                 # no simulator; works anywhere
uv sync --extra sim     # adds isaacsim 6.2.0.0 and torch; Linux or Windows
git lfs install         # once per machine; USD and textures are in LFS
uv run pre-commit install
```

The first launch compiles shaders and takes a few minutes. isaacdrones accepts
the Omniverse EULA by setting `OMNI_KIT_ACCEPT_EULA=YES`.

## Use

```bash
uv run isaacdrones                           # a window and a ground plane
uv run isaacdrones --headless --steps 100    # no window, 100 frames
uv run isaacdrones --headless --camera out.png    # one image of the cube
uv run isaacdrones --headless --video DIR         # the cube falling, 120 frames
```

| Flag | Effect |
|---|---|
| `--steps N` | Run N frames, one physics step each at 60 Hz. Default: until closed; 30 with `--camera`, 120 with `--video`. |
| `--scene NAME` | `empty`, `cube` or `falling-cube`. |
| `--camera PNG` | Add an RTX camera and save its last image. Default scene: `cube`. |
| `--video DIR` | Save one PNG per frame. Default scene: `falling-cube`. |
| `--stream` | Show the scene in Isaac Sim's GUI over WebRTC. |

On exit isaacdrones prints the frame count, the simulation time and the
physics engine. `ffmpeg -framerate 60 -i DIR/frame_%04d.png out.mp4` turns
saved frames into a video. `--stream` serves the Isaac Sim WebRTC Streaming
Client, which connects to `127.0.0.1`. It needs a GPU with a video encoder,
which an H100 lacks.

```bash
uv run pytest -m "not sim"   # the tests that need no simulator
uv run pytest                # adds the tests that boot Isaac Sim
uv run ruff check . && uv run ruff format .
```

## Layout

```text
src/isaacdrones/
  app.py               # the command line
  sim/simulation.py    # Simulation: the app, the stage and the loop
  sim/isaacdrones.kit  # which Isaac Sim extensions load
  assets/data/         # USD assets (Git LFS)
scripts/pace.sh          # setup and runs on Georgia Tech PACE ICE
scripts/stream_relay.py  # carries the --stream video out of ICE
docs/PACE.md             # running on ICE, and what was verified there
docs/TODO.md             # open work
docs/reference/          # the survey of Isaac Sim projects behind the design
```

Only `src/isaacdrones/sim/` imports `isaacsim`, `omni` or `pxr`, and it does so
inside functions, because those modules exist only after `SimulationApp` has
started. The rest of the package therefore imports on any machine, and a test
enforces the rule. All stepping goes through `Simulation`, and logic reads
`Simulation.time`, not the wall clock. Quaternions are `wxyz`, units are
metres and radians, and Z is up; names carry the order, as in `quat_wxyz`.
The reasons are in
[docs/reference/ISAAC_SIM_NOTES.md](docs/reference/ISAAC_SIM_NOTES.md).

`uv.lock` is committed. Isaac Sim pins some pre-release dependencies, so a
change of the `isaacsim` version needs a matching change to
`constraint-dependencies` in `pyproject.toml`.
