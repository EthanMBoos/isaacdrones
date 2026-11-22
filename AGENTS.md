# AGENTS.md

isaacdrones is a prototype: research and student code that changes quickly.
Nothing here is fixed.

## How to work here

- Write the simple, direct version. Add an abstraction, config layer or
  interface only when a present need requires it.
- Experiments are welcome. Replace what does not fit.
- `docs/` records where things stand, not rules. If a decision is in the way,
  change it and update the doc.
- Do not gold-plate: no exhaustive tests, no handling of cases that cannot
  happen, no compatibility shims. Keep a test if it catches a real mistake.
- State what was run and what was not. Isaac Sim does not run on macOS, and the
  only machine that runs it is Georgia Tech's PACE ICE cluster, so code that
  touches the simulator is unverified until it has run there.
- The README is written for any Linux machine with an RTX card. ICE specifics go
  in `docs/PACE.md` and `scripts/pace.sh`.

## How to write here

Write docs and comments as a methods section is written: full sentences in
paragraphs, each opening with its claim, followed by the reason and then what
follows from it. Do not write "we"; "I" is fine where Ethan is speaking for
himself. Use American spelling.

- State each fact once, in one place. Elsewhere, point to it.
- Give the measurement and what it was measured on, not an adjective.
- Mark what is measured, what is read from documentation, and what is untried.
- A comment says why the code is so. It does not restate the code or tell the
  story of how it was found out.
- Use a heading only for a section a reader would look up. Within a section,
  open a paragraph with a bold phrase if it needs a label.
- Use a list only for items of one kind. Reasoning goes in prose.
- Wrap Markdown at 80 columns.

## Where things stand

The reasoning is in `docs/reference/ISAAC_SIM_NOTES.md`.

- The repository holds a boot, a fixed-rate loop, an RTX camera, a cube scene
  and `--ros`, which publishes the clock and camera over ROS 2 from `rclpy`
  (`src/isaacdrones/sim/ros.py`). `--scene drone` flies a rigid-body drone at
  the cube with a camera on its nose, and `--scene warehouse` flies it through
  NVIDIA's warehouse. With a drone, `--ros` also publishes `/truth/pose` and
  `/tf_static`. A scene is a function in `app.py` that builds its world and
  gives the drone a `flight.Flight`. `src/isaacdrones/flight.py` holds the
  controller and the flight; it exists to give the sensors a flight to record
  and is not a flight controller under study. The direction is to find out how realistic a
  camera feed Isaac Sim can give a drone, in a detailed scene. ROS 2 is the
  only integration; no autopilot or flight stack is connected. `docs/TODO.md`
  has the steps.
- There is no IMU. Isaac Sim's was tried and is exact, with no noise or bias,
  and it reads once per physics step, at 60 Hz. The work is on the camera.
- The default run stays light: the blank and cube scenes load no assets and no
  extra extensions. A detailed scene loads what it needs only when chosen.
- The Gaussian splat work this grew out of is in the splatsim repository.
  Isaac Sim drew a splat 9 dB worse than its trainer, so splats are not a
  reason to add them back here.
- The code builds directly on Isaac Sim 6.2, not Isaac Lab, on the
  `isaacsim.core.experimental` API. That API changes between releases, and
  that is accepted.
- Only `src/isaacdrones/sim/` imports Isaac Sim (`isaacsim`, `omni`, `pxr`). It
  exposes one lifecycle class, `Simulation`, which owns the loop.
- The loop runs whole Kit frames, so ROS 2 and sensors can be added without
  changing it. Logic reads simulation time, not wall-clock time.
- Quaternions are `wxyz`, units are meters and radians, Z is up. Names carry
  the order, as in `quat_wxyz`. The logic outside the simulator uses NumPy.

## Practical

- `uv sync` works anywhere. `uv sync --extra sim` adds the simulator on Linux
  or Windows.
- `uv run pytest -m "not sim"` runs the tests that need no simulator. Run `uv
  run ruff check . && uv run ruff format .` before committing.
- `uv.lock` is committed. Isaac Sim pins some pre-release dependencies, so a
  change of the `isaacsim` version needs a matching change to
  `constraint-dependencies` in `pyproject.toml`.
- `git lfs install` once per machine, since USD and textures are in LFS, and
  `uv run pre-commit install`.
- `ffmpeg -framerate 60 -i DIR/frame_%04d.png out.mp4` turns `--video` frames
  into a video.
- Create `SimulationApp` before importing anything from `isaacsim` or `omni`.
- `SimulationApp.close()` exits the process, so a test that boots the simulator
  does so in a subprocess.
- On ICE, everything goes through `scripts/pace.sh`. See `docs/PACE.md`.
- Open work is in `docs/TODO.md`.
