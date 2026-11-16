# Isaac Sim practice, November 2026

Findings from reading the source of 34 repos built on Isaac Sim, and the Isaac
Sim 6.0 to 6.2 and Isaac Lab 3.0 release and migration notes. The baseline is
Isaac Sim 6.2.0 and Isaac Lab 3.0.0, both released in October 2026.

Two limits apply. Nothing here was run, so no performance claim is our own
measurement. The release-note details came through a summarising web reader, so
check exact identifiers against the official pages. Everything else was read
directly in code or in-repo docs.

Per-repo details are in [ISAAC_SIM_REPO_SURVEY.md](ISAAC_SIM_REPO_SURVEY.md).
What this means for isaacdrones is in [ISAAC_SIM_NOTES.md](ISAAC_SIM_NOTES.md).

## Changes in 6.0 to 6.2

**`World` is deprecated.** Since 6.0, `isaacsim.core.api`, `isaacsim.core.prims`
and `isaacsim.core.utils` are deprecated, along with the old camera,
physics-sensor, manipulator and wheeled-robot extensions. They still ship in 6.2
but live in a `deprecated` folder and may not be enabled by your app. The
`omni.isaac.*` names were removed outright in 6.0.

There is no single replacement for `World`. Isaac Sim's own examples now use
separate pieces:

- `isaacsim.core.experimental.utils.stage` to create the stage and add
  references
- `isaacsim.core.experimental.utils.app` for `play()` and `stop()`
- `simulation_app.update()` for a whole Kit frame including physics, or
  `SimulationManager.step()` for physics only plus `RenderingManager.render()`
  for a frame without physics. Don't combine the two, or physics steps twice.
- `SimulationManager.register_callback(fn,
  event=SimulationEvent.PHYSICS_POST_STEP)` in place of
  `world.add_physics_callback`. The older `IsaacEvents` enum is deprecated,
  though Isaac Sim's own examples still use it.
- `Articulation`, `RigidPrim` and `GeomPrim` from
  `isaacsim.core.experimental.prims`, which are always batched
- `isaacsim.sensors.experimental.rtx` for cameras and
  `isaacsim.sensors.experimental.physics` for IMU, contact, effort and raycast
  sensors

Four changes cause errors during that move. The old `add_physics_callback` now
fires before the physics step, not after. The new getters return Warp arrays,
not NumPy. The new articulation setters write USD attributes, without
commanding the sim, when the physics tensor view does not exist yet. The old
semantics helpers such as `add_update_semantics` are removed; use `add_labels`.

Few projects outside NVIDIA have made this move. Of the third-party repos read,
only OmniLRS is fully on the new API. Most sit on Isaac Lab, which shields
them.

**Isaac Lab 3.0 no longer assumes Isaac Sim.** It is now a multi-backend
framework where physics, renderer and visualizer are three independent choices.
Physics can be PhysX inside Isaac Sim, Newton with no Isaac Sim at all, or PhysX
without Kit. Rendering can be Isaac Sim RTX, standalone RTX, or a Warp renderer.
You pick with a command-line preset such as `physics=newton_mjwarp` and the task
code stays the same.

Lab also rewrote the Isaac Sim utilities it depended on (stage helpers, the
cloner, the simulation manager) inside its own tree, so that it keeps working
when the deprecated Isaac Sim modules are deleted. It owns the USD stage itself
and turns off Isaac Sim's default simulation-manager callbacks, because those
invalidate the tensor views Lab holds.

If you have Lab 2.x code or copy from 2.x tutorials, these are the breaks:

- Quaternions are now `xyzw`. They were `wxyz`.
- `robot.data.joint_pos` returns a `ProxyArray`. Add `.torch` or `.warp`.
- `write_*_to_sim(data, env_ids)` became `write_*_to_sim_index` and
  `write_*_to_sim_mask`.
- `AppLauncher` became `with launch_simulation(env_cfg, args_cli):`.
- `--headless` is gone. Use `--viz none`.
- `./isaaclab.sh` is gone. Install with `uv sync --extra isaacsim`.
- `TiledCamera` was folded into `Camera`.

**Newton ships, and PhysX remains the default.** Isaac Sim 6.2 bundles Newton
1.6.1 with MuJoCo-Warp. You switch engines with
`SimulationManager.switch_physics_engine("newton")` or a launch setting. The
Newton extension switches to Newton automatically when it is enabled, and in the
source only the Newton-specific app file enables it. One of NVIDIA's own
guides says the standard full app does too, which contradicts the files, so
check which engine the app starts with. Newton views go stale after a
model rebuild and must be recreated.

As a result, each long-lived framework now has its own physics-backend
interface: Isaac Lab, Genie Sim, OmniGibson and ProtoMotions.

**NVIDIA is moving code out of Kit.** The Isaac Sim repo
has a new `source/libraries` tree for code that "must work without Kit", shipped
as plain wheels, with Kit extensions reduced to thin adapters. There is a
standalone RTX renderer (`ovrtx`) and a standalone PhysX (`ovphysx`). These
cannot share a process with Kit because their bundled USD versions clash, so
Kit-less is a separate environment and not a runtime flag.

**Warp is the data layer.** Lab 3.0 keeps simulation state in Warp arrays and
hands out zero-copy torch views. Third-party repos write custom sensors and
terrain as Warp kernels. Warp's own API still moves: OmniLRS hit a compile error
because building a matrix from vectors changed to `wp.matrix_from_cols`.

## Structure of the projects surveyed

**They keep simulator imports out of the core.** Nothing from `isaacsim` or
`omni` is imported at module top level outside the entry point. OmniGibson uses
a lazy-import proxy. Genie Sim has a hard rule that its core package imports no
backend at all, so its tests run on machines with no simulator installed.

**They ship their own `.kit` app file.** Lab has six and picks one by mode,
because headless without cameras needs far fewer extensions. OmniGibson keeps
its changes as a patch against the stock file so upgrades are a re-apply.

**They treat physics step, render and app update as three different things.**
Lab's PhysX step calls `simulate` and `fetch_results` directly and does not pump
the Kit app. OmniLRS goes the other way and lets Kit's loop drive everything
with one `app.update()` per render step, as do most of Isaac Sim's own examples,
including all the ROS 2 ones. Either works. Several explicit physics steps
followed by one render is fine; a plain app update on top of explicit steps runs
physics twice.

**They make pacing a pluggable policy.** Genie Sim has realtime, lockstep and
free-run modes behind one interface. Pegasus does PX4 lockstep by blocking on
the actuator message inside its physics callback.

**They build scenes by cloning prototypes.** Lab 3.0 describes the scene as a
plan (which assets exist, which layout each environment uses, where each sits)
and replays that plan into physics and every renderer. Importers now produce
instanceable assets by default.

**They read state through batched tensor views.** One shared simulation view,
with typed views per asset. Two rules follow. Stopping the timeline invalidates
every view, so recreate them on play. And never read a simulated pose from USD,
because with Fabric on, physics writes to Fabric and USD still holds the
starting pose. NVIDIA calls this the most common silent bug.

**They keep config typed, with one route per setting.** Lab uses config
dataclasses with Hydra overrides and presets. Genie Sim is the strictest: two
YAML files, no environment variables, no hidden module constants, and every
tunable documented with the measurement that set its value.

**They queue commands from other threads.** PhysX ignores drive writes made
while it is stepping, so OmniLRS queues commands from ROS and applies them in
the sim loop.

**They document for coding agents.** Isaac Sim, Isaac Lab, Arena and Genie Sim
all ship `AGENTS.md` and a `skills/` folder of short how-to guides. These state
each project's conventions and are versioned with the code. The two most
relevant here are Isaac Sim's `skills/physics-simulation` and
`skills/isaac-sim-migration`.

For talking to other processes: OmniGraph nodes for standard ROS 2 sensor
topics, raw `rclpy` with the bundled Jazzy libraries for custom logic, a C++
bridge when rate matters (Genie Sim), ZMQ with Protobuf for non-ROS clients, and
MAVLink for flight stacks (Pegasus).

## Performance

The settings every fast repo touches:

- Fabric on and USD write-back off (`/physics/updateToUsd=false`). Only
  transforms go to Fabric; everything else is read through tensors.
- `/physics/suppressReadback` on when using the GPU pipeline.
- GPU contact buffers raised for contact-heavy scenes. OmniGibson uses four
  times the defaults and warns that going much higher can crash.
- Scene queries off unless you need them.
- Tiled rendering, so all environment cameras share one render pass.

Colliders are a common cost. Use the simplest shape that works and avoid
overlapping collision geometry. A warning that a convex mesh "failed to cook as
GPU-compatible" means that collider has dropped to the CPU. Put the rigid-body
and collision APIs on the same prim, and don't scale a prim that carries a
collider; scale a child mesh instead.

GPU physics is not faster for one world. OmniGibson and OmniLRS both default to
CPU physics for single-environment work, and OmniGibson notes NumPy controllers
beat torch there. The GPU pipeline pays off with many parallel environments. CCD
only works on CPU.

Keep data on the GPU. Convert camera frames to NumPy only where they leave the
simulator. Lab now returns some metrics as GPU scalars specifically to avoid a
sync.

The default renderer changed to `RealTimePathTracing`. Old RTX setting keys can
silently do nothing on the new Kit, so read a setting back after writing it.

For measuring, Lab has `isaaclab benchmark runtime`, `startup`, `training` and
`play`, and both Lab and Isaac Sim document Nsight Systems profiling. The tuning
reference NVIDIA points to is the Isaac Sim Performance Optimization Handbook.
No official numbers compare backends.

## Custom features

- **Custom forces (drones, thrusters):** a per-step physics callback that
  applies body forces and torques. Pegasus is the model, though it still uses
  the removed dynamic-control API. It also spins rotor joints purely for looks,
  scaled by thrust.
- **New sensor types:** take stock annotators (point cloud, depth, segmentation)
  and post-process them with Warp kernels. OceanSim builds sonar and an
  underwater camera this way.
- **High-rate event camera:** render sparse keyframes with motion vectors and
  warp the frames in between (the EVIS plugin).
- **Deformable terrain:** OmniLRS does it in plain Python and Warp outside
  PhysX, which is why it survived PhysX dropping its old deformables.
- **Physics PhysX lacks:** TacEx runs a second solver (libuipc) alongside and
  couples it.
- **Render without stepping physics:** turn `/app/player/playSimulations` off
  around an `app.update()` call (OmniLRS).
- **Silence noisy log channels:** OmniGibson wraps specific calls in a
  context manager that suppresses logging.
- **Match a real robot's SDK:** Unitree's sim publishes the same DDS topics as
  the hardware and passes images through shared memory.
- **Edit one instance of an instanceable asset:** Genie Sim expands only the
  instanced geometry wrappers first and never touches shared materials.
- **Clusters:** point Kit's cache and app data at local disk, not a network
  filesystem.

## Migrating to 6.2

These come from OmniLRS's write-up of moving from 5.0 to 6.2, the most detailed
public record found.

- Most of the breakage is at 6.0: Python 3.12, NumPy 2, Kit 110. They advise
  moving straight to 6.2.
- Isaac Sim hard-pins `numpy==2.3.1`. It also installs its own headless OpenCV,
  so adding `opencv-python` breaks `cv2`.
- USD's `Gf.Vec` and `Gf.Quat` constructors now reject NumPy scalars. Cast to
  Python floats.
- Don't write your own `SimulationApp.close()`. Fast shutdown now exits the
  process and has a watchdog you would bypass.
- `SimulationApp` now limits itself to 16 CPU threads by default. It was 32.
- IMU readings changed in 6.1, so revalidate anything that depends on them.
- With Jazzy, `rclpy` handles Ctrl-C itself. Your loop must also check
  `rclpy.ok()` or it never exits.
- The 6.2 container is Ubuntu 24.04 and runs as a non-root user. Ubuntu 22.04 is
  deprecated.
- SciPy 1.17 can't correctly unpickle splines saved by older versions. Combined
  with a bare `except: pass`, that became an infinite hang for them.

## Direction

Interest is shifting to lighter stacks. `mjlab`, which is Isaac Lab's API on
MuJoCo-Warp with no Omniverse, has 3.4k stars, and its stated reason for
existing is Omniverse's install weight and startup time. Lab 3.0's Kit-less mode
is NVIDIA's answer.

Projects that skipped versions have fallen behind. OmniDrones, InternUtopia,
TacEx and Pegasus are all one or more major versions behind, and each skipped
version made the next jump bigger.

## Sources

- Isaac Sim 6.2 release notes:
  https://docs.isaacsim.omniverse.nvidia.com/6.2.0/overview/release_notes.html
- Isaac Sim 6.0 migration guides (start with "Core API to Core Experimental
  API", which was not fetched):
  https://docs.isaacsim.omniverse.nvidia.com/6.1.0/migration_guides/isaac_sim_6_0/migration_strategy.html
- Isaac Lab 3.0 release:
  https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0
- Isaac Lab migration guide:
  https://isaac-sim.github.io/IsaacLab/develop/source/isaaclab/migrating.html
- Performance handbook:
  https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/sim_performance_optimization_handbook.html
- OmniLRS migration write-up: `docs/migration_plan_6.2.md` in
  https://github.com/OmniLRS/OmniLRS
