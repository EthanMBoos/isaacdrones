# Design decisions

Written 2026-11-14 and still in force. The survey these decisions rest on is
in [ISAAC_SIM_BEST_PRACTICES.md](ISAAC_SIM_BEST_PRACTICES.md), and what has
run is in [PACE.md](../PACE.md).

## The loop

`Simulation`, in `src/isaacdrones/sim/simulation.py`, is the whole lifecycle.
Isaac Sim 6.0 deprecated `World` and gave it no single replacement, so the
class makes these calls itself:

```python
stage_utils.create_new_stage()
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.set_stage_up_axis("Z")
SimulationManager.setup_simulation(dt=dt)  # physics timestep
RenderingManager.set_dt(dt)  # Kit frame timestep
app_utils.play()
while simulation_app.is_running():
    simulation_app.update()
```

`simulation_app.update()` runs one whole Kit frame: physics, OmniGraph,
sensors and rendering. Isaac Sim's ROS 2 examples loop the same way. The
physics timestep and the frame timestep are separate settings, and both
are set to the same value, which gives one physics step per frame.

The other calls relate to it as follows, according to Isaac Sim's source.
`SimulationManager.step()` steps physics only, and `RenderingManager.render()`
is a Kit frame with physics switched off. Calling `step()` in a loop that also
calls `update()` therefore steps physics twice. If physics is later needed
faster than the frame rate, `Simulation.step()` becomes several `step()` calls
followed by one `render()`. Genie Sim measured a Kit frame at about 2 ms with
the viewport frozen and about 33 ms with it live, which is why fast projects
do this.

Four behaviors of the loop were found by running it on ICE.

- `SimulationApp.close()` exits the process, with status 0 unless told
  otherwise. Nothing runs after it, so a test that boots the simulator does so
  in a subprocess.
- Playing the timeline takes two warm-up physics steps, so simulation time
  starts at two timesteps. Elapsed time is measured from after play.
- A reset is stop, one Kit frame, then play. Physics and the simulation clock
  act on the stop only during a frame, and without it time carries on.
- Camera annotators return no data unless the app file sets
  `omni.replicator.captureOnPlay`, as the stock app does. With the setting off
  the camera gave nothing in 120 frames; with it on an image arrived within 3
  to 16 frames.

## Decisions

**Isaac Sim, not Isaac Lab.** isaacdrones builds directly on Isaac Sim 6.2.
Owning the simulation loop keeps open what is likely to be wanted: the ROS 2
bridge, RTX lidar and radar, tools inside the Isaac Sim GUI, direct access to
engine data, and pacing against an external process such as a flight stack.
Under Lab each of these works against Lab's ownership of the stage and of
stepping.

The price is everything Lab provides, which must be written here if it is
needed: parallel environments with their cloning, layout and resets; batched
asset classes with cached state; sensor wrappers and tiled cameras; actuator
models, controllers, terrain generation and randomisation; the choice of physics
engine or renderer by preset; the RL environment structure; and the config,
benchmark and launch tooling. Lab's buffer against API change is also lost. The
code builds on `isaacsim.core.experimental`, which still changes between
releases, and those changes are absorbed here. If parallel training becomes the
main use, the likely route is to add Lab as a second backend and not to rebuild
it.

**One lifecycle class and one importing package.** The question was how much
of an interface to put between the logic and the engine. The projects
surveyed answer it in five ways, checked against their code between 9 and 14
November 2026:

| Style | Project | What it is | Cost |
|---|---|---|---|
| Lifecycle wrapper | OmniLRS | One class owns play, step, reset and physics settings; other code imports Isaac Sim freely. | about 200 lines |
| Narrow loop interface | Genie Sim | About six methods with NumPy in and out; engines registered by name. | one class per engine |
| Fat base class | ProtoMotions | A shared base with unified state types; each engine fills in about 18 hooks. | 1,300+ lines per engine |
| Per-object factory | Isaac Lab | Every asset and sensor class has a per-engine version. | a package per engine |
| Retrofitted facade | OmniGibson | About 100 methods mirroring PhysX and USD, added late. | high, and most engine code bypasses it |

An interface does not keep a project current: OmniLRS has none and is up to date
on 6.2, while OmniGibson's is mostly nominal. An interface is also cheap to add
later, and one designed with a single engine behind it tends to mirror that
engine. Two things are expensive to retrofit: Isaac Sim imports spread through
every file, and the types that cross the boundary (array type, quaternion order,
whether state is shaped per environment). Those two are therefore fixed and
nothing else. One class owns the lifecycle, and only one package imports Isaac
Sim. A larger interface waits for a second engine or for a mode without Kit, in
which an engine such as Newton runs as a plain Python library; that starts
faster but has no GUI, ROS 2 bridge or Isaac Sim sensors.

Two points bear on a later interface. Genie Sim's has escape hatches
(`raw_model()`, `raw_data()`) that return the native engine objects, so the
interface never blocks access to the engine, and its optional operations are
explicit no-ops, so a backend that lacks one is visible in review. And Isaac
Sim is building its own backend-neutral physics manager, versioned `7.0.0a1`
in the changelog, which may make a home-grown interface redundant.

**Every loop iteration is a Kit frame.** The ROS 2 bridge and the RTX sensors
are extensions that tick once per Kit frame. As long as the loop runs Kit
frames, adding them means enabling an extension and adding nodes or sensor
prims, with no change to the loop; a loop that only steps physics leaves them
frozen. Two rules keep this true. All stepping goes through `Simulation`, and
the logic reads simulation time, never the wall clock.

Two changes would cause real rework, and neither is planned: running without
Kit, where the bridge and the RTX sensors do not exist, and switching to
Newton under Kit, where Genie Sim hit stepping quirks and sensor coverage is
unconfirmed. PhysX is the default.

For ROS the simplest pattern uses no threads: `rclpy.spin_once(node,
timeout_sec=0.0)` inside the loop, as Isaac Sim's examples do. If threads are
used, callbacks only enqueue and the loop is the only writer. The stock app
runs as fast as it can, so pacing to real time is ours to add, starting with a
sleep.

**Conventions follow Isaac Sim's API.** Quaternions are `wxyz`, which is what
the current API returns; units are meters and radians, and Z is up. The
systems disagree on quaternion order, so some conversion is unavoidable: Isaac
Sim's API, USD and MuJoCo put the real part first, and Isaac Lab 3.0, Warp,
PhysX, Newton, ROS and SciPy put it last. Most calls go to Isaac Sim
and the ROS bridge nodes convert for themselves, so matching Isaac Sim gives
the fewest conversions in this code. Names carry the order, as in `quat_wxyz`,
so a mix-up is visible.

The logic uses NumPy, and `Simulation` converts the Warp arrays that Isaac Sim
returns. The projects that run one world on the CPU (OmniLRS, Pegasus, Genie
Sim's engine interface) use NumPy; Torch and Warp pay off with many worlds on
a GPU, the case Isaac Lab and ProtoMotions are built for.

Two rules come from the survey. A simulated pose is never read from USD,
because physics writes to Fabric and USD keeps the starting pose. NumPy
scalars are never passed to `Gf` constructors, which reject them.

Four conventions are left until a robot or an external system fixes them.

- Frames. Pegasus simulates in ENU with a forward-left-up body and converts to
  NED and forward-right-down for PX4. Any flight stack forces this choice.
- Camera axes. USD cameras look down -Z and ROS optical frames down +Z.
- Joint and body order. The same robot file gives different orders in PhysX
  and Newton, so Isaac Lab and ProtoMotions keep explicit name-to-index maps.
- Degrees. USD stores angular velocity in degrees per second.

Other projects name these things as follows. OmniGibson works in `xyzw` and
reorders only where it writes USD. ProtoMotions gives each simulator a
`w_last` flag and converts to one form. Arena puts the order in field names
such as `world_orientation_xyzw`, writes transforms as `T_A_B` (maps points
from B into A), and keeps Isaac Lab's suffixes for the frame a quantity is
expressed in (`_w` world, `_b` base).

**Dependencies.** `pyproject.toml` takes torch 2.11.0 from the `cu128` index,
matching the `torch==2.11.0+cu128` in Isaac Sim 6.2's own `deps/pip_ml.toml`.
Isaac Sim pins `numpy==2.3.1`, which rules out dependencies that need NumPy 1,
and installs its own headless OpenCV, with which `opencv-python` collides.
`SimulationApp` limits itself to 16 CPU threads unless `limit_cpu_threads` is
set.

## Where to look

| Question | Source |
|---|---|
| What replaces a call from the old core API | The "Core API to Core Experimental API" migration page; `source/standalone_examples/api/isaacsim.core.experimental.api/` in the Isaac Sim repo |
| Whether anything 6.0 removed is used | `audit_isaac_sim_6_0.py` in the Isaac Sim repo |
| Physics setup, Newton against PhysX | `skills/physics-simulation/SKILL.md` in the Isaac Sim repo |
| A worked `World` replacement | OmniLRS, `src/simulation.py` |
| Backend interface design | Genie Sim, `genie_sim_engine_core`; OmniGibson, `physics_backends/base.py` |
| Vehicle dynamics and flight-stack lockstep | Pegasus, `logic/vehicles/` and `logic/backends/` |
| Custom sensors | OceanSim, `isaacsim/oceansim/sensors/` |
