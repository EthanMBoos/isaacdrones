# Repos built on Isaac Sim

Snapshot from 2026-11-14. Star counts are from GitHub that day. "Targets" is
what each repo's README or code states. The repos were found by searching
GitHub for Isaac Sim and Isaac Lab by stars and topic, then adding well-known
frameworks whose names do not mention Isaac. 34 were cloned and read.

The robot-specific Isaac Lab task packs were sampled, not read in full, because
they follow Lab's project template and add little architecturally. Isaac ROS,
cuRobo's internals and the ETH navigation suites were not read.

| Repo | Stars | Targets | Current? |
|---|---|---|---|
| isaac-sim/IsaacLab | 8.6k | Sim 6.2, Lab 3.0 | yes |
| newton-physics/newton | 6.0k | standalone | n/a |
| isaac-sim/IsaacSim | 4.5k | 6.2 | yes |
| mujocolab/mjlab | 3.4k | MuJoCo-Warp | n/a |
| NVlabs/ProtoMotions | 2.5k | Lab 3.0 and others | yes |
| fan-ziqi/robot_lab | 2.0k | Lab 3.0 | yes |
| RoboVerseOrg/RoboVerse | 1.9k | many sims | Lab 2.x style |
| StanfordVL/BEHAVIOR-1K | 1.8k | Sim 6.0 | mixed |
| AgibotTech/genie_sim | 1.6k | Sim 6.2 | yes |
| InternRobotics/InternUtopia | 1.3k | `omni.isaac.*` | no |
| PegasusSimulator | 920 | Sim 5.1 | no |
| LightwheelAI/leisaac | 757 | Lab 2.x | no |
| isaac-sim/IsaacLab-Arena | 655 | Sim 6.2, Lab 3.0 | mostly |
| btx0424/OmniDrones | 591 | `omni.isaac.*` | no |
| unitree_sim_isaaclab | 571 | Lab 3.0 | yes |
| umfieldrobotics/OceanSim | 512 | old core API | no |
| OmniLRS/OmniLRS | 277 | Sim 6.2 | yes |
| space_robotics_bench | 215 | Lab, mid-move | partly |
| DH-Ng/TacEx | 172 | Sim 4.5 | no |
| isaac-sim/IsaacSimZMQ | 61 | Sim 6.0.1 | yes |

The new core API has spread little. Python files importing
`isaacsim.core.experimental` number 826 in Isaac Sim itself, 13 in OmniLRS, 6
in Isaac Lab, and 2 or fewer in every other repo. The deprecated core API is
still imported by 37 files in Genie Sim, 21 in OmniGibson, 20 in TacEx, 15 in
OceanSim and 10 in Pegasus.

## Notes by repository

**Isaac Sim.** The examples to copy from are
`source/standalone_examples/tutorials/getting_started/` and
`source/standalone_examples/api/isaacsim.core.experimental.api/`. The `skills/`
folder has about 45 short guides. There is also a script,
`source/tools/isaac_sim_migration/audit_isaac_sim_6_0.py`, that scans a project
for things 6.0 broke. Cortex was removed in 6.2.

**Isaac Lab 3.0.** Split into a backend-neutral `isaaclab` package and backend
packages (`isaaclab_physx`, `isaaclab_newton`, `isaaclab_ov`). New pieces are
the cloner, a scene-data provider that feeds every renderer from whichever
physics is active, and a benchmark CLI. The recommended way to start your own
project is its template generator, which makes a standalone uv project. The old
extension-template repo is archived.

**Isaac Lab Arena.** Composable environments and policy evaluation on Lab 3.0,
still alpha. Good conventions: no eager imports in `__init__.py`, and explicit
coordinate-frame naming rules.

**Genie Sim (AgiBot).** A
core package defines the physics-engine interface, run loop and a staged
scene-build pipeline, and imports no backend. Backends are Isaac Sim (PhysX or
Newton inside Kit) and standalone Newton. Its `AGENTS.md` files state the
config rules and step modes.

**OmniGibson (BEHAVIOR-1K).** Own kit file, lazy imports, separate physics and
render backend interfaces, and a large object-state system on top of batched
view caches. Sets its Fabric and RTX settings by hand at startup, so its
startup code lists the settings that matter.

**OmniLRS.** Lunar rover simulator, fully migrated to 6.2. Two files are
useful: `docs/migration_plan_6.2.md`, a detailed log of the
migration, and `src/simulation.py`, a short readable replacement for `World`.
They checked each API swap by running old and new side by side in one process
and comparing outputs.

**Pegasus.** Drone simulator with PX4 and ArduPilot support. Its parts are
vehicles, pluggable control backends, sensors updated in physics callbacks,
thrust and drag models. Still on `World` and the old dynamic-control API, so
its design carries over and its calls do not.

**ProtoMotions and RoboVerse.** Both put one simulator interface over several
engines (Isaac Gym, Isaac Lab, Newton, MuJoCo and more) so the same policy can
be checked across them.

**Lab-based robot repos** (Unitree, Lightwheel, robot_lab and many others). A
package of task configs registered with Gymnasium and launched by Lab's scripts.
Unitree's adds DDS topics that match the real robots.

**Add-ons.** OceanSim (sonar and underwater camera from annotators plus Warp),
the EVIS event-camera plugin, TacEx (tactile sensing and a coupled soft-body
solver), and IsaacSimZMQ (a reference bridge to external processes).

**mjlab.** Not Isaac Sim, but relevant: Isaac Lab's API on MuJoCo-Warp with
minimal dependencies. Lab 3.0 borrowed its `ProxyArray` idea from here.

All repos are at `https://github.com/<name>` using the names in the table. The
two written without an owner are `unitreerobotics/unitree_sim_isaaclab` and
`AndrejOrsula/space_robotics_bench`, and Pegasus is
`PegasusSimulator/PegasusSimulator`.
