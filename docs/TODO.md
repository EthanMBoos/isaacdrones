# To do

The goal is to find out how realistic a camera feed Isaac Sim can give a drone.
ROS 2 is the only integration: no autopilot or flight stack is connected. The
steps below are in the order they should be taken.

`--scene drone` flies a rigid-body drone at a cube, and `--scene warehouse`
flies it through NVIDIA's warehouse, with a camera on its nose. `--ros`
publishes the clock, the camera, the drone's true pose and the transform from
the drone to the camera over ROS 2, and on ICE `scripts/pace.sh bag` records
them ([PACE.md](PACE.md)). The default run stays light: the blank and cube
scenes load no assets, the ROS 2 bridge loads only with `--ros`, and a detailed
scene loads what it needs only when it is chosen.

The warehouse renders at 1280 by 720 at about 57 frames per second on a
Blackwell, and the recorder keeps every image at 20 Hz. The flight crosses the
open floor and stops short of the racks, because their positions have not been
mapped. A flight down an aisle needs them.

**1. Cesium, as a probe.** Cesium for Omniverse streams real places as 3D tiles,
including Google's photorealistic tiles. Its latest release targets Isaac Sim
6.0.1, and nothing found says whether it runs on 6.2 or headless. The probe
installs it on ICE, loads the tiles over a known place and renders from 150 m
looking down. The result is whether to go on, and why. It needs a Cesium ion
token, read from the environment and never committed. Google's terms need
reading before anyone builds on the tiles.

**2. A Cesium scene, if the probe passes.** Anchor the world at a latitude and
longitude, mount the camera looking down, fly at altitude, and wait for the
tiles to load before recording. Streamed tiles can load differently from run to
run, so two runs of one flight may not match.

`--video` and `--stream` have not run since the camera code last changed. RTX
lidar and radar are untried.

Later, each when something needs it:

- lens distortion and sensor noise from Isaac Sim's camera pipeline;
- a measure of how realistic the feed is, possibly against the real flights in
  splatsim;
- a rotor model, and collisions between the drone and the scene: the cube has
  no collider, so the drone would pass through it;
- pacing to real time outside `--stream`.
