# To do

The goal is to find out how realistic a camera feed Isaac Sim can give a drone.
ROS 2 is the only integration: no autopilot or flight stack is connected. The
steps below are in the order they should be taken.

`--scene drone` flies a rigid-body drone at a cube, with a camera on its nose.
`--ros` publishes the clock, the camera, the drone's true pose and the transform
from the drone to the camera over ROS 2, and on ICE `scripts/pace.sh bag`
records them ([PACE.md](PACE.md)). The default run stays light: the blank and
cube scenes load no assets and no extra extensions, and a detailed scene loads
what it needs only when it is chosen.

**1. Scenes that bring their own flight.** `--scene` gains scenes that each
declare their assets, the drone's flight and where its camera points. The blank
and cube scenes are unchanged, and a test holds the default run to loading
nothing more than it does now.

**2. The warehouse.** Load NVIDIA's full warehouse from the hosted Isaac Sim
assets, fly the drone down an aisle, and record a bag and a video on ICE. This
also measures how long the first load takes and whether the downloaded assets
are kept between jobs.

**3. Camera settings.** Raise the resolution from 640 by 480 (e.g., to 1280 by
720), publish at 20 Hz, and compare Isaac Sim's default renderer with its
path-tracing mode on the same frames. The recorder has only been checked at 640
by 480.

**4. Cesium, as a probe.** Cesium for Omniverse streams real places as 3D tiles,
including Google's photorealistic tiles. Its latest release targets Isaac Sim
6.0.1, and nothing found says whether it runs on 6.2 or headless. The probe
installs it on ICE, loads the tiles over a known place and renders from 150 m
looking down. The result is whether to go on, and why. It needs a Cesium ion
token, read from the environment and never committed. Google's terms need
reading before anyone builds on the tiles.

**5. A Cesium scene, if the probe passes.** Anchor the world at a latitude and
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
