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

Cesium for Omniverse streams real places as 3D tiles, including Google's
photorealistic tiles. Its release 0.30.0 runs in Isaac Sim 6.2, headless on ICE:
a public sample tileset loaded from a URL, was placed by latitude and longitude,
and rendered through the RTX camera. Three things are needed to run it. The
extension must be enabled when the app starts, because it registers USD schemas.
It needs Isaac Sim's stock app, since it imports GUI extensions that the slim
app leaves out. It needs the `lxml` package, which it cannot install for itself
in this environment.

**1. Real imagery from Cesium.** The sample tileset has no texture, so the probe
says nothing yet about how real places look. Load Google's photorealistic tiles
over a known place and render from 150 m looking down. This needs a Cesium ion
account and its token, read from the environment and never committed. Google's
terms need reading before anyone builds on the tiles.

**2. A Cesium scene.** Add it as a `--scene`, which starts the stock app with
the extension, anchors the world at a latitude and longitude, mounts the camera
looking down and flies at altitude. Cesium chooses which tiles to load from the
camera's view, and can be told to wait until they have loaded, which recording
needs so that two runs of one flight match.

**3. Renderers.** Compare Isaac Sim's default renderer with its path-tracing
mode on the same warehouse frames, for how they look and what they cost.

`--video` and `--stream` have not run since the camera code last changed. RTX
lidar and radar are untried.

Later, each when something needs it:

- lens distortion and sensor noise from Isaac Sim's camera pipeline;
- a measure of how realistic the feed is, possibly against the real flights in
  splatsim;
- a rotor model, and collisions between the drone and the scene: the cube has
  no collider, so the drone would pass through it;
- pacing to real time outside `--stream`.
