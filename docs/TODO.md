# To do

The goal is to find out how realistic a camera feed Isaac Sim can give a drone.
ROS 2 is the only integration: no autopilot or flight stack is connected. The
steps below are in the order they should be taken.

`--scene drone` flies a rigid-body drone at a cube, `--scene warehouse` flies it
through NVIDIA's warehouse, and `--scene campus` flies it over the Georgia Tech
campus in Google's 3D tiles, streamed by Cesium ([CESIUM.md](CESIUM.md)).
`--ros` publishes the clock, the camera, the drone's true pose and the transform
from the drone to the camera over ROS 2, and on ICE `scripts/pace.sh bag`
records them ([PACE.md](PACE.md)). The default run stays light: the blank and
cube scenes load no assets, the ROS 2 bridge loads only with `--ros`, and a
detailed scene loads what it needs only when it is chosen.

The warehouse renders at 1280 by 720 at about 57 frames per second on a
Blackwell, and the recorder keeps every image at 20 Hz. Its flight crosses the
open floor and stops short of the racks, because their positions have not been
mapped. A flight down an aisle needs them.

Over the campus, buildings, trees and roads are recognizable from 150 m, and the
detail softens toward the ground.

**1. Google's terms.** Read what Google allows to be done with the tiles before
anyone records data from them or trains on it, and show Google's credit with the
imagery, which the recordings do not yet do.

**2. Other places and other views.** The campus, the flight and the camera's 45
degree tilt are fixed in the code. A place given by latitude and longitude, and
a camera that looks straight down, are the obvious options to add.

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
