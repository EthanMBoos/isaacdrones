# To do

The goal is to find out how realistic a camera feed Isaac Sim can give a drone.
ROS 2 is the only integration: no autopilot or flight stack is connected.

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

Camera:

- lens distortion, checked by calibrating on a checkerboard in the scene;
- noise, exposure and motion blur;
- depth and a point cloud;
- a place by latitude and longitude, a camera that looks straight down, a sky;
- RTX Minimal against the default and higher sample counts: look and cost.

Other sensors:

- an IMU, which needs a useful rate from 60 Hz physics;
- RTX lidar and radar, both untried;
- an altimeter and a position fix, from the true pose plus noise.

In ROS 2:

- fly the drone from velocity commands;
- detect AprilTags in the warehouse;
- visual odometry against the true path;
- an estimator on the camera, IMU and altimeter.

Open:

- Google's terms bar storing or analyzing the tiles. Decide how the campus
  scene may be used, and show Google's credit;
- `--video` and `--stream` have not run since the camera code last changed.

Later, each when something needs it:

- a measure of how realistic the feed is, possibly against the real flights in
  splatsim;
- a rotor model, and collisions between the drone and the scene: the cube has
  no collider, so the drone would pass through it;
- pacing to real time outside `--stream`.
