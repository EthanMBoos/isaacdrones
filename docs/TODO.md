# To do

The goal is to find out how good Isaac Sim's simulated sensors are for
developing software that will run on a real drone. ROS 2 is the only
integration: no autopilot or flight stack is connected. The steps below are in
the order they should be taken.

`--scene drone` flies a rigid-body drone at a cube, with a camera on its nose
and an IMU at its center. `--ros` publishes the clock, the camera, the IMU, the
drone's true pose and the transform from the drone to the camera over ROS 2, and
on ICE `scripts/pace.sh bag` records them ([PACE.md](PACE.md)).

The simulated IMU is exact. Over a recorded flight its specific force agrees
with the second difference of the true position to 0.003 m/s^2, and its angular
velocity with the difference of the true orientation to 0.0002 rad/s. Its
readings are in the drone's frame, with +g along z at hover. For the IMU,
therefore, the simulator supplies the motion and none of the error.

**1. IMU noise.** A real IMU has white noise and a bias that drifts, and a
drone's has motor vibration on top. These are added to the readings before they
are published, with the exact readings kept on a second topic for comparison.
The noise densities and bias drift come from the datasheet of a real IMU, and
which one is not decided.

**2. An estimator on the bag.** Run an open-source estimator on the recorded
camera and IMU data and compare its output with the true pose. Which estimator
is not decided. The IMU and the camera both run at 60 Hz, one reading per
physics step, and that rate is taken as given. A visual-inertial estimator is
usually given an IMU several times faster than its camera, so whether one works
at equal rates is part of what this step finds out.

The recorder keeps every message at 640 by 480 with the simulator running
unpaced. A larger image or a second camera may not keep up, and that has not
been tried.

`--video` and `--stream` have not run since the camera code last changed. RTX
lidar and radar are untried.

Later, each when something needs it:

- a rotor model, and collisions between the drone and the scene: the cube has
  no collider, so the drone would pass through it;
- a flight with more motion than a straight line, which an estimator needs to
  observe the IMU's biases;
- RTX lidar and radar on the drone;
- a comparison of the simulated sensors with a real flight log, which is not
  decided;
- pacing to real time outside `--stream`.
