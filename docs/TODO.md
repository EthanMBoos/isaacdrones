# To do

The goal is to find out how good Isaac Sim's simulated sensors are for
developing software that will run on a real drone. ROS 2 is the only
integration: no autopilot or flight stack is connected. The steps below are in
the order they should be taken.

`--ros` publishes the clock and the camera over ROS 2, and on ICE
`scripts/pace.sh bag` records them ([PACE.md](PACE.md)). The scene is still a
cube, so the steps start with the drone.

**1. A drone.** A rigid body with a thrust and a torque applied at each physics
step, before any rotor model. The camera moves onto it and needs a near clip
below Isaac Sim's default of 1 m, because a drone passes close to objects; 0.01
m has worked.

**2. An IMU.** The IMU comes from the `isaacsim.sensors.experimental.physics`
extension and has not been run here. It reads every physics step and can report
specific force. Its readings are published as `sensor_msgs/Imu` in the same way
as the camera, and the bag also carries the drone's true pose and the transform
from the body to the camera. The true pose is there to score an estimator
against, and an estimator must not read it.

**3. IMU noise.** The documentation describes no noise or bias, so the simulated
IMU is ideal. A real IMU has white noise and a bias that drifts, and a drone's
has motor vibration on top. These have to be added to the readings before the
IMU says anything about a real one. This is the first answer to the question in
the goal: for the IMU, the simulator supplies the motion and none of the error.

**4. An estimator on the bag.** Run an open-source estimator on the recorded
camera and IMU data and compare its output with the true pose. Which estimator
is not decided.

The recorder keeps every message at 640 by 480 with the simulator running
unpaced. A larger image or a second camera may not keep up, and that has not
been tried.

`--video` and `--stream` have not run since the camera code last changed. RTX
lidar and radar are untried.

Later, each when something needs it:

- RTX lidar and radar on the drone;
- a comparison of the simulated sensors with a real flight log, which is not
  decided;
- pacing to real time outside `--stream`;
- body frames, which follow the ROS conventions.
