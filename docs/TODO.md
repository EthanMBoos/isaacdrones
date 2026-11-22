# To do

The goal is to find out how good Isaac Sim's simulated sensors are for
developing software that will run on a real drone. The first step is to get
sensor data out of the simulator and into a ROS 2 robot, and look at what
arrives. ROS 2 is the only integration: no autopilot or flight stack is
connected. None of the steps below has been started, and they are in the order
they should be taken.

The ROS 2 bridge comes before the drone. It is the largest unknown, and it can
be tested with the scene that already exists.

**1. The bridge publishes a clock.** Enable `isaacsim.ros2.bridge`, which is
switched off in `src/isaacdrones/sim/isaacdrones.kit`, and publish the
simulation time on `/clock`. Isaac Sim's documentation says the simulator ships
its own ROS 2 libraries (Jazzy on Ubuntu 24.04), so nothing else should need
installing on the simulator's side. This has not been run on ICE.

**2. A receiver on ICE.** The Isaac Sim container has no ROS 2 tools, so
something else must listen. The plan is a second Apptainer container from the
standard ROS 2 Jazzy image, on the same node, running `ros2 topic echo /clock`.
Two containers on one node share the node's network, so they should find each
other, but that is an assumption until it has run.

**3. The camera that already exists, recorded to a bag.** Publish the cube
scene's camera image and its camera info, and record them with `ros2 bag record`
in the receiver. A bag can be opened on a laptop with ROS 2 or Foxglove, so the
data can be examined without the cluster.

**4. A drone.** A rigid body with a thrust and a torque applied at each physics
step, before any rotor model. The camera moves onto it and needs a near clip
below Isaac Sim's default of 1 m, because a drone passes close to objects; 0.01
m has worked.

**5. An IMU.** The IMU comes from the `isaacsim.sensors.experimental.physics`
extension, by Isaac Sim's documentation, and has not been run here. It reads
every physics step and can report specific force. How its readings are published
over ROS 2 was not found in the documentation. The bag also carries the drone's
true pose, which is there to score an estimator against and which an estimator
must not read.

**6. IMU noise.** The documentation describes no noise or bias, so the simulated
IMU is ideal. A real IMU has white noise and a bias that drifts, and a drone's
has motor vibration on top. These have to be added to the readings before the
IMU says anything about a real one. This is the first answer to the question in
the goal: for the IMU, the simulator supplies the motion and none of the error.

**7. An estimator on the bag.** Run an open-source estimator on the recorded
camera and IMU data and compare its output with the true pose. Which estimator
is not decided.

**If steps 1 or 2 fail.** The fallback is to write the bag file directly from
Python with the `mcap` package, without the bridge. The data would still play
back in ROS 2 on another machine, but nothing would run live against the
simulator.

`--video` and `--stream` have not run since the camera code last changed. RTX
lidar and radar are untried.

Later, each when something needs it:

- RTX lidar and radar on the drone;
- a comparison of the simulated sensors with a real flight log, which is not
  decided;
- pacing to real time outside `--stream`;
- body frames and camera axes, which follow the ROS conventions once the
  bridge is in.
