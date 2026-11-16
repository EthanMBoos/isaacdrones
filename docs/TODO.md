# To do

Updated 2026-11-16. What has been verified is in [PACE.md](PACE.md), and the
decisions behind the design are in
[reference/ISAAC_SIM_NOTES.md](reference/ISAAC_SIM_NOTES.md).

The next step is a drone: a rigid body with thrust and torque applied at each
physics step, before any rotor model. A camera on a drone will pass close to
objects, so it needs a near clip below Isaac Sim's default of 1 m. The splat
version set 0.01 m and a horizontal field of view, and both were removed with
it.

`--video` and `--stream` have not run since the splat code was removed. RTX
lidar and radar are untried.

Later, each when something needs it:

- the ROS 2 bridge, with an RTX sensor in the loop;
- pacing to real time outside `--stream`;
- body frames, camera axes and joint order, which a flight stack or a
  specific robot will fix.
