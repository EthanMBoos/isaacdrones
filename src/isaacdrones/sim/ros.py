"""ROS 2 publishing through Isaac Sim's bridge.

Isaac Sim ships its own ROS 2 Jazzy libraries, `rclpy` included, so nothing is
installed beside it. Enabling `isaacsim.ros2.bridge` loads them, which works
only if their folder was on `LD_LIBRARY_PATH` when the process started;
`library_path` returns that folder and `isaacdrones.app` restarts with it set.
This follows Isaac Sim's standalone example camera_rclpy_async.py, which
publishes from `rclpy` without an OmniGraph.

Every message is stamped with simulation time, so a subscriber with
`use_sim_time` set sees one consistent clock however fast the simulator runs.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

DISTRO = "jazzy"


def library_path() -> Path | None:
    """Folder of the ROS 2 libraries inside the installed isaacsim package,
    or None if Isaac Sim is not installed. Does not import Isaac Sim.
    """
    spec = importlib.util.find_spec("isaacsim")
    if spec is None or not spec.submodule_search_locations:
        return None
    return Path(spec.submodule_search_locations[0]) / "exts/isaacsim.ros2.core" / DISTRO / "lib"


class RosPublisher:
    """Publishes `/clock`, and each of these when given the data:
    `/camera/image_raw` with `/camera/camera_info`, `/truth/pose`, and
    on `/tf_static` the transform from the drone to its camera. Create it
    after `SimulationApp` has started.

    The frames are `world`, `base_link` (the drone: x forward, y left, z up)
    and `camera_optical` (z forward, x right, y down). `/truth/pose` is the
    drone's true pose from physics. It is there to score an estimator
    against, and an estimator must not subscribe to it.
    """

    def __init__(self) -> None:
        from isaacsim.core.experimental.utils.app import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        try:
            import rclpy
        except ModuleNotFoundError as error:
            raise RuntimeError(
                f"The ROS 2 bridge did not load. Start with {library_path()} on LD_LIBRARY_PATH."
            ) from error
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_msgs.msg import TFMessage

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("isaacdrones")
        self._clock = self._node.create_publisher(Clock, "/clock", 10)
        self._image = self._node.create_publisher(Image, "/camera/image_raw", 10)
        self._info = self._node.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self._pose = self._node.create_publisher(PoseStamped, "/truth/pose", 100)
        # Latched, as static transforms are: a late subscriber still gets it.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._tf_static = self._node.create_publisher(TFMessage, "/tf_static", latched)
        self._Clock, self._Image, self._CameraInfo = Clock, Image, CameraInfo
        self._PoseStamped = PoseStamped
        self._TransformStamped, self._TFMessage = TransformStamped, TFMessage
        self.clock_count = 0
        self.image_count = 0

    def _stamp(self, time_s: float):
        from builtin_interfaces.msg import Time

        sec = int(time_s)
        return Time(sec=sec, nanosec=int(round((time_s - sec) * 1e9)) % 1_000_000_000)

    def publish_clock(self, time_s: float) -> None:
        self._clock.publish(self._Clock(clock=self._stamp(time_s)))
        self.clock_count += 1

    def publish_image(self, time_s: float, rgb, focal_px: float) -> None:
        """`rgb` is uint8 of shape (height, width, 3). `focal_px` is the focal
        length in pixels. The principal point is the image center and there
        is no distortion, which is what the RTX camera renders by default.
        """
        height, width = rgb.shape[:2]
        stamp = self._stamp(time_s)

        image = self._Image()
        image.header.stamp = stamp
        # The ROS optical frame: z forward, x right, y down.
        image.header.frame_id = "camera_optical"
        image.height, image.width = height, width
        image.encoding, image.step, image.is_bigendian = "rgb8", width * 3, 0
        image.data = rgb.tobytes()
        self._image.publish(image)

        cx, cy = width / 2.0, height / 2.0
        info = self._CameraInfo()
        info.header = image.header
        info.height, info.width = height, width
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [focal_px, 0.0, cx, 0.0, focal_px, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal_px, 0.0, cx, 0.0, 0.0, focal_px, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info.publish(info)
        self.image_count += 1

    def publish_true_pose(self, time_s: float, position_m, quat_wxyz) -> None:
        pose = self._PoseStamped()
        pose.header.stamp = self._stamp(time_s)
        pose.header.frame_id = "world"
        p, q = pose.pose.position, pose.pose.orientation
        p.x, p.y, p.z = position_m
        q.w, q.x, q.y, q.z = quat_wxyz
        self._pose.publish(pose)

    def publish_camera_mount(self, position_m) -> None:
        """Publish once where the camera sits on the drone. It looks along
        the drone's x axis, so its optical frame is the drone's frame turned
        to put z forward and y down.
        """
        mount = self._TransformStamped()
        mount.header.frame_id = "base_link"
        mount.child_frame_id = "camera_optical"
        t, q = mount.transform.translation, mount.transform.rotation
        t.x, t.y, t.z = position_m
        q.w, q.x, q.y, q.z = 0.5, -0.5, 0.5, -0.5
        self._tf_static.publish(self._TFMessage(transforms=[mount]))

    def close(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()
