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
    """Publishes `/clock`, and `/camera/image_raw` with `/camera/camera_info`
    when given images. Create it after `SimulationApp` has started.
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
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("isaacdrones")
        self._clock = self._node.create_publisher(Clock, "/clock", 10)
        self._image = self._node.create_publisher(Image, "/camera/image_raw", 10)
        self._info = self._node.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self._Clock, self._Image, self._CameraInfo = Clock, Image, CameraInfo
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

    def close(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()
