"""Lifecycle wrapper around Isaac Sim. Owns the app, the stage and the loop.

Isaac Sim modules exist only while `SimulationApp` is running, so methods
import them after `__init__` has started the app.

Isaac Sim returns Warp arrays. Methods here convert to NumPy, so the rest of
isaacdrones never sees Warp. Quaternions are wxyz.

Verified headless on PACE ICE on an H100 and a Blackwell (docs/PACE.md).
"""

from __future__ import annotations

import math
import os
from pathlib import Path

KIT = Path(__file__).with_name("isaacdrones.kit")
# Where the drone's camera sits in the drone's frame.
DRONE_CAMERA_M = (0.2, 0.0, 0.0)
# Google Photorealistic 3D Tiles, as a Cesium ion asset.
GOOGLE_TILES_ASSET = 2275207


def _quat_multiply(a, b):
    """The rotation `b` followed by `a`, both wxyz."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


class Simulation:
    """One Isaac Sim application with one stage.

    `step()` runs one whole Kit frame: physics, OmniGraph, sensors, rendering.
    The frame and physics timesteps are both `dt`, so one frame is one physics
    step. Nothing else in isaacdrones steps the simulator.
    """

    def __init__(
        self,
        *,
        headless: bool = False,
        dt: float = 1.0 / 60.0,
        stream_ip: str | None = None,
        cesium_dir: str | None = None,
    ) -> None:
        """`stream_ip` streams the Isaac Sim GUI to the WebRTC Streaming
        Client. It is the address the client connects to.

        `cesium_dir` is the folder that holds the Cesium for Omniverse
        extensions, and starts the app with them (docs/CESIUM.md).
        """
        os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

        from isaacsim import SimulationApp

        # limit_cpu_threads keeps Isaac Sim's default of 16.
        self._cesium = None
        if cesium_dir is not None:
            # Cesium registers USD schemas, so it must be enabled as the app
            # starts, and it imports GUI extensions that our app file leaves
            # out, so it runs in the stock app.
            self._app = SimulationApp(
                {
                    "headless": headless,
                    "extra_args": [
                        "--ext-folder",
                        cesium_dir,
                        "--enable",
                        "cesium.usd.plugins",
                        "--enable",
                        "cesium.omniverse",
                    ],
                }
            )
        elif stream_ip is None:
            # Our app file loads only the extensions we use: under 10 s to
            # start on ICE, against 150 s for the stock app. Drop `experience`
            # to run the stock app.
            self._app = SimulationApp({"headless": headless}, experience=str(KIT))
        else:
            # Streaming needs the menus and panels, so it uses the stock app.
            # Follows Isaac Sim's standalone example livestream.py.
            stream = "--/exts/omni.kit.livestream.app/primaryStream/"
            self._app = SimulationApp(
                {
                    "headless": True,
                    "hide_ui": False,
                    "window_width": 1920,
                    "window_height": 1080,
                    "display_options": 3286,
                    "extra_args": [
                        f"{stream}publicIp={stream_ip}",
                        f"{stream}signalPort=49100",
                        f"{stream}streamPort=47998",
                    ],
                }
            )
            from isaacsim.core.experimental.utils.app import enable_extension

            self._app.set_setting("/app/window/drawMouse", True)
            enable_extension("omni.kit.livestream.app")
        self.dt = dt

        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.rendering_manager import RenderingManager
        from isaacsim.core.simulation_manager import SimulationManager

        stage_utils.create_new_stage()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        stage_utils.set_stage_up_axis("Z")

        # Physics dt and frame dt are separate settings; equal values give
        # one physics step per frame.
        SimulationManager.setup_simulation(dt=dt)
        RenderingManager.set_dt(dt)
        self._ros = None
        self._camera = None
        self._drone = None
        # Called at the start of every step, before physics. The drone's
        # thrust and torque are applied here.
        self.before_step = None

    def add_ground_and_light(self) -> None:
        from isaacsim.core.experimental.objects import DistantLight, DomeLight, GroundPlane

        GroundPlane("/World/GroundPlane", positions=[0, 0, 0])
        # The distant light shines straight down and lights only tops. The
        # dome light reaches the sides.
        DistantLight("/World/DistantLight").set_intensities(300)
        DomeLight("/World/DomeLight").set_intensities(500)

    def add_isaac_asset(self, asset: str, path: str) -> None:
        """Add one of NVIDIA's hosted Isaac Sim assets to the stage at `path`.
        `asset` is its path under the asset root, as the Isaac Sim
        documentation lists it (e.g., `/Isaac/Environments/Office/office.usd`).
        It is downloaded on first use, which needs the internet.
        """
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.utils.app import enable_extension

        enable_extension("isaacsim.storage.native")
        self._app.update()
        from isaacsim.storage.native import get_assets_root_path

        root = get_assets_root_path()
        if root is None:
            raise RuntimeError("Isaac Sim's hosted assets could not be reached.")
        stage_utils.add_reference_to_stage(usd_path=root + asset, path=path)

    def add_cesium_tiles(
        self, *, latitude_deg: float, longitude_deg: float, height_m: float, token: str
    ) -> None:
        """Add Google's photorealistic 3D tiles, streamed by Cesium, with the
        world's origin at this place: x east, y north, z up. `height_m` is
        above the WGS84 ellipsoid. Needs `cesium_dir` and the internet.
        """
        import isaacsim.core.experimental.utils.stage as stage_utils
        from cesium.omniverse import extension
        from cesium.omniverse.usdUtils import get_path_to_current_ion_server
        from cesium.usd.plugins.CesiumUsdSchemas import Data, Georeference, Tileset, Tokens
        from isaacsim.core.experimental.objects import DomeLight

        stage = stage_utils.get_current_stage()
        Data.Define(stage, "/Cesium")
        origin = Georeference.Define(stage, "/CesiumGeoreference")
        origin.GetGeoreferenceOriginLatitudeAttr().Set(latitude_deg)
        origin.GetGeoreferenceOriginLongitudeAttr().Set(longitude_deg)
        origin.GetGeoreferenceOriginHeightAttr().Set(height_m)
        tiles = Tileset.Define(stage, "/World/Tiles")
        tiles.GetSourceTypeAttr().Set(Tokens.ion)
        tiles.GetIonAssetIdAttr().Set(GOOGLE_TILES_ASSET)
        tiles.GetIonAccessTokenAttr().Set(token)
        tiles.GetGeoreferenceBindingRel().AddTarget("/CesiumGeoreference")
        server = get_path_to_current_ion_server()
        if server:
            tiles.GetIonServerBindingRel().AddTarget(server)
        # The tiles are photographs, with the sunlight of their day in them,
        # so the scene adds even light and no sun.
        DomeLight("/World/DomeLight").set_intensities(700)
        self._cesium = extension

    def _load_cesium_tiles(self) -> None:
        """Have Cesium load the tiles the camera sees, and wait for them.
        Cesium's own update does not wait, so a moving camera would record
        tiles arriving late, differently on every run.
        """
        from cesium.omniverse.bindings import Viewport
        from omni.kit.viewport.window import get_viewport_window_instances

        views = []
        for window in get_viewport_window_instances():
            api = window.viewport_api
            view = Viewport()
            view.viewMatrix, view.projMatrix = api.view, api.projection
            view.width, view.height = float(api.resolution[0]), float(api.resolution[1])
            views.append(view)
        self._cesium._cesium_omniverse_interface.on_update_frame(views, True)

    def add_cube(self, *, falling: bool = False) -> None:
        """Add an orange cube: a 1 m cube resting at the origin, or with
        `falling` a 0.6 m rigid body that starts 2 m up, tilted and spinning.
        """
        from isaacsim.core.experimental.objects import Cube

        if not falling:
            Cube("/World/Cube", sizes=1.0, positions=[0.0, 0.0, 0.5], colors="orange")
            return

        from isaacsim.core.experimental.prims import GeomPrim, RigidPrim

        Cube(
            "/World/Cube",
            sizes=0.6,
            positions=[-1.2, 0.0, 2.0],
            orientations=[0.9239, 0.2706, 0.2706, 0.0],  # quat_wxyz, tilted 45 degrees
            colors="orange",
        )
        GeomPrim("/World/Cube", apply_collision_apis=True)
        self._cube = RigidPrim("/World/Cube", masses=[1.0])
        self._cube.set_velocities(
            linear_velocities=[1.2, 0.0, 0.0], angular_velocities=[2.0, 3.0, 1.0]
        )

    def cube_pose(self):
        """World pose of the falling cube from physics: `(position_m,
        quat_wxyz)`, NumPy arrays of shape (3,) and (4,).
        """
        positions, orientations = self._cube.get_world_poses()
        return positions.numpy()[0], orientations.numpy()[0]

    def add_drone(
        self,
        *,
        position_m: tuple[float, float, float],
        quat_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
        mass_kg: float,
        size_m: tuple[float, float, float],
    ) -> None:
        """Add the drone: a rigid box with no rotors, moved by
        `apply_drone_wrench`. Its frame is x forward, y left, z up. PhysX
        derives its inertia from the mass and the box.
        """
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.objects import Cube
        from isaacsim.core.experimental.prims import GeomPrim, RigidPrim

        # The box is a scaled child, so the camera, another child, is not
        # scaled with it.
        stage_utils.define_prim("/World/Drone", "Xform")
        Cube("/World/Drone/Body", sizes=1.0, scales=[list(size_m)], colors="blue")
        GeomPrim("/World/Drone/Body", apply_collision_apis=True)
        self._drone = RigidPrim(
            "/World/Drone",
            positions=[list(position_m)],
            orientations=[list(quat_wxyz)],
            masses=[mass_kg],
            reset_xform_op_properties=True,
        )

    def drone_state(self):
        """The drone's true state from physics: `(position_m, quat_wxyz,
        velocity_mps, angular_velocity_radps)`, each a tuple of floats. The
        velocities are in the world frame.
        """
        positions, orientations = self._drone.get_world_poses()
        linear, angular = self._drone.get_velocities()
        return tuple(
            tuple(float(c) for c in array.numpy()[0])
            for array in (positions, orientations, linear, angular)
        )

    def apply_drone_wrench(self, thrust_n: float, torque_nm: tuple[float, float, float]) -> None:
        """Push the drone along its own z axis and turn it, both in its body
        frame, for the next physics step only.
        """
        self._drone.apply_forces_and_torques_at_pos(
            [[0.0, 0.0, thrust_n]], [list(torque_nm)], local_frame=True
        )

    def add_camera(
        self,
        *,
        position_m: tuple[float, float, float] = (0.0, -8.0, 1.0),
        quat_wxyz: tuple[float, float, float, float] = (0.7071068, 0.7071068, 0.0, 0.0),
        resolution_hw: tuple[int, int] = (480, 640),
        on_drone: bool = False,
        pitch_down_deg: float = 0.0,
        far_m: float = 1000.0,
    ) -> None:
        """Add one RTX camera. The default sits on -Y and looks along +Y with
        world Z up in the image. A USD camera looks down its own -Z with +Y
        up, so that orientation is a quarter turn about X.

        With `on_drone` the camera is fixed to the drone's nose and looks
        along the drone's x axis, tipped down by `pitch_down_deg`, and the
        position and orientation are not used. `far_m` is how far it sees.
        """
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

        path = "/World/Camera"
        if on_drone:
            path = "/World/Drone/Camera"
            # In the drone's frame: 0.2 m ahead of center, clear of the box,
            # looking along x with z up in the image, then tipped down about
            # the drone's y axis.
            half = math.radians(pitch_down_deg) / 2.0
            tip = (math.cos(half), 0.0, math.sin(half), 0.0)
            position_m, quat_wxyz = DRONE_CAMERA_M, _quat_multiply(tip, (0.5, 0.5, -0.5, -0.5))
            # The ROS optical frame is the USD camera turned half a turn
            # about its x axis: z forward and y down.
            self._camera_optical_quat = _quat_multiply(quat_wxyz, (0.0, 1.0, 0.0, 0.0))
        camera = RtxCamera(path, translations=[list(position_m)], orientations=[list(quat_wxyz)])
        from pxr import UsdGeom

        prim = UsdGeom.Camera(stage_utils.get_current_stage().GetPrimAtPath(path))
        if on_drone:
            # A drone passes close to objects, and Isaac Sim's default near
            # clip of 1 m would cut them out of the image.
            camera.camera.set_clipping_ranges(0.01, far_m)
            # A 90 degree horizontal field of view, wide as a drone's camera
            # is: the focal length is half the aperture.
            prim.GetFocalLengthAttr().Set(prim.GetHorizontalApertureAttr().Get() / 2.0)
        self._camera = CameraSensor(camera, resolution=resolution_hw, annotators=["rgb"])
        if self._cesium is not None:
            # Cesium loads the tiles that the viewport's camera sees.
            from omni.kit.viewport.utility import get_active_viewport

            get_active_viewport().camera_path = path

        # Both lengths are in the same unit, so their ratio is the focal
        # length in image widths. Pixels are square.
        focal, aperture = prim.GetFocalLengthAttr().Get(), prim.GetHorizontalApertureAttr().Get()
        self.camera_focal_px = resolution_hw[1] * focal / aperture

    def camera_rgb(self):
        """Latest camera image: NumPy uint8 of shape (height, width, 3). None
        until the renderer has an image.
        """
        data, _ = self._camera.get_data("rgb")
        return None if data is None else data.numpy()

    def play(self) -> None:
        import isaacsim.core.experimental.utils.app as app_utils

        app_utils.play()

    def stop(self) -> None:
        import isaacsim.core.experimental.utils.app as app_utils

        app_utils.stop()

    def reset(self) -> None:
        """Stop and play again, which returns the stage to its initial state.

        Physics and the simulation clock act on the stop only during a Kit
        frame, so one frame runs in between. Without it, time carries on.
        """
        self.stop()
        self._app.update()
        self.play()

    def enable_ros(self, *, image_every: int = 1) -> None:
        """Publish over ROS 2 from now on: `/clock` after every step, and the
        camera's image and info on every `image_every`th step once it has
        one. See `isaacdrones.sim.ros`.
        """
        from isaacdrones.sim.ros import RosPublisher

        self._ros = RosPublisher()
        self._image_every = image_every
        self._steps = 0
        # Set False to hold the camera back, as while the renderer warms up.
        self.publish_images = True
        if self._drone is not None and self._camera is not None:
            self._ros.publish_camera_mount(DRONE_CAMERA_M, self._camera_optical_quat)

    @property
    def ros_counts(self) -> tuple[int, int]:
        """Messages published so far: `(clock, image)`."""
        return self._ros.clock_count, self._ros.image_count

    def step(self) -> None:
        """Run one Kit frame, then publish it if ROS 2 is enabled."""
        if self.before_step is not None:
            self.before_step()
        if self._cesium is not None:
            self._load_cesium_tiles()
        self._app.update()
        if self._ros is None:
            return
        now = self.time
        self._ros.publish_clock(now)
        if self._drone is not None:
            position_m, quat_wxyz, _, _ = self.drone_state()
            self._ros.publish_true_pose(now, position_m, quat_wxyz)
        self._steps += 1
        if not self.publish_images:
            return
        if self._camera is not None and self._steps % self._image_every == 0:
            rgb = self.camera_rgb()
            if rgb is not None:
                self._ros.publish_image(now, rgb, self.camera_focal_px)

    def is_running(self) -> bool:
        # is_exiting covers File > Exit in a streamed GUI.
        return self._app.is_running() and not self._app.is_exiting()

    @property
    def time(self) -> float:
        """Simulation time in seconds. Logic reads this, not wall-clock time.

        Isaac Sim takes two warm-up physics steps in `play()`, so time is
        `2 * dt` when it returns.
        """
        from isaacsim.core.simulation_manager import SimulationManager

        return SimulationManager.get_simulation_time()

    @property
    def num_physics_steps(self) -> int:
        from isaacsim.core.simulation_manager import SimulationManager

        return SimulationManager.get_num_physics_steps()

    @property
    def physics_engine(self) -> str:
        from isaacsim.core.simulation_manager import SimulationManager

        return SimulationManager.get_active_physics_engine()

    def close(self, exit_code: int = 0) -> None:
        """Shut down Isaac Sim. This exits the process, so call it last."""
        if self._ros is not None:
            self._ros.close()
        self._app.close(exit_code=exit_code)
