"""
MuJoCo plumbing of the fer_hand_mjc node, kept apart from what it simulates.

- RealTimeSim: loads a scene and steps it in real time (wall clock) with an optional passive
  viewer; looks up mocap bodies and their shapes.
"""
from __future__ import annotations

import os
import time
from typing import Callable, Tuple

from ament_index_python.packages import get_package_share_directory
import mujoco
import mujoco.viewer


def share_path(*parts: str) -> str:
    """Path inside the installed share directory of fer_hand_mjc."""
    return os.path.join(get_package_share_directory('fer_hand_mjc'), *parts)


class RealTimeSim:
    """
    A MuJoCo scene stepped in real time.

    before_step() is called before every physics step, to place the mocap bodies. step() runs
    the physics up to the wall time elapsed since start() and refreshes the viewer.
    """

    def __init__(self, model_path: str, before_step: Callable[[], None]) -> None:
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self._before_step = before_step
        self._viewer = None
        self._t0 = time.monotonic()

    def mocap_body(self, name: str) -> Tuple[int, int]:
        """Body id and mocap id of a mocap body."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0 or self.model.body_mocapid[body_id] < 0:
            raise ValueError(f"model has no mocap body '{name}'")
        return body_id, int(self.model.body_mocapid[body_id])

    def sphere_radius(self, body_id: int) -> float:
        """Radius (size[0]) of the body's first geom."""
        return float(self.model.geom_size[self.model.body_geomadr[body_id]][0])

    def shape(self, body_id: int) -> Tuple[str, Tuple[float, ...], Tuple[float, ...]]:
        """
        Shape of the body's first geom, for an RViz marker.

        Returns (kind, full extent x/y/z in m, rgba): kind 'sphere' or 'box'; rgba from the
        geom's material if it has one.
        """
        g = self.model.body_geomadr[body_id]
        size = [float(v) for v in self.model.geom_size[g]]
        if self.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            kind, extent = 'box', tuple(2.0 * v for v in size)
        elif self.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
            kind, extent = 'sphere', (2.0 * size[0],) * 3
        else:
            raise ValueError(f'body {body_id}: only sphere and box geoms are supported')
        mat = self.model.geom_matid[g]
        rgba = self.model.mat_rgba[mat] if mat >= 0 else self.model.geom_rgba[g]
        return kind, extent, tuple(float(v) for v in rgba)

    def position(self, body_id: int) -> Tuple[float, float, float]:
        """Body position in the world frame."""
        return tuple(float(v) for v in self.data.xpos[body_id])

    def start(self, viewer: bool, logger) -> None:
        """Open the viewer (if asked and possible) and start the real-time clock."""
        mujoco.mj_forward(self.model, self.data)
        if viewer:
            try:
                # Panels hidden so the scene gets the whole window (Tab / Shift+Tab show them).
                self._viewer = mujoco.viewer.launch_passive(
                    self.model, self.data, show_left_ui=False, show_right_ui=False)
            except Exception as e:  # no display, no OpenGL
                logger.warning(f'{e}; running headless')
        if self._viewer is not None:
            with self._viewer.lock():  # the scene's view (<statistic>, <global>), not the origin
                mujoco.mjv_defaultFreeCamera(self.model, self._viewer.cam)
        self._t0 = time.monotonic()

    def step(self) -> None:
        """Advance the physics to the wall time and refresh the viewer."""
        wall = time.monotonic() - self._t0
        if self._viewer is not None and self._viewer.is_running():
            with self._viewer.lock():
                self._step_to(wall)
            self._viewer.sync()
        else:
            self._step_to(wall)

    def close(self) -> None:
        """Close the viewer and wait for its thread, so GLFW is not torn down under it."""
        if self._viewer is None:
            return
        self._viewer.close()
        deadline = time.monotonic() + 2.0
        while self._viewer.is_running() and time.monotonic() < deadline:
            time.sleep(0.01)

    def _step_to(self, wall: float) -> None:
        if wall - self.data.time > 0.1:  # stalled: skip ahead instead of catching up
            self._t0 += wall - self.data.time
            wall = self.data.time
        while self.data.time < wall:
            self._before_step()
            mujoco.mj_step(self.model, self.data)
