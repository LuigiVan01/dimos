# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A point cloud message that preserves a per-point timestamp.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

# Reuse the generic LCM PointCloud2 wire type — no new schema is introduced.
from dimos_lcm.sensor_msgs.PointCloud2 import (
    PointCloud2 as LCMPointCloud2,
)
from dimos_lcm.sensor_msgs.PointField import PointField
from dimos_lcm.std_msgs.Header import Header
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    pass

# PointField datatype enum value for FLOAT32 (ROS sensor_msgs/PointField.FLOAT32 == 7).
_FLOAT32 = 7
# Fixed point layout this message serializes: x, y, z, intensity, t (5 × float32).
_POINT_STEP = 20
_TIME_ATTR = "t"


class StampedPointCloud(PointCloud2):
    """A ``PointCloud2`` that additionally carries a per-point time field (``t``).

    Per-point time is stored on the underlying Open3D tensor cloud under the ``"t"``
    attribute, as float32 seconds relative to ``self.ts`` (see module docstring).
    """

    msg_name = "sensor_msgs.StampedPointCloud"

    @classmethod
    def from_numpy(  # type: ignore[override]
        cls,
        points: np.ndarray,
        frame_id: str = "world",
        timestamp: float | None = None,
        intensities: np.ndarray | None = None,
        times: np.ndarray | None = None,
    ) -> StampedPointCloud:
        """Create a ``StampedPointCloud`` from numpy arrays."""
        pcd_t = o3d.t.geometry.PointCloud()
        pcd_t.point["positions"] = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32)
        if intensities is not None:
            arr = intensities.astype(np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(-1, 1)
            pcd_t.point["intensities"] = o3c.Tensor(arr, dtype=o3c.float32)
        if times is not None:
            tarr = times.astype(np.float32)
            if tarr.ndim == 1:
                tarr = tarr.reshape(-1, 1)
            pcd_t.point[_TIME_ATTR] = o3c.Tensor(tarr, dtype=o3c.float32)
        return cls(pointcloud=pcd_t, ts=timestamp, frame_id=frame_id)

    def times_f32(self) -> np.ndarray | None:
        """Get per-point times as a flat float32 array, or ``None`` if absent.

        Times are seconds relative to ``self.ts`` (the scan-start time).
        """
        self._ensure_tensor_initialized()
        if _TIME_ATTR in self._pcd_tensor.point:
            arr = self._pcd_tensor.point[_TIME_ATTR].numpy().flatten()
            return arr.astype(np.float32) if arr.dtype != np.float32 else arr  # type: ignore[no-any-return]
        return None

    @staticmethod
    def _create_xyzit_fields() -> list:  # type: ignore[type-arg]
        """Create x, y, z, intensity, t field definitions (all FLOAT32)."""
        fields = []
        for i, name in enumerate(["x", "y", "z", "intensity", _TIME_ATTR]):
            field = PointField()
            field.name = name
            field.offset = i * 4
            field.datatype = _FLOAT32
            field.count = 1
            fields.append(field)
        return fields

    def lcm_encode(self, frame_id: str | None = None) -> bytes:  # type: ignore[override]
        """Encode to an LCM ``PointCloud2`` with an x/y/z/intensity/t point layout."""
        msg = LCMPointCloud2()

        msg.header = Header()
        msg.header.seq = 0
        msg.header.frame_id = frame_id or self.frame_id
        ts = self.ts if self.ts is not None else 0.0
        msg.header.stamp.sec = int(ts)
        msg.header.stamp.nsec = int((ts - int(ts)) * 1e9)

        msg.is_dense = True
        msg.is_bigendian = False
        msg.fields = self._create_xyzit_fields()
        msg.fields_length = len(msg.fields)
        msg.point_step = _POINT_STEP

        points = self.points_f32()

        if len(points) == 0:
            msg.height = 0
            msg.width = 0
            msg.row_step = 0
            msg.data_length = 0
            msg.data = b""
            return msg.lcm_encode()  # type: ignore[no-any-return]

        intensities = self.intensities_f32()
        if intensities is None:
            intensities = np.zeros(len(points), dtype=np.float32)
        times = self.times_f32()
        if times is None:
            times = np.zeros(len(points), dtype=np.float32)

        msg.height = 1
        msg.width = len(points)
        # Column order must match _create_xyzit_fields: x, y, z, intensity, t.
        point_data = np.column_stack([points, intensities, times]).astype(np.float32)
        data_bytes = point_data.tobytes()
        msg.row_step = msg.point_step * msg.width
        msg.data_length = len(data_bytes)
        msg.data = data_bytes
        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> StampedPointCloud:  # type: ignore[override]
        """Decode an LCM ``PointCloud2``, extracting x/y/z (+ intensity, + t if present)."""
        msg = LCMPointCloud2.lcm_decode(data)

        ts = (
            msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
            if hasattr(msg, "header") and msg.header.stamp.sec > 0
            else None
        )
        frame_id = msg.header.frame_id if hasattr(msg, "header") else ""

        if msg.width == 0 or msg.height == 0:
            return cls(pointcloud=o3d.geometry.PointCloud(), frame_id=frame_id, ts=ts)

        offsets: dict[str, int] = {f.name: f.offset for f in msg.fields}
        if not all(k in offsets for k in ("x", "y", "z")):
            raise ValueError("StampedPointCloud message missing X, Y, or Z fields")

        num_points = msg.width * msg.height
        raw = msg.data
        step = msg.point_step

        def _field(name: str) -> np.ndarray | None:
            """Extract one FLOAT32 field by name via a strided structured view."""
            off = offsets.get(name)
            if off is None:
                return None
            # Build padding fields only when non-zero — a zero-width "V0" pad is not
            # reliably accepted as a leading field across numpy versions.
            spec: list[tuple[str, str]] = []
            if off > 0:
                spec.append(("_pre", f"V{off}"))
            spec.append((name, "<f4"))
            post = step - off - 4
            if post > 0:
                spec.append(("_post", f"V{post}"))
            return np.frombuffer(raw, dtype=np.dtype(spec), count=num_points)[name].astype(
                np.float32
            )

        xs, ys, zs = _field("x"), _field("y"), _field("z")
        points = np.column_stack([xs, ys, zs]).astype(np.float32)

        pcd_t = o3d.t.geometry.PointCloud()
        pcd_t.point["positions"] = o3c.Tensor(points, dtype=o3c.float32)

        intensities = _field("intensity")
        if intensities is not None and np.any(intensities != 0):
            pcd_t.point["intensities"] = o3c.Tensor(
                intensities.reshape(-1, 1), dtype=o3c.float32
            )

        times = _field(_TIME_ATTR)
        if times is not None:
            pcd_t.point[_TIME_ATTR] = o3c.Tensor(times.reshape(-1, 1), dtype=o3c.float32)

        return cls(pointcloud=pcd_t, frame_id=frame_id, ts=ts)

    def __repr__(self) -> str:
        return (
            f"StampedPointCloud(points={len(self)}, frame_id='{self.frame_id}', "
            f"ts={self.ts}, has_times={self.times_f32() is not None})"
        )


__all__ = ["StampedPointCloud"]
