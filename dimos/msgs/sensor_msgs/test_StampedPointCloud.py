#!/usr/bin/env python3
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

import numpy as np

from dimos.msgs.helpers import resolve_msg_type
from dimos.msgs.sensor_msgs.StampedPointCloud import StampedPointCloud


def test_per_point_time_round_trip() -> None:
    """Per-point time (and intensity) survive an lcm_encode -> lcm_decode round trip."""
    points = np.array(
        [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0], [0.0, 0.0, 0.0]],
        dtype=np.float32,
    )
    intensities = np.array([0.25, 1.1, 0.0, 3.0], dtype=np.float32)
    # Seconds relative to the scan-start timestamp (small => exact in float32).
    times = np.array([0.0, 0.03, 0.06, 0.099], dtype=np.float32)

    original = StampedPointCloud.from_numpy(
        points, frame_id="lidar", timestamp=42.5, intensities=intensities, times=times
    )

    # Getter works before encoding.
    src_t = original.times_f32()
    assert src_t is not None
    np.testing.assert_allclose(src_t, times, rtol=0, atol=1e-7)

    decoded = StampedPointCloud.lcm_decode(original.lcm_encode())

    # Points preserved.
    op, _ = original.as_numpy()
    dp, _ = decoded.as_numpy()
    assert len(op) == len(dp) == 4
    np.testing.assert_allclose(op, dp, rtol=1e-6, atol=1e-6)

    # Per-point time preserved.
    dec_t = decoded.times_f32()
    assert dec_t is not None, "times_f32() returned None after decode"
    np.testing.assert_allclose(dec_t, times, rtol=0, atol=1e-7)

    # Intensity preserved.
    dec_i = decoded.intensities_f32()
    assert dec_i is not None
    np.testing.assert_allclose(dec_i, intensities, rtol=0, atol=1e-7)

    # Frame and (scan-start) timestamp preserved.
    assert decoded.frame_id == "lidar"
    assert decoded.ts is not None and abs(decoded.ts - 42.5) < 1e-6


def test_wire_compatible_with_plain_pointcloud2() -> None:
    """Bytes are a valid sensor_msgs.PointCloud2: a plain PointCloud2 decodes x/y/z/intensity."""
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    intensities = np.array([0.5, 0.75], dtype=np.float32)
    times = np.array([0.0, 0.05], dtype=np.float32)

    raw = StampedPointCloud.from_numpy(
        points, frame_id="lidar", timestamp=1.0, intensities=intensities, times=times
    ).lcm_encode()

    # A consumer using the standard wrapper still reads geometry + intensity (ignores t).
    plain = PointCloud2.lcm_decode(raw)
    pp, _ = plain.as_numpy()
    np.testing.assert_allclose(pp, points, rtol=1e-6, atol=1e-6)
    pi = plain.intensities_f32()
    assert pi is not None
    np.testing.assert_allclose(pi, intensities, rtol=0, atol=1e-7)


def test_empty_cloud_round_trip() -> None:
    """An empty cloud encodes/decodes without error and stays empty."""
    empty = StampedPointCloud.from_numpy(
        np.zeros((0, 3), dtype=np.float32), frame_id="lidar", timestamp=7.0
    )
    decoded = StampedPointCloud.lcm_decode(empty.lcm_encode())
    assert len(decoded) == 0
    assert decoded.times_f32() is None


def test_msg_name_resolves() -> None:
    """The transport can route the type string back to this class."""
    assert StampedPointCloud.msg_name == "sensor_msgs.StampedPointCloud"
    assert resolve_msg_type("sensor_msgs.StampedPointCloud") is StampedPointCloud
