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

"""X2 bring-up: drive the robot from the web dashboard while FAST-LIO2 runs on
the chest LiDAR + IMU stream.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.robot.agibot.x2_ultra.connection import X2Connection
from dimos.visualization.vis_module import vis_module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

agibot_x2_teleop_fastlio = (
    autoconnect(
        X2Connection.blueprint(),
        # init_pose left at identity — refine once integrated with the X2 TF tree.
        FastLio2.blueprint(
            input_mode="stream",
            config="agibot_x2.yaml",
            voxel_size=0.05,
            map_voxel_size=0.05,
            map_freq=1.0,
        ),
        vis_module(viewer_backend=global_config.viewer),
    )
    .remappings(
        [
            (X2Connection, "lidar", "fastlio_lidar_in"),
            (FastLio2, "lidar_in", "fastlio_lidar_in"),
            (X2Connection, "imu", "fastlio_imu_in"),
            (FastLio2, "imu_in", "fastlio_imu_in"),
            (WebsocketVisModule, "tele_cmd_vel", "cmd_vel"),
        ]
    )
    .global_config(n_workers=6, robot_model="agibot_x2_ultra")
)

__all__ = ["agibot_x2_teleop_fastlio"]
