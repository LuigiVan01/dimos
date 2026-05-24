// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0


#ifndef LIDAR_IMU_SOURCE_HPP_
#define LIDAR_IMU_SOURCE_HPP_

#include <chrono>

class LidarImuSource {
public:
    virtual ~LidarImuSource() = default;

    virtual bool start() = 0;

    virtual void stop() = 0;

    virtual void tick(std::chrono::steady_clock::time_point /*now*/) {}
};

#endif  // LIDAR_IMU_SOURCE_HPP_
