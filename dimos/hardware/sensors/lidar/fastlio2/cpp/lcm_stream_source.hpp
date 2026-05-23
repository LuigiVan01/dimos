// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// LcmStreamSource — feeds FAST-LIO from LCM subscriptions instead of the
// Livox SDK. Used when the upstream is a non-Livox sensor whose data already
// arrives as dimos streams (e.g. the X2 Ultra's chest RoboSense E1R LiDAR +
// FORSENSE IMU, delivered by X2Connection).
//
// IMU caveat: this path does NOT multiply accel by GRAVITY_MS2 — the chest
// IMU already reports m/s² and rad/s. Copying the Livox path's multiply
// would silently corrupt the EKF.
//
// Per-point timestamp: the upstream PointCloud2 should carry a `t` field
// (frame-relative seconds). If absent, deskew is disabled (offset_time=0)
// and a one-time warning is printed. See SLAM_SPEC.md for the contract.

#ifndef LCM_STREAM_SOURCE_HPP_
#define LCM_STREAM_SOURCE_HPP_

#include <lcm/lcm-cpp.hpp>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>

#include <boost/make_shared.hpp>

#include "dimos_native_module.hpp"
#include "fast_lio.hpp"
#include "fast_lio_debug.hpp"
#include "lidar_imu_source.hpp"

#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

class LcmStreamSource : public LidarImuSource {
public:
    struct Config {
        std::string lidar_in_topic;
        std::string imu_in_topic;

        static Config from_args(const dimos::NativeModule& mod) {
            // mod.topic() throws if the port is missing, which is what we
            // want — both inputs are mandatory in stream mode.
            Config c;
            c.lidar_in_topic = mod.topic("lidar_in");
            c.imu_in_topic   = mod.topic("imu_in");
            return c;
        }
    };

    LcmStreamSource(FastLio* fast_lio,
                    lcm::LCM& lcm,
                    Config cfg,
                    std::atomic<bool>& running)
        : fast_lio_(fast_lio),
          lcm_(lcm),
          cfg_(std::move(cfg)),
          running_(running) {}

    LcmStreamSource(const LcmStreamSource&) = delete;
    LcmStreamSource& operator=(const LcmStreamSource&) = delete;

    bool start() override {
        // LCM subscribe accepts member function pointers + this directly,
        // so no thunk indirection is needed (unlike the Livox C SDK).
        lcm_.subscribe(cfg_.lidar_in_topic, &LcmStreamSource::on_lidar_in, this);
        lcm_.subscribe(cfg_.imu_in_topic,   &LcmStreamSource::on_imu_in,   this);
        if (fastlio_debug) {
            std::printf("[fastlio2] LcmStreamSource subscribed: lidar_in=%s  imu_in=%s\n",
                        cfg_.lidar_in_topic.c_str(), cfg_.imu_in_topic.c_str());
        }
        return true;
    }

    void stop() override {}

private:
    void on_imu_in(const lcm::ReceiveBuffer*, const std::string&,
                   const sensor_msgs::Imu* msg) {
        if (!running_.load() || msg == nullptr || !fast_lio_) return;

        auto imu_msg = boost::make_shared<custom_messages::Imu>();
        const double ts = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;
        imu_msg->header.stamp = custom_messages::Time().fromSec(ts);
        imu_msg->header.seq = msg->header.seq;
        imu_msg->header.frame_id = msg->header.frame_id;

        imu_msg->orientation.x = 0.0;
        imu_msg->orientation.y = 0.0;
        imu_msg->orientation.z = 0.0;
        imu_msg->orientation.w = 1.0;
        for (int j = 0; j < 9; ++j) imu_msg->orientation_covariance[j] = 0.0;

        imu_msg->angular_velocity.x = msg->angular_velocity.x;
        imu_msg->angular_velocity.y = msg->angular_velocity.y;
        imu_msg->angular_velocity.z = msg->angular_velocity.z;
        for (int j = 0; j < 9; ++j) imu_msg->angular_velocity_covariance[j] = 0.0;

        // CRITICAL: NO GRAVITY_MS2 multiply — chest IMU is already in m/s².
        imu_msg->linear_acceleration.x = msg->linear_acceleration.x;
        imu_msg->linear_acceleration.y = msg->linear_acceleration.y;
        imu_msg->linear_acceleration.z = msg->linear_acceleration.z;
        for (int j = 0; j < 9; ++j) imu_msg->linear_acceleration_covariance[j] = 0.0;

        fast_lio_->feed_imu(imu_msg);
    }

    void on_lidar_in(const lcm::ReceiveBuffer*, const std::string&,
                     const sensor_msgs::PointCloud2* msg) {
        if (!running_.load() || msg == nullptr || !fast_lio_) return;

        // Walk fields[] to find x/y/z (required), intensity + t (optional).
        // Mirrors pgo/cpp/point_cloud_utils.hpp:parse_pointcloud2 — should
        // be factored into common/ once both binaries need it (see SLAM_SPEC).
        int x_off = -1, y_off = -1, z_off = -1, i_off = -1, t_off = -1;
        uint8_t t_dtype = 0;
        for (const auto& f : msg->fields) {
            if      (f.name == "x")         x_off = f.offset;
            else if (f.name == "y")         y_off = f.offset;
            else if (f.name == "z")         z_off = f.offset;
            else if (f.name == "intensity") i_off = f.offset;
            else if (f.name == "t")       { t_off = f.offset; t_dtype = f.datatype; }
        }
        if (x_off < 0 || y_off < 0 || z_off < 0) return;  // malformed cloud

        if (t_off < 0 && !warned_no_t_field_) {
            std::fprintf(stderr,
                "[fastlio2] WARNING: incoming PointCloud2 has no `t` field — "
                "motion deskew disabled (offset_time=0). This is acceptable "
                "for static bring-up but should not pass silently in prod.\n");
            warned_no_t_field_ = true;
        }

        const uint32_t step = msg->point_step;
        const uint32_t n    = static_cast<uint32_t>(msg->width) * msg->height;
        if (step == 0 || n == 0) return;
        if (static_cast<size_t>(step) * n > msg->data.size()) return;

        const double ts = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;

        auto lidar_msg = boost::make_shared<custom_messages::CustomMsg>();
        lidar_msg->header.seq = msg->header.seq;
        lidar_msg->header.stamp = custom_messages::Time().fromSec(ts);
        lidar_msg->header.frame_id = msg->header.frame_id;
        lidar_msg->timebase = static_cast<uint64_t>(ts * 1e9);
        lidar_msg->lidar_id = 0;
        for (int i = 0; i < 3; ++i) lidar_msg->rsvd[i] = 0;
        lidar_msg->point_num = static_cast<uli>(n);
        lidar_msg->points.resize(n);

        for (uint32_t k = 0; k < n; ++k) {
            const uint8_t* base = msg->data.data() + k * step;
            custom_messages::CustomPoint& cp = lidar_msg->points[k];

            float x = 0, y = 0, z = 0;
            std::memcpy(&x, base + x_off, sizeof(float));
            std::memcpy(&y, base + y_off, sizeof(float));
            std::memcpy(&z, base + z_off, sizeof(float));
            cp.x = x; cp.y = y; cp.z = z;

            if (i_off >= 0) {
                float intensity = 0.0f;
                std::memcpy(&intensity, base + i_off, sizeof(float));
                cp.reflectivity = static_cast<uint8_t>(
                    std::clamp(intensity, 0.0f, 255.0f));
            } else {
                cp.reflectivity = 0;
            }
            cp.tag = 0;
            cp.line = 0;

            // Per-point `t` is frame-relative seconds (set on the Python
            // side: absolute per-point stamp minus frame minimum).
            // Convert to ns and store in CustomPoint.offset_time.
            if (t_off >= 0) {
                double t_seconds = 0.0;
                if (t_dtype == sensor_msgs::PointField::FLOAT32) {
                    float t32 = 0.0f;
                    std::memcpy(&t32, base + t_off, sizeof(float));
                    t_seconds = t32;
                } else if (t_dtype == sensor_msgs::PointField::FLOAT64) {
                    std::memcpy(&t_seconds, base + t_off, sizeof(double));
                }
                cp.offset_time = static_cast<uli>(t_seconds * 1e9);
            } else {
                cp.offset_time = 0;
            }
        }

        fast_lio_->feed_lidar(lidar_msg);
    }

    FastLio* fast_lio_;
    lcm::LCM& lcm_;
    Config cfg_;
    std::atomic<bool>& running_;
    bool warned_no_t_field_ = false;
};

#endif  // LCM_STREAM_SOURCE_HPP_
