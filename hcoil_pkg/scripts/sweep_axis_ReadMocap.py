#!/usr/bin/env python3
# NOTE: this script depends on mocap4r2_msgs (from mocap_ws) in addition to
# hcoil_interfaces (from ros2_ws). Both workspaces must be sourced in this
# shell for this script to run:
#   source /opt/ros/humble/setup.bash
#   source ~/ros2_ws/install/setup.bash
#   source ~/mocap_ws/install/setup.bash

import rclpy
from rclpy.node import Node
from hcoil_interfaces.msg import MagField
from mocap4r2_msgs.msg import Markers
import numpy as np
import time
import csv
import os

# Sweep range, defined here rather than via ROS parameter.
# There is a max jump of 15 in field_node.cpp
Bx_START, Bx_STOP, Nx_STEPS = -10.0, 10.0, 5
By_START, By_STOP, Ny_STEPS = -10.0, 10.0, 5
Bz_START, Bz_STOP, Nz_STEPS = -0.0, 0.0, 1

# Which axes to sweep.
AXES = ["x", "y"]  # any subset of "x", "y", "z"

# Marker indices, per your mocap setup: one fixed reference, one mobile.
REFERENCE_MARKER_INDEX = 1
MOBILE_MARKER_INDEX = 2

# Timing, in seconds.
STARTUP_SETTLE_TIME = 5.0   # pause after node construction, before sweep begins
STEP_DWELL_TIME = 3.0       # pause after each published step, before advancing/logging

OUTPUT_CSV = os.path.expanduser(
    "~/ros2_ws/src/field_displacement_log.csv")

class FieldMocapLogger(Node):
    def __init__(self):
        super().__init__('field_mocap_logger')
        self.publisher_ = self.create_publisher(MagField, 'magfield', 10)
        self.markers_sub_ = self.create_subscription(
            Markers, 'markers', self.on_markers, 10)

        self.latest_markers = None

        loop_rate = 0.5
        self._loop_rate = self.create_rate(loop_rate, self.get_clock())

        for a in AXES:
            if a not in ("x", "y", "z"):
                raise ValueError(f"AXES must contain only 'x','y','z', got {a!r}")

        self.axis_ranges = {
            "x": np.linspace(Bx_START, Bx_STOP, Nx_STEPS),
            "y": np.linspace(By_START, By_STOP, Ny_STEPS),
            "z": np.linspace(Bz_START, Bz_STOP, Nz_STEPS),
        }

        active_ranges = {a: self.axis_ranges[a] for a in AXES}
        total_combinations = int(np.prod([len(v) for v in active_ranges.values()])) if AXES else 0
        self.get_logger().info(
            "Starting node: axes: %s, total combinations: %d" % (AXES, total_combinations))

        self.csv_file = open(OUTPUT_CSV, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'bx', 'by', 'bz',
            'ref_x', 'ref_y', 'ref_z',
            'mobile_x', 'mobile_y', 'mobile_z',
            'disp_x', 'disp_y', 'disp_z', 'disp_magnitude',
        ])
        self.get_logger().info("Logging to %s" % OUTPUT_CSV)

        time.sleep(STARTUP_SETTLE_TIME)

    def on_markers(self, msg):
        # Cache the most recent markers message; consumed during the sweep dwell.
        self.latest_markers = msg

    def _get_marker(self, msg, index):
        for m in msg.markers:
            if m.marker_index == index:
                return m.translation
        return None

    def _wait_and_log(self, bx, by, bz):
        start = time.monotonic()
        while time.monotonic() - start < STEP_DWELL_TIME:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.latest_markers is None:
            self.get_logger().warn("No marker data received yet — skipping log for this step")
            return

        ref = self._get_marker(self.latest_markers, REFERENCE_MARKER_INDEX)
        mobile = self._get_marker(self.latest_markers, MOBILE_MARKER_INDEX)

        if ref is None or mobile is None:
            self.get_logger().warn(
                "Reference or mobile marker not found in latest message — skipping log for this step")
            return

        dx = mobile.x - ref.x
        dy = mobile.y - ref.y
        dz = mobile.z - ref.z
        disp_mag = (dx**2 + dy**2 + dz**2) ** 0.5

        self.get_logger().info(
            "b=(%.3f,%.3f,%.3f) disp=(%.4f,%.4f,%.4f) |disp|=%.4f" %
            (bx, by, bz, dx, dy, dz, disp_mag))

        self.csv_writer.writerow([
            bx, by, bz,
            ref.x, ref.y, ref.z,
            mobile.x, mobile.y, mobile.z,
            dx, dy, dz, disp_mag,
        ])
        self.csv_file.flush()

    def _publish(self, bx, by, bz):
        mag = MagField()
        mag.bx = bx
        mag.by = by
        mag.bz = bz
        mag.header.stamp = self.get_clock().now().to_msg()
        mag.header.frame_id = "sweep_field"
        self.get_logger().info("Publishing field: x: %f, y: %f, z: %f" % (bx, by, bz))
        self.publisher_.publish(mag)

    def sweep_fields(self):
        x_values = self.axis_ranges["x"] if "x" in AXES else [0.0]
        y_values = self.axis_ranges["y"] if "y" in AXES else [0.0]
        z_values = self.axis_ranges["z"] if "z" in AXES else [0.0]

        for bx in x_values:
            for by in y_values:
                for bz in z_values:
                    self._publish(bx, by, bz)
                    self._wait_and_log(bx, by, bz)

        self.get_logger().info("Sweep complete, zeroising all axes")
        self._publish(0.0, 0.0, 0.0)
        self._wait_and_log(0.0, 0.0, 0.0)
        self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = FieldMocapLogger()
    node.sweep_fields()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
