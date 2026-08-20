#!/usr/bin/env python3
import rclpy  # Python client library for ROS2
import rclpy.logging
from rclpy.node import Node
from hcoil_interfaces.msg import MagField
import rclpy.parameter
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np
import itertools
import time

# Sweep range, defined here rather than via ROS parameter.
# There is a max jump of 15 in field_node.cpp
Bx_START, Bx_STOP, Nx_STEPS = -10.0, 10.0, 10
By_START, By_STOP, Ny_STEPS = -10.0, 10.0, 10
Bz_START, Bz_STOP, Nz_STEPS = -0.0, 0.0, 1

# Which axes to sweep. Order matters for how the grid is traversed —
# see note below on rate-limit interaction at grid wrap-around points.
AXES = ["x", "y"]  # any subset of "x", "y", "z"

# Timing, in seconds.
STARTUP_SETTLE_TIME = 5.0   # pause after node construction, before sweep begins
STEP_DWELL_TIME = 3.0       # pause after each published step, before advancing


class MinimalPublisher(Node):
    def __init__(self):
        # This is where "field_publisher" becomes a node
        super().__init__('field_publisher')
        # declare an outbound channel - topic := "magfield"
        # field_node.cpp will receive this because it called create_subscription<MagField>("magfield", ...)
        self.publisher_ = self.create_publisher(MagField, 'magfield', 10)

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
        for a in AXES:
            self.get_logger().info(
                "  axis %s: %f to %f, %d steps" %
                (a, active_ranges[a][0], active_ranges[a][-1], len(active_ranges[a])))

        time.sleep(STARTUP_SETTLE_TIME)

    def sweep_fields(self):
        # Full grid: every combination of every active axis's steps.
        # For any axis NOT in AXES, we just loop over a single value (0.0),
        # so the nested loops below still work regardless of which axes are active.
        x_values = self.axis_ranges["x"] if "x" in AXES else [0.0]
        y_values = self.axis_ranges["y"] if "y" in AXES else [0.0]
        z_values = self.axis_ranges["z"] if "z" in AXES else [0.0]

        for bx in x_values:
            for by in y_values:
                for bz in z_values:
                    self._publish(bx, by, bz)
                    time.sleep(STEP_DWELL_TIME)

        # Return to zero field once the sweep is complete.
        self.get_logger().info("Sweep complete, zeroising all axes")
        self._publish(0.0, 0.0, 0.0)

    def _publish(self, bx, by, bz):
        mag = MagField()
        mag.bx = bx
        mag.by = by
        mag.bz = bz
        mag.header.stamp = self.get_clock().now().to_msg()
        mag.header.frame_id = "sweep_field"
        self.get_logger().info("Publishing field: x: %f, y: %f, z: %f" % (bx, by, bz))
        # Actually send a message:
        self.publisher_.publish(mag)


def main(args=None):
    rclpy.init(args=args)  # This opens the communication with ros
    minimal_publisher = MinimalPublisher()
    minimal_publisher.sweep_fields()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
