#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node

from hcoil_interfaces.msg import MagField
import rclpy.parameter
from rcl_interfaces.msg import ParameterDescriptor


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('field_publisher')
        self.publisher_ = self.create_publisher(MagField, 'magfield', 10)
        self.actual_sub_ = self.create_subscription(
            MagField, 'magfield_actual', self.on_actual, 10)

        field_desc = ParameterDescriptor(
            description="Field in mT.",
            )
        self.declare_parameter("bx", 0.0, field_desc)
        self.declare_parameter("by", 0.0, field_desc)
        self.declare_parameter("bz", 0.0, field_desc)

        self.bx_ = self.get_parameter("bx").get_parameter_value().double_value
        self.by_ = self.get_parameter("by").get_parameter_value().double_value
        self.bz_ = self.get_parameter("bz").get_parameter_value().double_value

        self.confirmed_ = False

    def on_actual(self, msg):
        tol = 1e-3
        clamped = (abs(msg.bx - self.bx_) > tol or
                   abs(msg.by - self.by_) > tol or
                   abs(msg.bz - self.bz_) > tol)
        if clamped:
            self.get_logger().warn(
                "TRIPPED: requested (x=%.3f, y=%.3f, z=%.3f) mT -> "
                "actual (x=%.3f, y=%.3f, z=%.3f) mT" %
                (self.bx_, self.by_, self.bz_, msg.bx, msg.by, msg.bz))
        else:
            self.get_logger().info(
                "Confirmed delivered: x=%.3f, y=%.3f, z=%.3f mT" %
                (msg.bx, msg.by, msg.bz))
        self.confirmed_ = True

    def publish_field(self):
        while self.publisher_.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        mag = MagField()
        mag.bx = self.bx_
        mag.by = self.by_
        mag.bz = self.bz_
        mag.header.stamp = self.get_clock().now().to_msg()
        mag.header.frame_id = "coil_frame"
        self.publisher_.publish(mag)
        self.get_logger().info("Publishing field: x: %f, y: %f, z: %f" %
                                (self.bx_, self.by_, self.bz_))

        timeout = 2.0
        elapsed = 0.0
        while not self.confirmed_ and elapsed < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed += 0.1
        if not self.confirmed_:
            self.get_logger().error(
                "No confirmation received from field_node within %.1fs — "
                "cannot verify what was actually commanded." % timeout)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    minimal_publisher.publish_field()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
