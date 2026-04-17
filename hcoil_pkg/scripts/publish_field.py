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

        field_desc = ParameterDescriptor(
            description="Field in mT.",
            )
        self.declare_parameter("bx", 0.0, field_desc)
        self.declare_parameter("by", 0.0, field_desc)
        self.declare_parameter("bz", 0.0, field_desc)


        self.bx_ = self.get_parameter("bx").get_parameter_value().double_value
        self.by_ = self.get_parameter("by").get_parameter_value().double_value
        self.bz_ = self.get_parameter("bz").get_parameter_value().double_value

    def publish_field(self):
        mag = MagField()
        mag.bx = self.bx_
        mag.by = self.by_
        mag.bz = self.bz_
        mag.header.frame_id = "coil_frame"
        self.publisher_.publish(mag)
        self.get_logger().info("Publishing field: x: %f, y: %f, z: %f" % (self.bx_, self.by_, self.bz_))

def main(args=None):
    rclpy.init(args=args)
    # print("Usage:")
    # print("  [--ros-args -p bx/by/bz]:=float")
    # print()
    # print("Options:")
    # print("  Set bx, by, bz, or any combination thereof.")
    # print()
    # print("Description:")
    # print("  This script publishes a magnetic field vector (bx, by, bz) to the 'magfield' topic.")
    # print("  Ensure that the given argument is a float (e.g., 10.0 instead of 10).")
    minimal_publisher = MinimalPublisher()

    # Publish the field once
    minimal_publisher.publish_field()

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
