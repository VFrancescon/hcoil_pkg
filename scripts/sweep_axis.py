#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node

from hcoil_interfaces.msg import MagField
import rclpy.parameter
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('field_publisher')
        self.publisher_ = self.create_publisher(MagField, 'magfield', 10)

        field_desc = ParameterDescriptor(
            description="Field in mT.",
            )
        self.declare_parameter("abs", 0.0, field_desc)
        self.declare_parameter("axis", "x")
        # self.declare_parameter("abs_y", 0.0, field_desc)
        # self.declare_parameter("abs_z", 0.0, field_desc)


        self.abs_field = abs(self.get_parameter("abs").get_parameter_value().double_value)
        self.axis = self.get_parameter("axis").get_parameter_value().string_value
        # self.abs_y_ = self.get_parameter("abs_y").get_parameter_value().double_value
        # self.abs_z_ = self.get_parameter("abs_z").get_parameter_value().double_value
        loop_rate = 0.5
        self._loop_rate = self.create_rate(loop_rate, self.get_clock())
        self.get_logger().info("Starting node: abs: %f, y: %s" % (-self.abs_field, self.axis))

        if(self.abs_field > 10):
            mag = MagField()
            mag.bx = -self.abs_field / 2 if self.axis == "x_" else 0.0
            mag.by = -self.abs_field / 2 if self.axis == "y_" else 0.0
            mag.bz = -self.abs_field / 2 if self.axis == "z_" else 0.0
            mag.header.stamp = self.get_clock().now().to_msg()
            mag.header.frame_id = "sweep_field"
            self.publisher_.publish(mag)
        
        self.num_steps = 10
        # self.field_array = np.arange(-self.abs_field, self.abs_field, self.num_steps)
        self.field_array = np.linspace(-self.abs_field, self.abs_field, self.num_steps)
        # self.rate.sleep()
        # self._loop_rate.sleep()
        time.sleep(2)
    def sweep_fields(self):
        for i in range(self.num_steps):
            
            mag = MagField()
            mag.bx = self.field_array[i] if self.axis == "x_" else 0.0
            mag.by = self.field_array[i] if self.axis == "y_" else 0.0
            mag.bz = self.field_array[i] if self.axis == "z_" else 0.0
            mag.header.stamp = self.get_clock().now().to_msg()
            mag.header.frame_id = "sweep_field"
            self.get_logger().info("Publishing field: x: %f, y: %f, z: %f" % (mag.bx, mag.by, mag.bz))
            self.publisher_.publish(mag)
            # self._loop_rate.sleep()
            time.sleep(2)


    # def publish_field(self):
    #     mag = MagField()
    #     mag.bx = self.bx_
    #     mag.by = self.by_
    #     mag.bz = self.bz_
    #     mag.header.frame_id = "coil_frame"
    #     self.publisher_.publish(mag)
    #     self.get_logger().info("Publishing field: x: %f, y: %f, z: %f" % (self.bx_, self.by_, self.bz_))

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    # Publish the field once
    minimal_publisher.sweep_fields()

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
