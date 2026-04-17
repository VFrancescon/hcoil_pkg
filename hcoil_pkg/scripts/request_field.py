#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from hcoil_interfaces.msg import MagField
from geometry_msgs.msg import Point
from magnetic_tentacle_interfaces.srv import ComputeMagneticField


class MinimalClient(Node):

    def __init__(self):
        super().__init__('field_request_client')

        self.client = self.create_client(ComputeMagneticField, '/compute_magnetic_field')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service ComputeField to become available...')
        self.get_logger().info("Found service. Requesting field now")
        self.req = ComputeMagneticField.Request()

    def send_request(self) -> ComputeMagneticField.Response:
        point = Point()
        point.x, point.y, point.z = (0.0, 0.0, 0.0)
        self.req.points = [Point()]
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()

    res = ComputeMagneticField.Response()

    response = minimal_client.send_request()
    bx, by, bz = response.fields[0].vector.x, response.fields[0].vector.y, response.fields[0].vector.z 
    print(f"Magnetic field components: bx={bx}, by={by}, bz={bz}")


    # Destroy the node explicitly
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
