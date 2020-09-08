# Copyright 2020 Evan Flynn
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
from grbl_interfaces.srv import CreateDevice

import rclpy

from rclpy.node import Node

node_name = 'grbl_services'


class grbl_service(Node):

    def __init__(self):
        super().__init__(node_name)

        self.get_logger().info('Initializing GRBL Device Services')
        self.srv_create_ = self.create_service(
            CreateDevice, node_name + '/create_grbl_device', self.create_callback)

    def create_callback(self, request, response):
        self.get_logger().warn('creating grbl device:')
        self.get_logger().warn('    machine id: ' + request.machine_id)
        self.get_logger().warn('    port:       ' + request.port)

    def shutdown_callback(self, request, response):
        self.get_logger().info('shutting down grbl device')


def main():
    rclpy.init()
    node = grbl_service()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
