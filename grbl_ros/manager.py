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

node_name = 'grbl_manager'


class grbl_manager(Node):

    def __init__(self):
        super().__init__(node_name)

        self.get_logger().info('Initializing GRBL Device Manager')
        self.cli = self.create_client(CreateDevice, 'grbl_services/create_grbl_device')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        self.req = CreateDevice.Request()
        # self.srv_delete_ = self.create_service(String, 'shutdown_grbl_device', shutdown_callback)

    def send_request(self):
        self.get_logger().warn('creating grbl device:')
        self.get_logger().warn('    machine id: ' + 'cnc_111')
        self.get_logger().warn('    port:       ' + '/tmp/ttyFAKE')


def main():
    rclpy.init()
    manager_node = grbl_manager()

    # send some service requests
    manager_node.send_request()

    while rclpy.ok():
        rclpy.spin_once(manager_node)
        if manager_node.future.done():
            response = manager_node.future.result()
        manager_node.get_logger().info(
            'Result of create_device: ' + response.success)
        break

    manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
