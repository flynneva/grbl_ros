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
import time

from geometry_msgs.msg import Pose

from grbl_ros import grbl

import rclpy

from rclpy.node import Node

from std_msgs.msg import String

grbl_node_name = 'cnc_001'


class grbl_node(Node):

    def __init__(self):
        super().__init__(grbl_node_name)
        self.pub_pos_ = self.create_publisher(Pose, grbl_node_name + '/position', 10)
        self.pub_status_ = self.create_publisher(String, grbl_node_name + '/status', 10)
        self.sub_cmd_ = self.create_subscription(
            String, grbl_node_name + '/send_gcode', self.gcodeCallback, 10)
        self.sub_pose_ = self.create_subscription(
            Pose, grbl_node_name + '/set_pose', self.poseCallback, 10)
        self.sub_stop_ = self.create_subscription(
            String, grbl_node_name + '/stop', self.stopCallback, 10)
        self.sub_stop_  # prevent unused variable warning

        # declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('acceleration', 1000)
        self.declare_parameter('x_max', 50)
        self.declare_parameter('y_max', 50)
        self.declare_parameter('z_max', 50)
        self.declare_parameter('default_speed', 100)
        self.declare_parameter('x_max_speed', 100)
        self.declare_parameter('y_max_speed', 100)
        self.declare_parameter('z_max_speed', 100)
        self.declare_parameter('x_steps_mm', 100)
        self.declare_parameter('y_steps_mm', 100)
        self.declare_parameter('z_steps_mm', 100)

        port = self.get_parameter('port')
        baud = self.get_parameter('baudrate')
        acc = self.get_parameter('acceleration')    # axis acceleration (mm/s^2)
        max_x = self.get_parameter('x_max')           # workable travel (mm)
        max_y = self.get_parameter('y_max')           # workable travel (mm)
        max_z = self.get_parameter('x_max')           # workable travel (mm)
        default_speed = self.get_parameter('default_speed')   # mm/min
        speed_x = self.get_parameter('x_max_speed')     # mm/min
        speed_y = self.get_parameter('y_max_speed')     # mm/min
        speed_z = self.get_parameter('z_max_speed')     # mm/min
        steps_x = self.get_parameter('x_steps_mm')      # axis steps per mm
        steps_y = self.get_parameter('y_steps_mm')      # axis steps per mm
        steps_z = self.get_parameter('z_steps_mm')      # axis steps per mm

        self.grbl_obj = grbl()
        self.grbl_obj.startup(port.get_parameter_value().string_value,
                              baud.get_parameter_value().integer_value,
                              acc.get_parameter_value().integer_value,
                              max_x.get_parameter_value().integer_value,
                              max_y.get_parameter_value().integer_value,
                              max_z.get_parameter_value().integer_value,
                              default_speed.get_parameter_value().integer_value,
                              speed_x.get_parameter_value().integer_value,
                              speed_y.get_parameter_value().integer_value,
                              speed_z.get_parameter_value().integer_value,
                              steps_x.get_parameter_value().integer_value,
                              steps_y.get_parameter_value().integer_value,
                              steps_z.get_parameter_value().integer_value)
        self.wake()
        self.refreshStatus()
        self.refreshPosition()

    def wake(self):
        self.grbl_obj.s.write(b'\r\n\r\n')
        time.sleep(2)   # wait for grbl to initialize
        self.grbl_obj.s.flushInput()

    def refreshStatus(self):
        status = self.grbl_obj.getStatus()
        for stat in status:
            self.get_logger().info(stat)
            if ('error' in stat):
                # TODO(evanflynn): temp code to clear alarm error, should be user decision
                self.get_logger().warn(self.grbl_obj.clearAlarm())
            ros_status = String()
            ros_status.data = stat
            self.pub_status_.publish(ros_status)

    def refreshPosition(self):
        pose = self.grbl_obj.getPose()
        self.pub_pos_.publish(pose)

    def poseCallback(self, msg):
        self.grbl_obj.moveTo(msg.position.x,
                             msg.position.y,
                             msg.position.z,
                             blockUntilComplete=True)
        self.refreshStatus()
        self.refreshPosition()

    def gcodeCallback(self, msg):
        self.get_logger().info('Sending GCODE command: ' + msg.data)
        status = self.grbl_obj.gcode(msg.data)
        # warn user of grbl response
        self.get_logger().warn(status)
        self.refreshStatus()
        self.refreshPosition()

    def stopCallback(self, msg):
        # stop steppers
        if msg.data == 's':
            self.grbl_obj.disableSteppers()
            # fire steppers
        elif msg.data == 'f':
            self.grbl_obj.enableSteppers()


def main():
    rclpy.init()
    interface = grbl()
    rclpy.spin(interface)


if __name__ == '__main__':
    main()
