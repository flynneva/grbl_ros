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
from geometry_msgs.msg import TransformStamped

from grbl_msgs.action import SendGcodeCmd
from grbl_msgs.msg import State
from grbl_msgs.srv import Stop
from grbl_ros import grbl

import rclpy
from rclpy.action import ActionServer

from rclpy.node import Node

from std_msgs.msg import String
from tf2_ros.transform_broadcaster import TransformBroadcaster


class grbl_node(Node):

    def __init__(self):
        # TODO(evanflynn): init node with machine_id param input or arg
        super().__init__('grbl_device')

        self.get_logger().info('Declaring ROS parameters')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('machine_id', None),
                ('port', None),
                ('baudrate', None),
                ('acceleration', None),  # mm / min^2
                ('x_max', None),  # mm
                ('y_max', None),  # mm
                ('z_max', None),  # mm
                ('default_v', None),  # mm / min
                ('x_max_v', None),  # mm / min
                ('y_max_v', None),  # mm / min
                ('z_max_v', None),  # mm / min
                ('x_steps', None),  # mm
                ('y_steps', None),  # mm
                ('z_steps', None),  # mm
            ])

        self.name = self.get_parameter('machine_id').get_parameter_value().string_value
        self.get_logger().info('Initializing Publishers & Subscribers')
        # Initialize Publishers
        self.pub_tf_ = TransformBroadcaster(self)
        self.pub_mpos_ = self.create_publisher(Pose, self.name + '/machine_position', 5)
        self.pub_wpos_ = self.create_publisher(Pose, self.name + '/work_position', 5)
        self.pub_state_ = self.create_publisher(State, self.name + '/state', 5)
        # Initialize Services
        self.srv_stop_ = self.create_service(
            Stop, self.name + '/stop', self.stopCallback)
        # Initialize Actions
        self.action_send_gcode_ = ActionServer(
                self,
                SendGcodeCmd,
                'send_gcode_cmd',
                self.gcodeCallback)

        self.get_logger().info('Getting ROS parameters')
        port = self.get_parameter('port')
        baud = self.get_parameter('baudrate')
        acc = self.get_parameter('acceleration')    # axis acceleration (mm/s^2)
        max_x = self.get_parameter('x_max')           # workable travel (mm)
        max_y = self.get_parameter('y_max')           # workable travel (mm)
        max_z = self.get_parameter('x_max')           # workable travel (mm)
        default_speed = self.get_parameter('default_v')   # mm/min
        speed_x = self.get_parameter('x_max_v')     # mm/min
        speed_y = self.get_parameter('y_max_v')     # mm/min
        speed_z = self.get_parameter('z_max_v')     # mm/min
        steps_x = self.get_parameter('x_steps')      # axis steps per mm
        steps_y = self.get_parameter('y_steps')      # axis steps per mm
        steps_z = self.get_parameter('z_steps')      # axis steps per mm

        self.get_logger().warn('  machine_id: ' + str(self.name))
        self.get_logger().warn('  port:       ' + str(port.get_parameter_value().string_value))
        self.get_logger().warn('  baudrate:   ' + str(baud.get_parameter_value().integer_value))

        self.get_logger().info('Initializing GRBL Device')
        self.machine = grbl(self)
        self.get_logger().info('Starting up GRBL Device...')
        self.machine.startup(self.name,
                             port.get_parameter_value().string_value,
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
        if(self.machine.s):
            self.machine.getStatus()
            self.machine.getSettings()
        else:
            self.get_logger().warn('Could not detect GRBL device '
                                   'on serial port ' + self.machine.port)
            self.get_logger().warn('Are you sure the GRBL device '
                                   'is connected and powered on?')
            # TODO(evanflynn): set this to a different color so it stands out?
            self.get_logger().info('Node running in `debug` mode')
            self.get_logger().info('GRBL device operation may not function as expected')
            self.machine.mode = self.grbl_obj.MODE.DEBUG

    def poseCallback(self, request, response):
        self.machine.moveTo(request.position.x,
                            request.position.y,
                            request.position.z,
                            blockUntilComplete=True)
        return response

    def gcodeCallback(self, goal_handle):
        result = SendGcodeCmd.Result()
        status = self.machine.send(str(goal_handle.request.command))
        if(status.find('error') > -1):
            # grbl device returned error code
            self.get_logger().warn(status)
            result.success = False
        elif(status.find('ok') > -1):
            # grbl device running command
            # check state
            self.machine.send(str('?'))
            if(self.machine.state.name.upper() == self.machine.STATE.RUN.name):
                # machine still running command
                # wait until machine is idle
                while self.machine.state.name.upper() == self.machine.STATE.RUN.name:
                    # poll status, publish position
                    time.sleep(0.2)
                    self.machine.send(str('?'))
            #elif(self.machine.state.name.upper() == self.machine.STATE.ALARM.name):
                # machine alarm is still active
                #self.get_logger().warn('ALARM')
            goal_handle.succeed()
            result.success = True
        else:
            # grbl device returned unknown
            self.get_logger().warn(status)
            result.success = False
        return result

    def stopCallback(self, request, response):
        # stop steppers
        if request.data == 's':
            self.machine.disableSteppers()
            # fire steppers
        elif request.data == 'f':
            self.machine.enableSteppers()

    def streamCallback(self, request, response):
        self.get_logger().info('Sending GCODE file: ')
        self.get_logger().info('  ' + request.filepath)
        # stream gcode file to grbl device
        status = self.machine.stream(request.filepath)
        # TODO(evanflynn): have stream method return something useful
        self.get_logger().info('GCODE file status: ' + str(status))

    def pub_status_thread(self):
        transforms = []
        msg = String()
        # getStatus has to wait for response over serial
        status = self.machine.getStatus()
        for line in status.split():
            msg.data = line.rstrip('\r\n')
            self.pub_status_.publish(msg)
            print(line)
            if(line.find('<') > -1 and line.find('MPos') > -1):
                # self.get_logger().info(line)
                m_tf = TransformStamped()
                m_tf.header.frame_id = 'base_link'
                m_tf.header.stamp = self.get_clock().now().to_msg()
                m_tf.child_frame_id = self.name + '_machine'

                w_tf = TransformStamped()
                w_tf.header.frame_id = 'base_link'
                w_tf.header.stamp = self.get_clock().now().to_msg()
                w_tf.child_frame_id = self.name + '_workpiece'

                coord = line[(line.find('MPos')+5):].split(',')

                m_x = float(coord[0]) / 1000.0
                m_y = float(coord[1]) / 1000.0
                m_z = float(coord[2].split('|')[0]) / 1000.0

                m_pose = Pose()

                m_pose.position.x = m_x
                m_pose.position.y = m_y
                m_pose.position.z = m_z

                m_tf.transform.translation.x = m_x
                m_tf.transform.translation.y = m_y
                m_tf.transform.translation.z = m_z

                self.pub_mpos_.publish(m_pose)

                transforms.append(m_tf)

                if(line.find('WPos') > -1):
                    w_x = float(coord[3].split(':')[1]) / 1000.0
                    w_y = float(coord[4]) / 1000.0
                    w_z = float(coord[5][:-1]) / 1000.0

                    w_pose = Pose()
                    w_pose.position.x = w_x
                    w_pose.position.y = w_y
                    w_pose.position.z = w_z

                    w_tf.transform.translation.x = w_x
                    w_tf.transform.translation.y = w_y
                    w_tf.transform.translation.z = w_z

                    self.pub_wpos_.publish(w_pose)
                    transforms.append(w_tf)

                self.pub_tf_.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = grbl_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
