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
from threading import Event
import time

from geometry_msgs.msg import Pose

from grbl_msgs.action import SendGcodeCmd, SendGcodeFile
from grbl_msgs.msg import State
from grbl_msgs.srv import Stop
from grbl_ros import grbl

import rclpy
from rclpy.action import ActionClient, ActionServer

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tf2_ros.transform_broadcaster import TransformBroadcaster


class grbl_node(Node):
    """
    A ROS2 node representing a single GRBL device.

    This nodes main function is to publish the real-time pose of the GRBL device as
    a ROS2 transform (tf).  Additionally it enables the ROS2 user to send GCODE commands
    and files to the GRBL device and monitor its status.

    """

    def __init__(self):
        # TODO(evanflynn): init node with machine_id param input or arg
        super().__init__('grbl_device')

        self.get_logger().info('Declaring ROS parameters')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('machine_id', 'cnc_001'),
                ('port', '/dev/ttyUSB0'),
                ('baudrate', 115200),
                ('acceleration', 50),  # mm / min^2
                ('x_max', 300),  # mm
                ('y_max', 200),  # mm
                ('z_max', 150),  # mm
                ('default_v', 100),  # mm / min
                ('x_max_v', 150),  # mm / min
                ('y_max_v', 150),  # mm / min
                ('z_max_v', 150),  # mm / min
                ('x_steps', 100),  # mm
                ('y_steps', 100),  # mm
                ('z_steps', 100),  # mm
            ])

        self.machine_id = self.get_parameter('machine_id').get_parameter_value().string_value
        self.get_logger().info('Initializing Publishers & Subscribers')
        # Initialize Publishers
        self.pub_tf_ = TransformBroadcaster(self)
        self.pub_mpos_ = self.create_publisher(Pose, self.machine_id + '/machine_position', 5)
        self.pub_wpos_ = self.create_publisher(Pose, self.machine_id + '/work_position', 5)
        self.pub_state_ = self.create_publisher(State, self.machine_id + '/state', 5)
        # Initialize Services
        self.srv_stop_ = self.create_service(
            Stop, self.machine_id + '/stop', self.stopCallback)
        # Initialize Actions
        self.action_done_event = Event()
        self.callback_group = ReentrantCallbackGroup()
        self.action_send_gcode_ = ActionServer(
                self,
                SendGcodeCmd,
                self.machine_id + '/send_gcode_cmd',
                self.gcodeCallback)
        self.action_send_gcode_file_ = ActionServer(
                self,
                SendGcodeFile,
                self.machine_id + '/send_gcode_file',
                self.streamCallback)
        self.action_client_send_gcode_ = ActionClient(
                self,
                SendGcodeCmd,
                self.machine_id + '/send_gcode_cmd', callback_group=self.callback_group)

        self.action_done_event = Event()

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

        self.get_logger().warn('  machine_id: ' + str(self.machine_id))
        self.get_logger().warn('  port:       ' + str(port.get_parameter_value().string_value))
        self.get_logger().warn('  baudrate:   ' + str(baud.get_parameter_value().integer_value))

        self.get_logger().info('Initializing GRBL Device')
        self.machine = grbl(self)
        self.get_logger().info('Starting up GRBL Device...')
        self.machine.startup(self.machine_id,
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
            self.get_logger().warn(
                '[ TIP ] Change the serial port and machine ID parameters')
            self.get_logger().warn(
                '[ TIP ] in the `grbl_ros/config` yaml file and use it at run-time:')
            self.get_logger().warn(
                '[ TIP ]   ros2 run grbl_ros grbl_node '
                '--ros-args --params-file <path/to/config>.yaml')
            # TODO(evanflynn): set this to a different color so it stands out?
            self.get_logger().info('Node running in `debug` mode')
            self.get_logger().info('GRBL device operation may not function as expected')
            self.machine.mode = self.machine.MODE.DEBUG

    def poseCallback(self, request, response):
        self.machine.moveTo(request.position.x,
                            request.position.y,
                            request.position.z,
                            blockUntilComplete=True)
        return response

    def gcodeCallback(self, goal_handle):
        """
        Send GCODE ROS2 action callback.

        This is the callback called each time the ROS2 send_gcode_cmd action is called.

        """
        result = SendGcodeCmd.Result()
        status = self.machine.send(str(goal_handle.request.command))
        if(status.find('error') > -1):
            # grbl device returned error code
            # decode error
            self.decode_error(status)
            result.success = False
        elif(status.find('ok') > -1):
            # grbl device running command
            # check state
            self.machine.send(str('?'))
            if(self.machine.state.name.upper() == self.machine.STATE.RUN.name):
                # machine still running command
                # wait until machine is idle
                status_msg = SendGcodeCmd.Feedback()
                while self.machine.state.name.upper() == self.machine.STATE.RUN.name:
                    # poll status, publish position
                    time.sleep(0.01)
                    self.machine.send(str('?'))
                    status_msg.status = 'Running ' + str(goal_handle.request.command)
                    goal_handle.publish_feedback(status_msg)
            # elif(self.machine.state.name.upper() == self.machine.STATE.ALARM.name):
                # machine alarm is still active
                # self.get_logger().warn('ALARM')
            goal_handle.succeed()
            result.success = True
        else:
            # grbl device returned unknown
            self.get_logger().warn(status)
            result.success = False
        return result

    def streamCallback(self, goal_handle):
        """
        Send GCODE file ROS2 action callback.

        This is the callback called each time the ROS2 send_gcode_file action is called
        and calls the send_gcode_cmd action for each line in the given file.

        """
        result = SendGcodeFile.Result()
        # open file to read each line
        f = open(goal_handle.request.file_path, 'r')

        self.action_client_send_gcode_.wait_for_server()
        gcode_msg = SendGcodeCmd.Goal()

        file_length = len(open(goal_handle.request.file_path, 'r').read().split('\n'))
        line_num = 0

        for raw_line in f:
            self.action_done_event.clear()
            line = raw_line.strip()  # strip all EOL characters for consistency
            gcode_msg.command = line
            status_msg = SendGcodeFile.Feedback()
            # send gcode line to action server
            send_goal_future = self.action_client_send_gcode_.send_goal_async(
                    gcode_msg, feedback_callback=self.file_feedback)
            send_goal_future.add_done_callback(self.line_response_callback)
            # wait for send gcode action to be done
            self.action_done_event.wait()

            status_msg = SendGcodeFile.Feedback()

            # dont send another line until its done running
            status_msg.status = '[ ' + str(line_num) + ' / ' + str(file_length) + \
                ' ] Running ' + str(line)
            goal_handle.publish_feedback(status_msg)
            line_num += 1

        goal_handle.succeed()
        result.success = True
        return result

    def file_feedback(self, feedback):
        """
        Feedback function during the send_gcode_file ROS2 aciton.

        This feedback callback can be called during the send_gcode_file in order to provide
        feedback to the user as the ROS2 action is executed.

        """
        # self.get_logger().info('received feedback')
        return

    def line_response_callback(self, future):
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_done_event.set()

    def stopCallback(self, request, response):
        # stop steppers
        if request.data == 's':
            self.machine.disableSteppers()
            # fire steppers
        elif request.data == 'f':
            self.machine.enableSteppers()


def main(args=None):
    rclpy.init(args=args)
    node = grbl_node()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
