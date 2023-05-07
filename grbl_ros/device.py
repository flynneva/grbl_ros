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

import rclpy
from rclpy.action import ActionClient, ActionServer

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tf2_ros.transform_broadcaster import TransformBroadcaster

from grbl_ros.models.common import VectorXYZ
from grbl_ros.models.connectors.uart import UART
from grbl_ros.models.grbl import GrblConfig, GrblDevice


class grbl_node(Node):
    """
    A ROS2 node representing a single GRBL device.

    This nodes main function is to publish the real-time pose of the GRBL device as
    a ROS2 transform (tf).  Additionally it enables the ROS2 user to send GCODE commands
    and files to the GRBL device and monitor its status.

    """

    def __init__(self, node_name: str):
        # TODO(evanflynn): init node with machine_id param input or arg
        super().__init__(node_name)

        # Initialize the GRBL device objects
        self.device = GrblDevice()

        self.get_logger().info("Declaring ROS parameters")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("machine_id", "cnc_001"),
                ("port", "/dev/ttyUSB0"),
                ("baudrate", 115200),
                ("timeout", 5),   # seconds
                ("acceleration", 50),  # mm / min^2
                ("x_max", 300),  # mm
                ("y_max", 200),  # mm
                ("z_max", 150),  # mm
                ("default_v", 100),  # mm / min
                ("x_max_v", 150),  # mm / min
                ("y_max_v", 150),  # mm / min
                ("z_max_v", 150),  # mm / min
                ("x_steps", 100),  # mm
                ("y_steps", 100),  # mm
                ("z_steps", 100),  # mm
            ])

        # Update the parameters if any are available
        self.get_params()

        self.get_logger().info('Initializing Publishers & Subscribers')
        # Initialize Publishers
        self.pub_tf_ = TransformBroadcaster(self)
        self.pub_mpos_ = self.create_publisher(Pose, self.device.id + '/machine_position', 5)
        self.pub_wpos_ = self.create_publisher(Pose, self.device.id + '/work_position', 5)
        self.pub_state_ = self.create_publisher(State, self.device.id + '/state', 5)
        # Initialize Services
        self.srv_stop_ = self.create_service(
            Stop, self.device.id + '/stop', self.stopCallback)
        # Initialize Actions
        self.action_done_event = Event()
        self.callback_group = ReentrantCallbackGroup()
        self.action_send_gcode_ = ActionServer(
                self,
                SendGcodeCmd,
                self.device.id + '/send_gcode_cmd',
                self.gcodeCallback)
        self.action_send_gcode_file_ = ActionServer(
                self,
                SendGcodeFile,
                self.device.id + '/send_gcode_file',
                self.streamCallback)
        self.action_client_send_gcode_ = ActionClient(
                self,
                SendGcodeCmd,
                self.device.id + '/send_gcode_cmd', callback_group=self.callback_group)

        self.action_done_event = Event()

        self.get_logger().warn('  machine_id: ' + str(self.device.id))
        self.get_logger().warn('  port:       ' + str(self.device.connection.port))
        self.get_logger().warn('  baudrate:   ' + str(self.device.connection.baudrate))

        self.get_logger().info('Starting up GRBL Device...')
        self.device.get_status()
        self.device.get_settings()

    def get_params(self):
        """Get the GRBL parameters and update the members with them."""
        self.get_logger().info('Getting ROS parameters')
        self.device.id = self.get_str_param('machine_id')

        connection = UART(
            port = self.get_str_param("port"),
            baudrate = self.get_int_param("baudrate"),
            timeout = self.get_int_param("timeout"),
        )
        self.device.connection = connection

        config = GrblConfig(
            acceleration = self.get_int_param("acceleration"),
            max_travel = VectorXYZ(
              x = self.get_int_param("x_max"),
              y = self.get_int_param("y_max"),
              z = self.get_int_param("z_max"),
            ),
            default_speed = self.get_int_param("default_v"),
            max_speed = VectorXYZ(
              x = self.get_int_param("x_max_v"),
              y = self.get_int_param("y_max_v"),
              z = self.get_int_param("z_max_v"),
            ),
            steps_per_mm = VectorXYZ(
              x = self.get_int_param("x_steps"),
              y = self.get_int_param("y_steps"),
              z = self.get_int_param("z_steps"),
            )
        )
        self.device.config = config

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
    
    def get_int_param(self, param_name: str) -> int:
        return self.get_parameter(param_name).get_parameter_value().integer_value

    def get_str_param(self, param_name: str) -> str:
        return self.get_parameter(param_name).get_parameter_value().string_value

def main(args=None):
    rclpy.init(args=args)
    # TODO(flynneva): make this a CLI arg
    node = grbl_node("grbl_device")
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
