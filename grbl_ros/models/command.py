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
"""
Functions to command the GRBL device.

The grbl device command functions
"""
# from typing import Optional
# from pydantic import BaseModel, root_validator
# 
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import TransformStamped
# 
# from grbl_msgs.msg import State
# 
# from models.common import Limits, Coordinate
# import serial
# 
# 
# 
#     def handle_responses(self, responses, cmd):
#         # iterate over each response line
#         for line in responses:
#             if(line.find('ok') == -1):
#                 self.node.get_logger().info('[ ' + str(cmd) + ' ] ' + str(line))
#             # check if line is grbl status report
#             if(line[0] == '<'):
#                 self.parse_status(line)
# 
#             # for f in fields:
#             #    self.node.get_logger().info(str(f))
#             # followed grbl message construction docs
#             # https://github.com/gnea/grbl/wiki/Grbl-v1.1-Interface#grbl-response-messages
#             # The first (Machine State) and second (Current Position) data fields
#             # are always included in every report.
#             # handle machine state
#             state_msg = self.handle_state(fields[0])
#             # parse current position
#             self.handle_current_pose(fields[1])
#             # publish status
#             self.node.pub_state_.publish(state_msg)
# 
#     def handle_current_pose(self, pose):
#         transforms = []
#         machine_tf = TransformStamped()
#         machine_tf.header.frame_id = 'base_link'
#         machine_tf.header.stamp = self.node.get_clock().now().to_msg()
#         machine_tf.child_frame_id = self.machine_id + '_machine'
#         machine_pose = Pose()
# 
#         # parse coordinates from current position
#         coords = pose.split(':')[1].split(',')
#         x = float(coords[0]) / 1000.0
#         y = float(coords[1]) / 1000.0
#         z = float(coords[2]) / 1000.0
# 
#         machine_tf.transform.translation.x = x
#         machine_tf.transform.translation.y = y
#         machine_tf.transform.translation.z = z
# 
#         machine_pose.position.x = x
#         machine_pose.position.y = y
#         machine_pose.position.z = z
# 
#         transforms.append(machine_tf)
#         self.node.pub_mpos_.publish(machine_pose)
#         self.node.pub_tf_.sendTransform(transforms)
#         return
# 
#     def handle_state(self, state):
#         state_msg = State()
#         state_msg.header.stamp = self.node.get_clock().now().to_msg()
#         state_msg.header.frame_id = self.machine_id
#         # TODO(flynneva): should probably be a switch/case?
#         if(state.upper() == self.STATE.IDLE.name):
#             state_msg.state = self.STATE.IDLE
#             state_msg.state_name = self.STATE.IDLE.name
#             self.state = self.STATE.IDLE
#         elif(state.upper() == self.STATE.RUN.name):
#             state_msg.state = self.STATE.RUN
#             state_msg.state_name = self.STATE.RUN.name
#             self.state = self.STATE.RUN
#         elif(state.upper() == self.STATE.ALARM.name):
#             state_msg.state = self.STATE.ALARM
#             state_msg.state_name = self.STATE.ALARM.name
#             self.state = self.STATE.ALARM
#         elif(state.upper() == self.STATE.JOG.name):
#             state_msg.state = self.STATE.JOG
#             state_msg.state_name = self.STATE.JOG.name
#             self.state = self.STATE.JOG
#         elif(state.upper() == self.STATE.HOLD.name):
#             state_msg.state = self.STATE.HOLD
#             state_msg.state_name = self.STATE.HOLD.name
#             self.state = self.STATE.HOLD
#         elif(state.upper() == self.STATE.DOOR.name):
#             state_msg.state = self.STATE.DOOR
#             state_msg.state_name = self.STATE.DOOR.name
#             self.state = self.STATE.DOOR
#         elif(state.upper() == self.STATE.CHECK.name):
#             state_msg.state = self.STATE.CHECK
#             state_msg.state_name = self.STATE.CHECK.name
#             self.state = self.STATE.CHECK
#         elif(state.upper() == self.STATE.HOME.name):
#             state_msg.state = self.STATE.HOME
#             state_msg.state_name = self.STATE.HOME.name
#             self.state = self.STATE.HOME
#         elif(state.upper() == self.STATE.SLEEP.name):
#             state_msg.state = self.STATE.SLEEP
#             state_msg.state_name = self.STATE.SLEEP.name
#             self.state = self.STATE.SLEEP
#         return state_msg
# 
#     def stream(self, fpath):
#         """
#         Send an entire file of GCODE commands to the GRBL machine.
# 
#         Args:
#         ----
#             fpath (str): filepath to GCODE (.nc, .gcode) file to send
# 
#         Return:
#         ------
#             str: status of sending the file
# 
#         """
#         f = open(fpath, 'r')
#         for raw_line in f:
#             line = raw_line.strip()  # strip all EOL characters for consistency
#             status = self.send(line)
#             if(self.mode == self.MODE.DEBUG):
#                 print('    ' + status)
#         return 'ok'
