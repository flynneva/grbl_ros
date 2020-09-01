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


def getPose(self):
    pose = Pose()
    pose.position.x = float(self.pos[0])
    pose.position.y = float(self.pos[1])
    pose.position.z = float(self.pos[2])
    # this parameters are set to 0
    # the cnc its a XYZ 3 DOF mechanism and doesnt need it
    # TODO(evanflynn): could be useful for higher DOF machines?
    pose.orientation.x = float(self.angular[0])
    pose.orientation.y = float(self.angular[1])
    pose.orientation.z = float(self.angular[2])
    pose.orientation.w = float(self.angular[3])
    return pose


def setSpeed(self, speed):
    self.defaultSpeed = speed


def setOrigin(self, x=0, y=0, z=0):
    # set current position to be (0,0,0), or a custom (x,y,z)
    gcode = 'G92 x{} y{} z{}\n'.format(x, y, z)
    self.s.write(str.encode(gcode))
    self.s.readline()
    # update our internal location
    self.pos = [x, y, z]


def clearAlarm(self):
    self.s.write(b'$X\n')
    return self.s.readline().decode('utf-8').strip()


def enableSteppers(self):
    self.s.write(b'M17\n')
    return self.s.readline().decode('utf-8').strip()


def disableSteppers(self):
    self.s.write(b'M18\n')
    return self.s.readline().decode('utf-8').strip()


def ensureMovementMode(self, absoluteMode=True):
    # GRBL has two movement modes
    # if necessary this function tells GRBL to switch modes
    if self.abs_move == absoluteMode:
        return
    self.abs_move = absoluteMode
    if absoluteMode:
        self.s.write(b'G90\r\n')  # absolute movement mode
    else:
        self.s.write(b'G91\r\n')  # relative movement mode
        self.s.readline()


def blockUntilIdle(self):
    # polls until GRBL indicates it is done with the last command
    pollcount = 0
    while True:
        self.s.write(b'?\r\n')
        status = self.s.readline()
        if status.startswith('<Idle'):
            break
        # not used
        pollcount += 1
        # poll every 10 ms
        time.sleep(.01)
