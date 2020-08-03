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
from grbl_ros.grbl_errors import GRBLSTATUS

import serial

#   This code was ported over to ROS2 from picatostas:
#   https://github.com/picatostas/cnc_interface
#   Special thanks to his work and the work before him


#   Class valid for interfacing XYZ cartesian grbl device
#   Future implementations might include control for other
#   GCODE compatible systems


class grbl:

    def __init__(self):
        """
        Summary line.

        Initialize the grbl class with default parameters

        """
        # Default parameter values set in startup
        self.s = None	 # serial port object
        self.abs_move = None	 # GRBL has 2 movement modes: relative and absolute
        self.baudrate = 0
        self.port = ''
        self.acceleration = 0
        self.x_max = 0
        self.y_max = 0
        self.z_max = 0
        self.defaultSpeed = 0
        self.x_max_speed = 0
        self.y_max_speed = 0
        self.z_max_speed = 0
        self.x_steps_mm = 0           # number of steps per centimeter
        self.y_steps_mm = 0           # number of steps per centimeter
        self.z_steps_mm = 0           # number of steps per centimeter
        self.idle = True           # machine is idle
        self.pos = [0.0, 0.0, 0.0]       # current position     [X, Y, Z]
        self.angular = [0.0, 0.0, 0.0, 0.0]  # quaterion  [X, Y, Z, W]
        self.origin = [0.0, 0.0, 0.0]	     # minimum coordinates  [X, Y, Z]
        self.limits = [0.0, 0.0, 0.0]	     # maximum coordinates  [X, Y, Z]

    def startup(self, port, baud, acc, maxx, maxy, maxz,
                spdf, spdx, spdy, spdz, stepsx, stepsy, stepsz):
        """
        Summary line.

        Startup the GRBL machine with the specified parameters

        Args:
        ----
          self (obj): the grbl object
          port (str): the serial port that connects to the grbl device
          baud (int): the baud rate to use for serial communication with the grbl device
          acc (int): the grbl device axis acceleration (mm/s^2)
          maxx (int): workable travel of the grbl device for its x axis (mm)
          maxy (int): workable travel of the grbl device for its y axis (mm)
          maxz (int): workable travel of the grbl device for tts z axis (mm)
          spdf (int): default speed of the grbl device (mm/min)
          spdx (int): maximum speed for x axis (mm/min)
          spdy (int): maximum speed for y axis (mm/min)
          spdz (int): maximum speed for z axis (mm/min)
          stepsx (int): number of steps per mm for x axis (steps)
          stepsy (int): number of steps per mm for y axis (steps)
          stepsz (int): number of steps per mm for z axis (steps)

        """
        # initiate all CNC parameters read from .launch file
        self.baudrate = baud
        self.port = port
        self.acceleration = acc
        self.x_max = maxx
        self.y_max = maxy
        self.z_max = maxz
        self.defaultSpeed = spdf
        self.x_max_speed = spdx
        self.y_max_speed = spdy
        self.z_max_speed = spdz
        self.x_steps_mm = stepsx
        self.y_steps_mm = stepsy
        self.z_steps_mm = stepsz
        self.limits = [self.x_max, self.y_max, self.z_max]
        # initiates the serial port
        self.s = serial.Serial(self.port, self.baudrate)

        # set movement to Absolute coordinates
        self.ensureMovementMode(True)
        # start homing procedure
        # TODO(flynneva): should this be done at startup?
        # should probably be configurable by user
        # self.home()
        # TODO(flynneva): could cause issues if user is resuming a job
        # set the current position as the origin (GRBL sometimes starts with z not 0)
        # self.setOrigin()

    def shutdown(self):
        # close the serial connection
        self.s.close()

    def gcode(self, gcode):
        # TODO(evanflynn): need to add some input checking
        self.s.write(str.encode(gcode + '\r\n'))
        return self.decodeStatus(self.s.readline().decode('utf-8').strip())

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

    def home(self):
        self.s.write(b'$H\n')
        return self.s.readline().decode('utf-8').strip()

    def clearAlarm(self):
        self.s.write(b'$X\n')
        return self.s.readline().decode('utf-8').strip()

    def enableSteppers(self):
        self.s.write(b'M17\n')
        return self.s.readline().decode('utf-8').strip()

    def disableSteppers(self):
        self.s.write(b'M18\n')
        return self.s.readline().decode('utf-8').strip()

    # TODO(flynneva): migrate to just pose msg arg to cover higher order axis machines too
    def moveTo(self, x=None, y=None, z=None, speed=None, blockUntilComplete=True):
        # move to an absolute position
        # return when movement completes
        if not self.idle:
            return
        if x is None and y is None and z is None:
            return
        if speed is None:
            speed = self.defaultSpeed
        self.ensureMovementMode(absoluteMode=True)
        gcode = 'G0'
        letters = 'XYZ'
        pos = (x, y, z)
        newpos = list(self.pos)
        # create gcode string and update position list for each argument that isn't None
        for i in range(3):
            if pos[i] is not None:
                # check against self.limits
                if pos[i] < 0 or pos[i] >= self.limits[i]:
                    # if position is outside the movement range, ignore
                    return
                gcode += ' ' + letters[i] + str(pos[i])
                newpos[i] = pos[i]
        # gcode += ' F' + str(speed)
        gcode += '\n'
        self.s.write(str.encode(gcode))
        return self.s.readline().decode('utf-8').strip()

    def moveRel(self, dx=None, dy=None, dz=None, speed=None, blockUntilComplete=True):
        # move a given distance, and return when movement completes
        # :param dx, dy, dz: distance to move
        # :param speed: units uncertain
        # :param blockUntilComplete: wait for the movement to complete or return immediately
        self.ensureMovementMode(absoluteMode=False)
        if speed is None:
            speed = self.defaultSpeed
        gcode = 'G0'
        letters = 'xyz'
        d = (dx, dy, dz)
        newpos = list(self.pos)
        # create gcode string and update position list for each argument that isn't None
        # (TODO: if successful?)
        for i in range(3):
            if d[i] is not None:
                gcode += ' ' + letters[i] + str(d[i])
                newpos[i] += d[i]
        gcode += ' f' + str(speed)
        gcode += '\r\n'
        self.s.write(str.encode(gcode))
        self.s.readline()
        # the position update should be done after reading state
        # update position if success
        # TODO check to make sure it's actually a success
        # self.pos = newpos
        if blockUntilComplete:
            self.blockUntilIdle()

    def moveToOrigin(self, speed=None):
        # move to starting position
        # return when movement completes
        if speed is None:
            speed = self.defaultSpeed
        self.moveTo(*self.origin, speed=speed)
        self.pos = list(self.origin)

    def setOrigin(self, x=0, y=0, z=0):
        # set current position to be (0,0,0), or a custom (x,y,z)
        gcode = 'G92 x{} y{} z{}\n'.format(x, y, z)
        self.s.write(str.encode(gcode))
        self.s.readline()
        # update our internal location
        self.pos = [x, y, z]

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

    def getStatus(self):
        # TODO(evanflynn): status should be ROS msg?
        self.s.write(b'?\n')
        time.sleep(0.1)
        buffer_status = []
        status = ''
        while True:
            if (self.s.inWaiting()):
                status = self.decodeStatus(self.s.readline().decode('utf-8').strip())
                buffer_status.append(status)
                status = ''
            else:
                self.s.flushInput()
                return buffer_status

    def decodeStatus(self, status):
        if('error' in status):
            return GRBLSTATUS(int(status.split(':', 1)[1])).name
        return status
