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
        self.s = None    # serial port object
        self.abs_move = None     # GRBL has 2 movement modes: relative and absolute
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
        self.origin = [0.0, 0.0, 0.0]        # minimum coordinates  [X, Y, Z]
        self.limits = [0.0, 0.0, 0.0]        # maximum coordinates  [X, Y, Z]

    # Import Methods
    from ._logging import getStatus, decodeStatus, STATUS
    from ._configure import getPose, setSpeed, setOrigin, ensureMovementMode, blockUntilIdle, \
        clearAlarm, enableSteppers, disableSteppers
    from ._control import home, moveTo, moveRel, moveToOrigin
