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
Functions to handle logging for the GRBL device.

The grbl device logging functions
"""

from enum import IntEnum

from geometry_msgs.msg import Pose


class logging(object):
    """Logging class to hold all logging functions for the grbl device class."""

    def getStatus(self):
        """Get the current status of the GRBL device."""
        # TODO(evanflynn): status should be ROS msg?
        if(self.mode == self.MODE.NORMAL):
            # ? returns the active GRBL state & current machine and work positions
            return self.send('?')
        elif(self.mode == self.MODE.DEBUG):
            return 'DEBUG GRBL device is happy!'
        else:
            return 'UNDEFINED GRBL MODE'

    def getSettings(self):
        """Get the current settings of the GRBL device."""
        # TODO(evanflynn): status should be ROS msg?
        if(self.mode == self.MODE.NORMAL):
            # ? returns the active GRBL state & current machine and work positions
            return self.send('$$')
        elif(self.mode == self.MODE.DEBUG):
            return 'DEBUG GRBL device is happy!'
        else:
            return 'UNDEFINED GRBL MODE'

    def getPose(self):
        """Get the last Pose recorded of the GRBL device."""
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

    def decodeStatus(self, status):
        if('error' in status):
            err = status.split(':', 1)[1]
            if(type(err) == int()):
                return self.STATUS(int(status.split(':', 1)[1])).name
            else:
                return err
        else:
            return status

    class MODE(IntEnum):
        """Operation modes of the GRBL device, helpful for debugging."""

        NORMAL = 0
        DEBUG = 1

    class STATE(IntEnum):
        """Operation states of the GRBL device."""

        IDLE = 0
        RUN = 1
        HOLD = 2
        JOG = 3
        ALARM = 4
        DOOR = 5
        CHECK = 6
        HOME = 7
        SLEEP = 8

    class STATUS(IntEnum):
        """
        Official GRBL error list.

        http://domoticx.com/cnc-machine-grbl-error-list/

        """

        NO_ERROR = 0
        MISSING_LETTER = 1  # G-code words consist of a letter and a value. Letter not found
        INVALID_NUMERIC = 2  # Numeric value format is not valid or missing an expected value
        COMMAND_NOT_SUPPORTED = 3  # Grbl '$' system command was not recognized or supported
        NEGATIVE_POSITION = 4  # Negtaive value received for an expected positive value
        DISABLED_HOMING = 5  # Homing cycle is not enabled via settings
        MIN_STEP_TOO_SMALL = 6  # must be larger than 3usec
        EEPROM_READ_FAIL = 7  # EEPROM read failed. Reset and restored to default values
        GRBL_NOT_IDLE = 8  # Grbl '$' cmd cannot be used unless Grbl is IDLE
        LOCKED_OUT = 9  # G-code locked out during alarm or jog state
        HOMING_NOT_ENABLED = 10  # Soft limits cannot be enabled without homing also enabled
        MAX_CHARS_PER_LINE = 11  #
