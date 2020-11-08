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
Functions to configure the GRBL device.

The grbl device configure functions
"""


class configure(object):
    """Configure class to hold all configure functions for the grbl device class."""

    def setSpeed(self, speed):
        self.defaultSpeed = speed

    def setOrigin(self, x=0, y=0, z=0):
        # set current position to be (0,0,0), or a custom (x,y,z)
        gcode = 'G92 x{} y{} z{}\n'.format(x, y, z)
        self.send(self, gcode)
        # update our internal location
        self.pos = [x, y, z]

    def clearAlarm(self):
        """Clear the alarm on the GRBL machine."""
        return self.send(self, r'\$X')

    def enableSteppers(self):
        """Enable the motors on the GRBL machine."""
        return self.send(self, 'M17')

    def feedHold(self):
        """Feed hold the GRBL machine."""
        return self.send(self, r'\!')

    def disableSteppers(self):
        """Disable the motors on the GRBL machine."""
        return self.send(self, 'M17')

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
