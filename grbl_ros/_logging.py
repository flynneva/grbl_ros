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
from enum import Enum

import time


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
        return STATUS(int(status.split(':', 1)[1])).name
    return status


class STATUS(Enum):
    # enum class for grbl error list
    # http://domoticx.com/cnc-machine-grbl-error-list/
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
