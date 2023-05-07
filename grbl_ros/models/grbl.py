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
Functions to initialize the GRBL device.

This code was ported over to ROS2 from picatostas:
https://github.com/picatostas/cnc_interface
Special thanks to his work and the work before him

The grbl device class initialized here imports the other control,
command, configure and logging files within this directory and takes
a ROS2 node as an argument in order to seperate out the ROS2 specific
code from the grbl device code.

"""
from typing import List, Optional
from pydantic import BaseModel

from grbl_ros.models.common import (
    GrblError,
    Limits,
    MovementMode,
    OperationState,
    VectorXYZ,
    Quaternion
)
from grbl_ros.models.connectors.uart import UART


class GrblConfig(BaseModel):
    acceleration: int                             # the grbl device axis acceleration (mm/s^2)
    max_travel: VectorXYZ                # workable travel of the grbl device for each axis (mm)
    default_speed: int                   # default speed of the grbl device (mm/min)
    max_speed: VectorXYZ                 # maximum speed for each axis (mm/min)
    steps_per_mm: VectorXYZ              # number of steps per mm
    limits: Limits                       #


class GrblDevice(BaseModel):
    id: str = "cnc_000"
    op_state: OperationState = OperationState.ALARM  # initalize to alarm state for safety
    movement_mode: MovementMode = MovementMode.RELATIVE
    connection: Optional[UART]                                # serial port connection
    position: VectorXYZ = VectorXYZ()                # the current position of the device [X, Y, Z]
    angular: Quaternion = Quaternion()               # quaterion [X, Y, Z, W]
    origin: VectorXYZ = VectorXYZ()                  # origin [X, Y, Z]
    config: Optional[GrblConfig]

    def home(self):
        """Home the GRBL device."""
        self.connection.send("$H")

    def is_idle(self) -> bool:
        """Returns true if machine is idle."""
        if self.op_state == OperationState.IDLE:
            return True
        return False

    def get_status(self) -> List[str]:
        """Get the current status of the GRBL device."""
        # ? returns the active GRBL state & current machine and work positions
        return self.connection.send_and_read_response("?")

    def get_settings(self):
        """Get the current settings of the GRBL device."""
        # $$ returns the current GRBL settings
        return self.connection.send_and_read_response("$$")

    def decode_grbl_error(self, grbl_err_str: str) -> str:
        if("error" in grbl_err_str):
            err = grbl_err_str.split(":", 1)[1]
            if(type(err) == int()):
                return GrblError((int(err))).name
            else:
                return err
        else:
            return grbl_err_str

    def set_speed(self, speed):
        self.defaultSpeed = speed

    def set_origin(self, new_origin: VectorXYZ):
        """Set the origin of the device."""
        # set current position to be (0,0,0), or a custom (x,y,z)
        gcode = "G92 x{} y{} z{}\n".format(
            new_origin.x,
            new_origin.y,
            new_origin.z)
        self.connection.send(self, gcode)

    def clear_alarm(self):
        """Clear the alarm on the GRBL machine."""
        return self.connection.send(self, r"\$X")

    def enable_steppers(self):
        """Enable the motors on the GRBL machine."""
        return self.connection.send(self, "M17")

    def disable_steppers(self):
        """Disable the motors on the GRBL machine."""
        return self.connection.send(self, "M17")

    def feed_hold(self):
        """Feed hold the GRBL machine."""
        return self.connection.send(self, r"\!")

    def ensure_movement_mode(self, absolute_mode=True):
        """Ensure GRBL device is in the expected movement mode."""
        if self.abs_move == absolute_mode:
            return
        if absolute_mode:
            self.connection.send("G90")  # absolute movement mode
        else:
            self.connection.send("G91")  # relative movement mode
        self.abs_move = absolute_mode
        return self.connection.read()

    def send_gcode(self, gcode: str) -> str:
        """Send some specified GCODE to the GRBL machine."""
        # TODO(evanflynn): need to add some input checking to make sure its valid GCODE
        return self.connection.send_and_read_response(gcode)

    def parse_status_fields(self, status: str) -> List[str]:
        if "|" not in status:
            # No fields to parse
            return []
        # seperate fields using pipe delimeter |
        fields = status.split('|')
        # clean first and last field of '<>'
        fields[0] = fields[0][1:]
        fields[-1] = fields[-1][:-1]
        return fields
