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
import serial


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
