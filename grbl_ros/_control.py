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


def home(self):
    self.s.write(b'$H\n')
    return self.s.readline().decode('utf-8').strip()


def moveTo(self, x=None, y=None, z=None, speed=None, blockUntilComplete=True):
    # TODO(flynneva): migrate to just pose msg arg to cover higher order axis machines too
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
