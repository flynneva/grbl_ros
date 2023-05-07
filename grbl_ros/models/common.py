from enum import IntEnum

from pydantic import BaseModel


class Limits(BaseModel):
    x_max: int = 0
    y_max: int = 0
    z_max: int = 0


class VectorXYZ(BaseModel):
    x: int = 0
    y: int = 0
    z: int = 0


class Quaternion(BaseModel):
    x: int = 0
    y: int = 0
    z: int = 0
    w: int = 0


class OperationState(IntEnum):
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


class MovementMode(IntEnum):
    """GRBL has 2 move modes: relative and absolute."""
    RELATIVE = 0
    ABSOLUTE = 1


class GrblError(IntEnum):
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
