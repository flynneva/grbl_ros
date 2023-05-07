from serial import Serial
from serial.serialutil import SerialException
import sys
from time import sleep
from typing import List, Optional

from pydantic import BaseModel, root_validator


class UART(BaseModel):
    """UART connector to a GRBL device."""
    baudrate: int
    port: str
    timeout: int = 5                 # Timeout in seconds
    connection: Optional[Serial]

    class Config:
        arbitrary_types_allowed = True

    @root_validator(pre=True)
    def validate_root(cls, values):
        port, baudrate = values.get("port"), values.get("baudrate")
        if port and baudrate:
            values["connection"] = Serial(port, baudrate)
        return values

    def __del__(self):
        self.connection.close()

    def connect(self):
        """Connect to the GRBL device."""
        try:
            self.conection = Serial(self.port, self.baudrate, timeout=self.timeout)
        except SerialException:
            # TODO(flynneva): log something here
            sys.exit(1)
    
    def read(self) -> List[str]:
        """Read all data available from the GRBL device via UART."""
        responses = []
        # wait until receive response with EOL character
        resp = self.connection.readline().decode('utf-8').strip()
        if(len(resp) > 0):
            responses.append(resp)
        # check to see if there are more lines in waiting
        while (self.connection.inWaiting() > 0):
            responses.append(self.read())
        return responses

    def send(self, data: str):
        """
        Transmit data to the GRBL device.
        
        Appends an end of line character (\\n) to the provided string.
        """
        self.connection.write(str.encode(data + "\n"))

    def send_and_read_response(self, data: str) -> List[str]:
        """Transmit data to the GRBL device and read the response."""
        self.send(data)
        sleep(0.05)  # give some time for GRBL device
        return self.read()


