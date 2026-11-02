"""Device I/O only. SerialTransport requires explicit transmit authorization."""
from collections import deque
import time
from typing import Protocol


class Transport(Protocol):
    def write(self, data: bytes) -> int: ...
    def read(self, size: int) -> bytes: ...
    def discard_input(self) -> None: ...
    def close(self) -> None: ...


class SerialTransport:
    def __init__(self, device: str, *, allow_send: bool = False, baud: int = 4_000_000,
                 read_timeout: float = 0.002):
        if allow_send is not True:
            raise PermissionError("serial I/O requires explicit --send / allow_send=True")
        if not device or baud <= 0 or read_timeout <= 0:
            raise ValueError("device, positive baud and read timeout required")
        import serial  # optional, never loaded by offline commands
        self.serial = serial.Serial(device, baudrate=baud, bytesize=8, parity="N", stopbits=1,
                                    timeout=read_timeout, write_timeout=0.05,
                                    xonxoff=False, rtscts=False, dsrdtr=False)

    def write(self, data):
        return self.serial.write(data)

    def read(self, size):
        return self.serial.read(size)

    def discard_input(self):
        self.serial.reset_input_buffer()

    def close(self):
        self.serial.close()


class VirtualTransport:
    """Each write consumes one scripted response (list of arbitrary read chunks).

    Empty response simulates timeout. No serializer is shared with the driver.
    """
    def __init__(self, responses=(), responder=None):
        self.responses = deque(responses)
        self.responder = responder
        self.pending = deque()
        self.writes = []
        self.closed = False

    def write(self, data):
        if self.closed:
            raise OSError("virtual transport closed")
        self.writes.append(bytes(data))
        response = self.responder(data) if self.responder else (self.responses.popleft() if self.responses else [])
        self.pending.extend(response)
        return len(data)

    def read(self, size):
        if not self.pending:
            time.sleep(0.0002)
            return b""
        data = self.pending.popleft()
        if len(data) > size:
            self.pending.appendleft(data[size:])
        return data[:size]

    def discard_input(self):
        self.pending.clear()

    def close(self):
        self.closed = True
