"""Synchronous request/reply bus. No unsolicited retries of motion commands."""
from dataclasses import asdict
import threading
import time
from .logging import EventLog
from .model import Command, Units, state_from_frame
from .protocol import CRCError, FrameError, FrameStream, PROFILES, checked_id, encode, finite


class BusTimeout(TimeoutError):
    pass


class MotorBus:
    def __init__(self, transport, *, profile=PROFILES["stock"], units=None,
                 timeout=0.05, max_consecutive_crc=3, log=None):
        finite(timeout)
        if timeout <= 0 or type(max_consecutive_crc) is not int or max_consecutive_crc < 1:
            raise ValueError("timeout and CRC error limit must be positive")
        self.transport, self.profile = transport, profile
        self.units = units or {}
        self.timeout, self.max_crc = timeout, max_consecutive_crc
        self.log = log or EventLog()
        self.stream = FrameStream(profile)
        self.lock = threading.Lock()
        self.last_transaction = None
        self.log.emit("session", profile=profile.name, units={str(k): asdict(v) for k,v in self.units.items()},
                      timeout_s=timeout, max_consecutive_crc=max_consecutive_crc,
                      clock="perf_counter", clock_resolution_s=time.get_clock_info("perf_counter").resolution,
                      evidence=profile.evidence)

    def exchange(self, command: Command):
        checked_id(command.motor_id)
        unit = self.units.get(command.motor_id, Units())
        raw_command = command.raw(unit)
        frame = encode(raw_command, self.profile)  # validate before I/O
        with self.lock:
            self.stream.reset()
            self.stream.consecutive_crc_errors = 0
            started = time.perf_counter()
            self.log.emit("tx", motor_id=command.motor_id, hex=frame.hex(" "),
                          command=asdict(command), raw=asdict(raw_command), sent_at=started)
            try:
                self.transport.discard_input()
                if self.transport.write(frame) != len(frame):
                    raise OSError("short serial write")
                while time.perf_counter() - started < self.timeout:
                    chunk = self.transport.read(256)
                    if not chunk:
                        continue
                    received = time.perf_counter()
                    self.log.emit("rx_chunk", hex=chunk.hex(" "), received_at=received)
                    if received-started > self.timeout:
                        raise BusTimeout("reply arrived after deadline")
                    before = self.stream.crc_errors
                    frames = self.stream.feed(chunk)
                    if self.stream.crc_errors != before:
                        self.log.emit("crc_error", count=self.stream.crc_errors-before,
                                      consecutive=self.stream.consecutive_crc_errors,
                                      peak_consecutive=self.stream.peak_consecutive_crc_errors)
                    if self.stream.peak_consecutive_crc_errors >= self.max_crc:
                        raise CRCError("consecutive CRC error threshold reached")
                    for reply in frames:
                        raw_id = reply[2] & 15
                        self.log.emit("rx", hex=reply.hex(" "), motor_id=raw_id)
                        if raw_id != command.motor_id:
                            self.log.emit("id_mismatch", expected=command.motor_id, received=raw_id)
                            continue
                        state = state_from_frame(reply, self.profile, unit, received)
                        if state.mode != command.mode or state.raw["reserved_mode_bit"]:
                            raise FrameError("reply mode mismatch or reserved mode bit set")
                        self.stream.consecutive_crc_errors = 0
                        self.log.emit("state", state=state.as_dict())
                        self.last_transaction = {"sent_at": started, "received_at": received}
                        self.log.emit("transaction", motor_id=command.motor_id, command_mode=command.mode, ok=True,
                                      sent_at=started, received_at=received, latency_s=received-started)
                        return state
                raise BusTimeout(f"ID {command.motor_id} reply timeout")
            except BaseException as exc:
                self.log.emit("transaction", motor_id=command.motor_id, command_mode=command.mode, ok=False, sent_at=started,
                              reason=str(exc), exception=type(exc).__name__)
                raise

    def status(self, motor_id):
        # Mode 0 is a transmitted command, not a passive read or proven safe stop.
        return self.exchange(Command(checked_id(motor_id), mode=0))

    def close(self):
        self.transport.close()
