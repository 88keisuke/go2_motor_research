"""Pure byte processing. No serial imports; portable reference for MCU work."""
from dataclasses import dataclass
import math
import struct


@dataclass(frozen=True)
class CRCSpec:
    initial: int
    polynomial_reflected: int = 0x8408  # normal polynomial 0x1021
    xor_out: int = 0
    byte_order: str = "little"

    def calculate(self, data: bytes) -> int:
        value = self.initial
        for byte in data:
            value ^= byte
            for _ in range(8):
                value = (value >> 1) ^ (self.polynomial_reflected if value & 1 else 0)
        return value ^ self.xor_out


@dataclass(frozen=True)
class Profile:
    name: str
    tx_crc: CRCSpec
    rx_crc: CRCSpec
    evidence: str


PROFILES = {
    "development": Profile("development", CRCSpec(0), CRCSpec(0), "official SDK GO-M8010-6"),
    "stock": Profile("stock", CRCSpec(0x2CBB), CRCSpec(0x2CBB),
                     "CRC matches supplied captures; field meanings/scales are hypotheses"),
}


class FrameError(ValueError):
    pass


class CRCError(FrameError):
    pass


def checked_id(motor_id: int) -> int:
    if type(motor_id) is not int or not 0 <= motor_id <= 14:
        raise ValueError("target ID must be an explicit integer 0..14; no broadcast")
    return motor_id


def integer(value: int, bits: int, signed: bool = True) -> int:
    low = -(1 << (bits - 1)) if signed else 0
    high = (1 << (bits - int(signed))) - 1
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"integer {value!r} outside {bits}-bit range [{low}, {high}]")
    return value


def finite(value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("all numeric settings must be finite numbers")
    return value


def f32(value: float) -> float:
    finite(value)
    try:
        result = struct.unpack("<f", struct.pack("<f", value))[0]
    except (OverflowError, struct.error) as exc:
        raise ValueError("float32 overflow") from exc
    finite(result)
    return result


@dataclass(frozen=True)
class RawCommand:
    motor_id: int
    mode: int = 0
    torque: int = 0
    speed: int = 0
    position: int = 0
    kp: int = 0
    kd: int = 0


def encode(command: RawCommand, profile: Profile) -> bytes:
    checked_id(command.motor_id)
    if type(command.mode) is not int or command.mode not in (0, 1):
        raise ValueError("only modes 0 and 1 are implemented; no calibration/config writes")
    values = (integer(command.torque, 16), integer(command.speed, 16),
              integer(command.position, 32), integer(command.kp, 16, False),
              integer(command.kd, 16, False))
    if command.kp > 32767 or command.kd > 32767:
        raise ValueError("gain exceeds official normalized Q15 range")
    if command.mode == 0 and any(values):
        raise ValueError("mode 0 requests must have zero control fields")
    body = b"\xfe\xee" + bytes([command.motor_id | command.mode << 4]) + struct.pack("<hhiHH", *values)
    return body + profile.tx_crc.calculate(body).to_bytes(2, "little")


def validate(frame: bytes, profile: Profile, direction: str) -> None:
    if direction not in ("tx", "rx"):
        raise ValueError("direction must be tx or rx")
    length, header, spec = (17, b"\xfe\xee", profile.tx_crc) if direction == "tx" else (16, b"\xfd\xee", profile.rx_crc)
    if len(frame) != length or frame[:2] != header:
        raise FrameError(f"expected {direction} header {header.hex(' ')} and {length} bytes")
    expected = spec.calculate(frame[:-2])
    actual = int.from_bytes(frame[-2:], spec.byte_order)
    if actual != expected:
        raise CRCError(f"{direction} CRC received=0x{actual:04x} calculated=0x{expected:04x}")


def decode(frame: bytes, profile: Profile, direction: str = "rx") -> dict:
    validate(frame, profile, direction)
    control = frame[2]
    result = {"motor_id": control & 15, "mode": (control >> 4) & 7,
              "reserved_mode_bit": control >> 7, "crc_valid": True,
              "profile": profile.name, "evidence": profile.evidence, "hex": frame.hex(" ")}
    if direction == "tx":
        tau, speed, pos, kp, kd = struct.unpack_from("<hhiHH", frame, 3)
        result.update(torque=tau, speed=speed, position=pos, kp=kp, kd=kd)
    else:
        tau, speed, pos, temp, auxiliary = struct.unpack_from("<hhibH", frame, 3)
        result.update(torque=tau, speed=speed, position=pos, temperature=temp,
                      auxiliary_raw=auxiliary, error_candidate=auxiliary & 7,
                      auxiliary_bits_3_14=(auxiliary >> 3) & 4095,
                      auxiliary_reserved_bit=auxiliary >> 15,
                      auxiliary_meaning="unconfirmed; not identified as foot force",
                      torque_source="unconfirmed: measured vs current-derived vs other")
    return result


class FrameStream:
    """Resynchronizing bounded parser. Retains incomplete header/frame across reads."""
    def __init__(self, profile: Profile):
        self.profile = profile
        self.buffer = bytearray()
        self.crc_errors = 0
        self.consecutive_crc_errors = 0
        self.noise_bytes = 0
        self.peak_consecutive_crc_errors = 0

    def feed(self, chunk: bytes) -> list[bytes]:
        self.peak_consecutive_crc_errors = self.consecutive_crc_errors
        self.buffer.extend(chunk)
        frames = []
        while self.buffer:
            index = self.buffer.find(b"\xfd\xee")
            if index < 0:
                keep = int(self.buffer[-1] == 0xFD)
                self.noise_bytes += len(self.buffer) - keep
                self.buffer[:] = self.buffer[-1:] if keep else b""
                break
            if index:
                self.noise_bytes += index
                del self.buffer[:index]
            if len(self.buffer) < 16:
                break
            candidate = bytes(self.buffer[:16])
            try:
                validate(candidate, self.profile, "rx")
            except CRCError:
                self.crc_errors += 1
                self.consecutive_crc_errors += 1
                self.peak_consecutive_crc_errors = max(self.peak_consecutive_crc_errors, self.consecutive_crc_errors)
                del self.buffer[0]
                continue
            frames.append(candidate)
            self.consecutive_crc_errors = 0
            del self.buffer[:16]
        return frames

    def reset(self) -> None:
        self.buffer.clear()
