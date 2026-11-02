"""Raw values, axis-aware conversions and explicit uncertainty."""
from dataclasses import asdict, dataclass
from .protocol import RawCommand, Profile, decode, f32, finite

TWO_PI_SDK = 6.2832


@dataclass(frozen=True)
class Units:
    axis: str = "rotor"
    direction: int = 1
    zero_raw: float = 0
    gear_ratio: float = 6.33
    gear_ratio_verified: bool = False
    angle_rad_per_count: float | None = None  # directly in the selected axis
    velocity_rad_s_per_count: float | None = None
    torque_nm_per_count: float | None = None
    angle_calibration_evidence: str = ""

    def __post_init__(self):
        if self.axis not in ("rotor", "output") or type(self.direction) is not int or self.direction not in (-1, 1):
            raise ValueError("axis must be rotor/output; direction must be +1/-1")
        finite(self.zero_raw)
        if finite(self.gear_ratio) <= 0 or finite(self.gear_ratio * self.gear_ratio) == 0:
            raise ValueError("gear ratio must be positive and its square representable")
        if type(self.gear_ratio_verified) is not bool or not isinstance(self.angle_calibration_evidence, str):
            raise ValueError("verification flag must be boolean and evidence must be text")
        for value in (self.angle_rad_per_count, self.velocity_rad_s_per_count, self.torque_nm_per_count):
            if value is not None and finite(value) <= 0:
                raise ValueError("empirical conversion coefficients must be positive; use direction for sign")
        if self.angle_rad_per_count is not None and not self.angle_calibration_evidence:
            raise ValueError("angle coefficient requires calibration evidence")

    @property
    def ratio(self):
        return self.gear_ratio if self.axis == "output" else 1.0

    @property
    def angle_scale(self):
        return self.angle_rad_per_count or TWO_PI_SDK / 32768 / self.ratio

    @property
    def speed_scale(self):
        return self.velocity_rad_s_per_count or TWO_PI_SDK / 256 / self.ratio

    @property
    def torque_scale(self):
        return self.torque_nm_per_count or self.ratio / 256

    def status(self, profile: Profile) -> dict:
        return {"axis": self.axis, "angle": "calibrated" if self.angle_rad_per_count else "uncalibrated",
                "angle_evidence": self.angle_calibration_evidence,
                "velocity": "empirical coefficient" if self.velocity_rad_s_per_count else "development scale; stock unverified",
                "torque": "empirical coefficient; source unconfirmed" if self.torque_nm_per_count else "development scale; stock unverified; source unconfirmed",
                "gear_ratio": self.gear_ratio, "gear_ratio_verified": self.gear_ratio_verified,
                "profile_evidence": profile.evidence,
                "gains": "SDK rotor convention; stock gain response unverified"}


@dataclass(frozen=True)
class Command:
    motor_id: int
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    mode: int = 1

    def raw(self, units: Units) -> RawCommand:
        if type(self.mode) is not int or self.mode not in (0, 1):
            raise ValueError("only integer modes 0 and 1 are supported")
        for value in (self.position, self.velocity, self.torque, self.kp, self.kd):
            finite(value)
        if self.mode == 0:
            if any((self.position, self.velocity, self.torque, self.kp, self.kd)):
                raise ValueError("mode 0 must have zero values")
            return RawCommand(self.motor_id)
        # Match official MotorCmd float32 input followed by double scaling and
        # truncation toward zero. Empirical position scale is inverted directly.
        if units.angle_rad_per_count is None and units.zero_raw == 0:
            pos = int(f32(self.position * units.ratio * units.direction) / TWO_PI_SDK * 32768)
        else:
            pos = int(finite(self.position / units.angle_scale * units.direction + units.zero_raw))
        speed = (int(f32(self.velocity * units.ratio * units.direction) / TWO_PI_SDK * 256)
                 if units.velocity_rad_s_per_count is None
                 else int(f32(self.velocity / units.speed_scale * units.direction)))
        torque = int(f32(self.torque / units.torque_scale * units.direction))
        kp = int(f32(self.kp / units.ratio**2) / 25.6 * 32768)
        kd = int(f32(self.kd / units.ratio**2) / 25.6 * 32768)
        # The fixed official SDK clamps these input ranges. Reject instead of
        # silently changing the requested command. These are encoding limits,
        # not ratings or verified stock motor operating limits.
        rotor_q = pos * TWO_PI_SDK / 32768
        rotor_dq = self.velocity * units.ratio if units.velocity_rad_s_per_count is None else speed * TWO_PI_SDK / 256
        rotor_tau = self.torque / units.ratio if units.torque_nm_per_count is None else torque / 256
        if abs(rotor_q) > 411774 or abs(rotor_dq) > 804 or abs(rotor_tau) > 127.99:
            raise ValueError("command exceeds official SDK encoding domain")
        if not 0 <= self.kp / units.ratio**2 <= 25.599 or not 0 <= self.kd / units.ratio**2 <= 25.599:
            raise ValueError("gain exceeds official SDK encoding domain [0, 25.599]")
        return RawCommand(self.motor_id, self.mode, torque, speed, pos, kp, kd)


@dataclass(frozen=True)
class State:
    motor_id: int
    mode: int
    position: float
    velocity: float
    torque: float
    temperature: int
    error: int
    raw: dict
    validation: dict
    timestamp: float

    def as_dict(self):
        return asdict(self)


def state_from_frame(frame: bytes, profile: Profile, units: Units, timestamp: float) -> State:
    raw = decode(frame, profile)
    return State(raw["motor_id"], raw["mode"],
                 (raw["position"] - units.zero_raw) * units.angle_scale * units.direction,
                 raw["speed"] * units.speed_scale * units.direction,
                 raw["torque"] * units.torque_scale * units.direction,
                 raw["temperature"], raw["error_candidate"], raw, units.status(profile), timestamp)
