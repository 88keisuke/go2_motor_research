"""Bounded trajectories and abort handling. No firmware/configuration operations."""
from dataclasses import dataclass
import math
import time
from .model import Command
from .protocol import checked_id, encode, finite


class SafetyAbort(RuntimeError):
    pass


@dataclass(frozen=True)
class Limits:
    position_min: float = -3.0
    position_max: float = 3.0
    velocity_max: float = 1.0
    torque_max: float = 0.5
    kp_max: float = 2.0
    kd_max: float = 0.5
    temperature_max: float = 60.0
    following_error_max: float = 0.15

    def __post_init__(self):
        for value in self.__dict__.values():
            finite(value)
        if self.position_min >= self.position_max or any(v <= 0 for k,v in self.__dict__.items() if k not in ("position_min", "position_max")):
            raise ValueError("invalid safety limits")

    def check_command(self, command):
        for v in (command.position, command.velocity, command.torque, command.kp, command.kd):
            finite(v)
        if not self.position_min <= command.position <= self.position_max:
            raise ValueError("command position outside configured range")
        if abs(command.velocity) > self.velocity_max or abs(command.torque) > self.torque_max:
            raise ValueError("command velocity/torque outside configured limits")
        if not 0 <= command.kp <= self.kp_max or not 0 <= command.kd <= self.kd_max:
            raise ValueError("gain outside configured limits")

    def check_state(self, state, expected=None):
        if state.temperature >= self.temperature_max:
            raise SafetyAbort(f"ID {state.motor_id}: overtemperature {state.temperature}")
        if state.error:
            raise SafetyAbort(f"ID {state.motor_id}: error candidate {state.error}")
        if not self.position_min <= state.position <= self.position_max:
            raise SafetyAbort(f"ID {state.motor_id}: measured position outside limits")
        if abs(state.velocity) > self.velocity_max or abs(state.torque) > self.torque_max:
            raise SafetyAbort(f"ID {state.motor_id}: measured speed/torque outside limits")
        if expected is not None and abs(state.position-expected) > self.following_error_max:
            raise SafetyAbort(f"ID {state.motor_id}: following error {state.position-expected:.6f} rad")


@dataclass(frozen=True)
class Termination:
    mode: int = 0
    verified: bool = False
    evidence: str = ""

    def require_verified(self):
        if type(self.mode) is not int or self.mode != 0 or self.verified is not True or not isinstance(self.evidence, str) or not self.evidence.strip():
            raise ValueError("motion requires a mode-0 termination verified on this setup, with evidence; default is unverified")


def smoothstep(fraction):
    x = max(0.0, min(1.0, fraction))
    return x*x*x*(10-15*x+6*x*x), 30*x*x*(1-x)*(1-x)


class MotorController:
    def __init__(self, bus, *, limits=None, termination=Termination()):
        self.bus = bus
        self.limits = limits or {}
        self.termination = termination

    def limit(self, motor_id):
        return self.limits.get(motor_id, Limits())

    def validate_command(self, command):
        self.limit(command.motor_id).check_command(command)
        encode(command.raw(self.bus.units.get(command.motor_id) or self._default_units()), self.bus.profile)

    @staticmethod
    def _default_units():
        from .model import Units
        return Units()

    def terminate(self, ids, reason):
        outcomes = []
        for motor_id in ids:
            try:
                state = self.bus.status(motor_id)
                # An ACK says only that the frame was answered. It does not prove
                # zero motion, de-energization or mechanical stopping.
                outcome = {"motor_id": motor_id, "acknowledged": True,
                           "reply": state.as_dict(), "mechanical_stop_proven": False}
                if state.error or state.temperature >= self.limit(motor_id).temperature_max:
                    outcome["warning"] = "termination reply still reports error/overtemperature"
            except BaseException as exc:
                outcome = {"motor_id": motor_id, "acknowledged": False, "error": str(exc),
                           "mechanical_stop_proven": False}
            outcomes.append(outcome)
            self.bus.log.emit("termination", reason=reason, evidence=self.termination.evidence, **outcome)
        return outcomes

    def run(self, targets, *, duration=1.0, period=0.02, kp=0.1, kd=0.01,
            kind="position", ramp=0.5):
        """Position targets are absolute selected-axis radians; velocity/torque
        targets use the same per-ID units. The initial status is always acquired.
        Serial ID ordering is preserved and actual sample time drives the curve.
        """
        if not targets or kind not in ("position", "velocity", "torque"):
            raise ValueError("explicit IDs and a known control kind are required")
        ids = [checked_id(i) for i in targets]
        for v in (duration, period, kp, kd, ramp):
            finite(v)
        if duration <= 0 or period <= 0 or ramp <= 0 or period > duration:
            raise ValueError("duration/period/ramp must be positive; period <= duration")
        self.termination.require_verified()  # before any I/O
        # Validate all target values and packing before the first status request.
        for motor_id, value in targets.items():
            finite(value)
            placeholder = min(max(0, self.limit(motor_id).position_min), self.limit(motor_id).position_max)
            command = Command(motor_id, value if kind == "position" else placeholder,
                              value if kind == "velocity" else 0, value if kind == "torque" else 0,
                              kp if kind == "position" else 0, kd if kind != "torque" else 0)
            self.validate_command(command)
        outcomes = []
        try:
            starts = {}
            for motor_id in ids:
                state = self.bus.status(motor_id)
                self.limit(motor_id).check_state(state)
                starts[motor_id] = state.position
            for motor_id in ids:
                if kind == "position":
                    # Maximum slope of quintic is 1.875. Never just relax following limits.
                    max_speed = 1.875*abs(targets[motor_id]-starts[motor_id])/duration
                    self.validate_command(Command(motor_id, targets[motor_id], max_speed, 0, kp, kd))
                else:
                    self.validate_command(Command(motor_id, starts[motor_id],
                                                   targets[motor_id] if kind == "velocity" else 0,
                                                   targets[motor_id] if kind == "torque" else 0,
                                                   0, kd if kind == "velocity" else 0))
            began, cycle = time.perf_counter(), 0
            while True:
                cycle_start = time.perf_counter()
                final_cycle = cycle_start-began >= duration
                offsets = {}
                for motor_id in ids:
                    sampled = time.perf_counter()
                    elapsed = sampled-began
                    if kind == "position":
                        blend, derivative = smoothstep(elapsed/duration)
                        delta = targets[motor_id]-starts[motor_id]
                        command = Command(motor_id, starts[motor_id]+delta*blend,
                                          delta*derivative/duration if elapsed < duration else 0, 0, kp, kd)
                    else:
                        blend, _ = smoothstep(elapsed/ramp)
                        command = Command(motor_id, starts[motor_id],
                                          targets[motor_id]*blend if kind == "velocity" else 0,
                                          targets[motor_id]*blend if kind == "torque" else 0,
                                          0, kd if kind == "velocity" else 0)
                    self.validate_command(command)
                    state = self.bus.exchange(command)
                    offsets[str(motor_id)] = self.bus.last_transaction["sent_at"]-cycle_start
                    self.limit(motor_id).check_state(state, command.position if kind == "position" else None)
                    self.bus.log.emit("tracking", motor_id=motor_id, expected=command.position,
                                      actual=state.position, error_rad=state.position-command.position,
                                      elapsed_s=elapsed, kind=kind)
                ended = time.perf_counter()
                self.bus.log.emit("cycle", cycle=cycle, per_id_send_offset_s=offsets,
                                  skew_s=max(offsets.values())-min(offsets.values()),
                                  duration_s=ended-cycle_start, requested_period_s=period,
                                  overrun_s=max(0, ended-cycle_start-period))
                if final_cycle:
                    break
                cycle += 1
                next_deadline = began+cycle*period
                if next_deadline < ended:
                    cycle = math.floor((ended-began)/period)+1  # never burst to catch up
                    next_deadline = began+cycle*period
                time.sleep(max(0, next_deadline-time.perf_counter()))
        except BaseException as exc:
            self.bus.log.emit("abort", reason=str(exc), exception=type(exc).__name__)
            outcomes = self.terminate(ids, str(exc))
            raise
        else:
            outcomes = self.terminate(ids, "trajectory completed")
            if not all(o["acknowledged"] for o in outcomes):
                raise SafetyAbort("trajectory ended but termination replies missing; stopping is unconfirmed")
        return {"trajectory_completed": True, "termination": outcomes, "mechanical_stop_proven": False}
