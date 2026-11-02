"""Offline by default. Every device-writing CLI path requires --send."""
import argparse
from dataclasses import asdict
import json
import sys
import time
from .calibration import compare_velocity, fit_angle
from .control import Limits, MotorController, Termination
from .driver import MotorBus
from .logging import EventLog, compare_logs, replay
from .model import Command, Units
from .protocol import PROFILES, checked_id, decode, encode, finite
from .transport import SerialTransport


def read_json(path):
    with open(path, encoding="utf-8") as file:
        return json.load(file)


def show(value):
    print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))


def configuration(path):
    config = read_json(path) if path else {}
    if not isinstance(config, dict) or not isinstance(config.get("motors", {}), dict):
        raise ValueError("configuration and motors must be JSON objects")
    for key in config.get("motors", {}):
        checked_id(int(key))
    defaults = config.get("units", {})
    default_limits = config.get("limits", {})
    units, limits = {}, {}
    for motor_id in range(15):
        entry = config.get("motors", {}).get(str(motor_id), {})
        units[motor_id] = Units(**{**defaults, **entry.get("units", {})})
        limits[motor_id] = Limits(**{**default_limits, **entry.get("limits", {})})
    return units, limits, Termination(**config.get("termination", {}))


def targets_for(args):
    if args.action == "multi":
        source = read_json(args.targets)
        if not isinstance(source, dict) or not source:
            raise ValueError("targets must be a nonempty JSON object {ID: absolute radians}")
        targets = {checked_id(int(k)): finite(v) for k,v in source.items()}
        if len(targets) != len(source):
            raise ValueError("duplicate motor IDs")
        return targets
    checked_id(args.id)
    return {args.id: args.target}


def dry_run(args, units, limits, termination):
    targets = targets_for(args) if args.action != "status" else {checked_id(i): 0 for i in args.ids}
    frames = []
    kind = {"move": "position", "multi": "position"}.get(args.action, args.action)
    for motor_id, target in targets.items():
        unit = units[motor_id]
        frames.append({"stage": "initial status", "motor_id": motor_id,
                       "hex": encode(Command(motor_id, mode=0).raw(unit), PROFILES[args.profile]).hex(" ")})
        if args.action == "status":
            continue
        for value in (args.duration, args.period, args.ramp):
            finite(value)
        if args.duration <= 0 or not 0 < args.period <= args.duration or args.ramp <= 0:
            raise ValueError("invalid duration/period/ramp")
        # A displayed example start is never used as the live trajectory start.
        start = args.preview_start
        finite(start)
        preview_count = int(args.duration/args.period)+2
        if preview_count > 10000:
            raise ValueError("dry-run preview exceeds 10000 steps; shorten duration or increase period")
        from .control import smoothstep
        for tick in range(preview_count):
            elapsed = min(tick*args.period, args.duration)
            fraction, derivative = smoothstep(elapsed/(args.duration if kind == "position" else args.ramp))
            command = Command(motor_id,
                              start+(target-start)*fraction if kind == "position" else start,
                              (target-start)*derivative/args.duration if kind == "position" else target*fraction if kind == "velocity" else 0,
                              target*fraction if kind == "torque" else 0,
                              args.kp if kind == "position" else 0, args.kd if kind != "torque" else 0)
            limits[motor_id].check_command(command)
            frame = encode(command.raw(unit), PROFILES[args.profile])
            frames.append({"stage": "trajectory preview", "elapsed_s": elapsed, "motor_id": motor_id,
                           "hex": frame.hex(" "), "command": asdict(command)})
            if elapsed >= args.duration:
                break
        frames.append({"stage": "termination candidate; verification required for live motion",
                       "motor_id": motor_id, "hex": encode(Command(motor_id, mode=0).raw(unit), PROFILES[args.profile]).hex(" ")})
    return {"dry_run": True, "device_opened": False, "profile": args.profile,
            "preview_start_rad": getattr(args, "preview_start", None),
            "note": "example start only; live control always polls actual positions; multi-ID TX is sequential",
            "termination": asdict(termination), "units": {str(i): units[i].status(PROFILES[args.profile]) for i in targets},
            "frames": frames}


def run_device(args):
    units, limits, termination = configuration(args.config)
    if not args.send:
        show(dry_run(args, units, limits, termination))
        return
    if not args.device or not args.log:
        raise ValueError("--send requires --device and a new --log JSONL path")
    if args.action != "status":
        targets_for(args)
        termination.require_verified()  # before opening the port
    else:
        if not args.ids or len(set(args.ids)) != len(args.ids):
            raise ValueError("explicit distinct --ids required")
        for motor_id in args.ids:
            checked_id(motor_id)
        if type(args.count) is not int or args.count < 1 or finite(args.period) <= 0:
            raise ValueError("count and period must be positive")
    log = EventLog(args.log)
    bus = None
    transport = None
    try:
        transport = SerialTransport(args.device, allow_send=True, baud=args.baud)
        bus = MotorBus(transport, profile=PROFILES[args.profile], units=units,
                       timeout=args.timeout, max_consecutive_crc=args.max_crc, log=log)
        controller = MotorController(bus, limits=limits, termination=termination)
        if args.action == "status":
            for _ in range(args.count):
                for motor_id in args.ids:
                    state = bus.status(motor_id)
                    show(state.as_dict())
                    limits[motor_id].check_state(state)
                if args.count > 1:
                    time.sleep(args.period)
        else:
            result = controller.run(targets_for(args), duration=args.duration, period=args.period,
                                    kp=args.kp, kd=args.kd, ramp=args.ramp,
                                    kind={"move":"position", "multi":"position"}.get(args.action,args.action))
            show(result)
    finally:
        if transport:
            transport.close()
        log.close()
    report = replay(args.log)
    show({"timing": report["timing"], "cycle_skew_s": report["cycle_skew_s"],
          "note": "host measured; not the motor internal control period"})


def parser():
    root = argparse.ArgumentParser(description="Go2 Air experimental driver: default dry-run, no automatic hardware TX")
    commands = root.add_subparsers(dest="action", required=True)
    commands.add_parser("ports", help="list OS device paths; does not open a device")
    p = commands.add_parser("decode", help="CRC-check and decode a hex frame")
    p.add_argument("hex")
    p.add_argument("--direction", choices=["tx","rx"], default="rx")
    p.add_argument("--profile", choices=PROFILES, default="stock")
    p = commands.add_parser("replay", help="reanalyze JSONL without opening serial")
    p.add_argument("path")
    p.add_argument("--profile", choices=PROFILES)
    p = commands.add_parser("compare", help="descriptive comparison of stock/development logs")
    p.add_argument("stock_log"); p.add_argument("development_log")
    p = commands.add_parser("calibrate", help="fit known angle/raw pairs; never drives a motor")
    p.add_argument("samples"); p.add_argument("--axis", choices=["rotor","output"], default="output")
    p.add_argument("--output", help="new JSON report path")
    p = commands.add_parser("speed-check", help="compare calibrated angle derivative with speed field")
    p.add_argument("samples"); p.add_argument("--calibration", required=True)
    p.add_argument("--min-dt", type=float, default=0.01); p.add_argument("--max-dt", type=float, default=0.5)
    p.add_argument("--raw-noise", type=float, default=1)
    for action in ("status", "move", "multi", "velocity", "torque"):
        p = commands.add_parser(action)
        p.add_argument("--profile", choices=PROFILES, default="stock")
        p.add_argument("--config"); p.add_argument("--device"); p.add_argument("--log")
        p.add_argument("--baud", type=int, default=4_000_000)
        p.add_argument("--timeout", type=float, default=0.05)
        p.add_argument("--max-crc", type=int, default=3)
        mode = p.add_mutually_exclusive_group()
        mode.add_argument("--send", action="store_true", help="explicitly enable physical TX")
        mode.add_argument("--dry-run", action="store_true", help="print planned bytes (default)")
        p.add_argument("--period", type=float, default=0.02)
        if action == "status":
            p.add_argument("--ids", type=int, nargs="+", required=True)
            p.add_argument("--count", type=int, default=1)
        else:
            if action == "multi":
                p.add_argument("--targets", required=True, help="JSON {ID: absolute target rad}")
            else:
                p.add_argument("--id", type=int, required=True)
                p.add_argument("--target", type=float, required=True)
            p.add_argument("--duration", type=float, default=2)
            p.add_argument("--ramp", type=float, default=0.5)
            p.add_argument("--kp", type=float, default=0.1)
            p.add_argument("--kd", type=float, default=0.01)
            p.add_argument("--preview-start", type=float, default=0, help="dry-run example start only")
    return root


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.action == "ports":
            from serial.tools.list_ports import comports
            show([{"device": p.device, "description": p.description} for p in comports()])
        elif args.action == "decode":
            raw = bytes.fromhex(args.hex)
            result = decode(raw, PROFILES[args.profile], args.direction)
            if args.direction == "rx":
                from .model import state_from_frame
                result = state_from_frame(raw, PROFILES[args.profile], Units(), 0).as_dict()
            show(result)
        elif args.action == "replay":
            show(replay(args.path, args.profile))
        elif args.action == "compare":
            show(compare_logs(args.stock_log, args.development_log))
        elif args.action == "calibrate":
            result = fit_angle(read_json(args.samples), axis=args.axis)
            if args.output:
                with open(args.output, "x", encoding="utf-8") as handle:
                    json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
            show(result)
        elif args.action == "speed-check":
            unit = Units(**read_json(args.calibration)["units"])
            show(compare_velocity(read_json(args.samples), unit, min_dt=args.min_dt,
                                  max_dt=args.max_dt, raw_noise_counts=args.raw_noise))
        else:
            run_device(args)
    except (ValueError, TypeError, KeyError, OSError, RuntimeError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
    except KeyboardInterrupt:
        print("Interrupted; inspect termination log. Mechanical stop is unconfirmed.", file=sys.stderr)
        raise SystemExit(130)
