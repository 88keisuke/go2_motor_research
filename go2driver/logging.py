"""Append-only event JSONL and offline replay/comparison; no device access."""
from datetime import datetime, timezone
import json
import statistics
import time
from .model import Units, state_from_frame
from .protocol import PROFILES, FrameStream, checked_id, decode, finite


class EventLog:
    def __init__(self, path=None):
        self.file = open(path, "x", encoding="utf-8") if path else None
        self.events = []

    def emit(self, event, **fields):
        record = {"schema": 1, "event": event, "monotonic": time.monotonic(),
                  "utc": datetime.now(timezone.utc).isoformat(), **fields}
        # Live runs stream to disk. In-memory collection is only for virtual tests.
        if self.file:
            self.file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            self.file.flush()
        else:
            self.events.append(record)
        return record

    def close(self):
        if self.file:
            self.file.close()


def distribution(values):
    return {"count": len(values), "mean": statistics.mean(values) if values else None,
            "stddev": statistics.pstdev(values) if values else None,
            "min": min(values) if values else None, "max": max(values) if values else None}


def timing_summary(events):
    per_id = {}
    for event in events:
        if event["event"] != "transaction":
            continue
        item = per_id.setdefault(str(event["motor_id"]), {"times": [], "motion_times": [], "latencies": [], "missing": 0})
        if event["ok"]:
            item["times"].append(event["received_at"])
            if event.get("command_mode") == 1:
                item["motion_times"].append(event["received_at"])
            item["latencies"].append(event["latency_s"])
        else:
            item["missing"] += 1
    return {key: {"update_interval_s": distribution([b-a for a,b in zip(v["times"], v["times"][1:])]),
                  "motion_update_interval_s": distribution([b-a for a,b in zip(v["motion_times"], v["motion_times"][1:])]),
                  "latency_s": distribution(v["latencies"]), "missing_replies": v["missing"]}
            for key, v in per_id.items()}


def replay(path, profile_name=None, units=None):
    """Replay tx and arbitrary rx_chunk records. Session config is used by default.

    profile_name overrides CRC interpretation for development/stock comparison.
    Synthetic capture logs have no timestamps, so no invented timing is reported.
    """
    events, results = [], []
    profile = PROFILES[profile_name or "stock"]
    unit_map = {}
    stream = FrameStream(profile)
    saw_chunks = False
    with open(path, encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, 1):
            try:
                event = json.loads(line)
                if not isinstance(event, dict) or not isinstance(event.get("event"), str):
                    raise ValueError("each log record must be an object with an event name")
                kind = event["event"]
                if kind == "transaction":
                    checked_id(event["motor_id"])
                    if type(event["ok"]) is not bool:
                        raise ValueError("transaction ok must be boolean")
                    if event["ok"]:
                        finite(event["received_at"])
                        finite(event["latency_s"])
                if kind == "cycle":
                    finite(event["skew_s"])
                if kind == "session":
                    if not profile_name:
                        profile = PROFILES[event["profile"]]
                    unit_map = {int(k): Units(**v) for k,v in event.get("units", {}).items()}
                    stream = FrameStream(profile)
                    saw_chunks = False
                if kind == "tx":
                    stream.reset()  # transaction boundaries, never concatenate stale partial replies
                    raw = decode(bytes.fromhex(event["hex"]), profile, "tx")
                    results.append({"line": line_no, "direction": "tx", "raw": raw})
                if kind == "rx_chunk":
                    saw_chunks = True
                    before = stream.crc_errors
                    frames = stream.feed(bytes.fromhex(event["hex"]))
                    if stream.crc_errors > before:
                        results.append({"line": line_no, "crc_errors": stream.crc_errors-before})
                elif kind == "rx" and not saw_chunks:
                    frames = [bytes.fromhex(event["hex"])]
                else:
                    frames = []
                for frame in frames:
                    motor_id = frame[2] & 15
                    unit = units or unit_map.get(motor_id, Units())
                    state = state_from_frame(frame, profile, unit, event.get("monotonic", 0))
                    results.append({"line": line_no, "direction": "rx", "state": state.as_dict()})
                events.append(event)
            except (ValueError, KeyError, TypeError) as exc:
                results.append({"line": line_no, "error": str(exc)})
    return {"profile": profile.name, "records": results, "timing": timing_summary(events),
            "cycle_skew_s": distribution([e["skew_s"] for e in events if e["event"] == "cycle"]),
            "noise_bytes": stream.noise_bytes,
            "note": "CRC validity does not validate physical units; host timings are not internal control periods"}


def compare_logs(stock_path, development_path):
    reports = {"stock": replay(stock_path, "stock"), "development": replay(development_path, "development")}
    for name, report in reports.items():
        states = [r["state"] for r in report["records"] if "state" in r]
        report["raw_distribution"] = {field: distribution([s["raw"][field] for s in states])
                                       for field in ("position", "speed", "torque", "temperature")}
    return {"logs": reports, "interpretation": "Separate runs; distributions are descriptive, not proof of equal units, gains or performance"}
