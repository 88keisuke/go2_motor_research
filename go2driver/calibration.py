"""Least-squares angle calibration and timestamp-aware speed comparison."""
import math
import statistics
from .logging import distribution
from .model import Units
from .protocol import finite


def fit_angle(samples, *, axis="output"):
    if len(samples) < 3:
        raise ValueError("at least three raw/angle_rad pairs required; include return measurements")
    x = [finite(s["raw"]) for s in samples]
    y = [finite(s["angle_rad"]) for s in samples]
    mx, my = statistics.mean(x), statistics.mean(y)
    xx = sum((v-mx)**2 for v in x)
    if xx == 0:
        raise ValueError("raw measurements need a nonzero span")
    slope = sum((a-mx)*(b-my) for a,b in zip(x,y)) / xx
    if not math.isfinite(slope) or slope == 0:
        raise ValueError("angle slope is zero or non-finite")
    intercept = my-slope*mx
    residuals = [b-(slope*a+intercept) for a,b in zip(x,y)]
    repeat = {}
    for angle in sorted(set(y)):
        raw_group = [a for a,b in zip(x,y) if b == angle]
        if len(raw_group) > 1:
            repeat[str(angle)] = {"count": len(raw_group), "raw_span": max(raw_group)-min(raw_group),
                                  "angle_span_rad": abs(slope)*(max(raw_group)-min(raw_group))}
    leg_slopes = {}
    for leg in {s.get("leg") for s in samples} - {None}:
        group = [s for s in samples if s.get("leg") == leg]
        if len(group) >= 2:
            gx, gy = statistics.mean(s["raw"] for s in group), statistics.mean(s["angle_rad"] for s in group)
            variance = sum((s["raw"]-gx)**2 for s in group)
            if variance:
                leg_slopes[leg] = sum((s["raw"]-gx)*(s["angle_rad"]-gy) for s in group)/variance
    unit = Units(axis=axis, direction=1 if slope > 0 else -1, zero_raw=-intercept/slope,
                 angle_rad_per_count=abs(slope), angle_calibration_evidence="least-squares supplied known-angle pairs; see report residuals/repeatability")
    return {"units": unit.__dict__, "slope_rad_per_count": slope, "intercept_rad": intercept,
            "residuals_rad": residuals, "rms_residual_rad": math.sqrt(statistics.mean(r*r for r in residuals)),
            "max_abs_residual_rad": max(abs(r) for r in residuals), "repeatability": repeat,
            "leg_slopes_rad_per_count": leg_slopes,
            "warning": "fit is not hardware validation; no automatic pass threshold; collect both directions"}


def compare_velocity(samples, units, *, min_dt=0.01, max_dt=0.5, raw_noise_counts=1.0):
    if not units.angle_rad_per_count:
        raise ValueError("calibrate angle before velocity comparison")
    for value in (min_dt, max_dt, raw_noise_counts):
        finite(value)
    if min_dt <= 0 or max_dt < min_dt or raw_noise_counts < 0:
        raise ValueError("invalid interval/noise settings")
    result = []
    for a,b in zip(samples, samples[1:]):
        dt = finite(b["time_s"])-finite(a["time_s"])
        if dt <= 0:
            raise ValueError("timestamps must increase strictly within each motor")
        if not min_dt <= dt <= max_dt:
            result.append({"dt_s": dt, "excluded": "interval outside comparison window"})
            continue
        measured = (finite(b["raw"])-finite(a["raw"]))*units.angle_scale*units.direction/dt
        field = (finite(a["speed_raw"])+finite(b["speed_raw"]))/2*units.speed_scale*units.direction
        noise = math.sqrt(2)*raw_noise_counts*units.angle_scale/dt
        result.append({"midpoint_s": (a["time_s"]+b["time_s"])/2, "dt_s": dt,
                       "angle_derived_rad_s": measured, "field_average_rad_s": field,
                       "difference_rad_s": field-measured, "noise_estimate_rad_s": noise})
    return {"intervals": result, "difference": distribution([r["difference_rad_s"] for r in result if "difference_rad_s" in r]),
            "note": "two-point derivative amplifies position noise; endpoint field mean assumes smooth motion; acquisition latency remains"}
