"""As-flown verification against a stored plan.

A flown sortie is submitted as a time-stamped GNSS track (position, AMSL
altitude, remaining battery), photo trigger points and camera attitudes.
The submission is validated (CRS, time order, units, missing data), the
track is matched to the planned flight lines, and cross-track deviation,
actual GSD, exposure spacing, coverage union, no-fly-zone intrusion and
return-home margin are computed with the same terrain/camera/energy
models used for planning.

Findings are ordered by time so the caller sees where the sortie first
left the plan; uncovered areas, affected photos and re-flyable strips are
reported, and qualified segments can be locked to seed a reflight plan.
"""
from __future__ import annotations

import math
from datetime import datetime, timezone

from pyproj import CRS, Transformer
from pyproj.exceptions import CRSError
from shapely.affinity import rotate, translate
from shapely.geometry import LineString, Point, Polygon, mapping
from shapely.ops import unary_union

from .errors import ValidationError
from .planner import GAP_AREA_THRESHOLD_M2, NFZ_TOLERANCE_M, prepare

VERIFY_VERSION = "1.0.0"

ALT_RANGE_M = (-500.0, 9000.0)     # AMSL sanity range (unit check)
MAX_IMPLIED_SPEED_MPS = 150.0      # faster => position/time units are wrong
BATTERY_TOLERANCE_WH = 0.5         # BMS noise allowed around monotonicity
BATTERY_UNIT_HEADROOM = 1.2        # readings may exceed the nominal pack slightly
MAX_TRACK_POINTS = 50000
MAX_UNCOVERED = 100                # cap on reported uncovered patches
ATTITUDE_LIMIT_DEG = 60.0          # beyond this the attitude is not plausible
PHOTO_TIME_SLACK_S = 300.0         # trigger times must bracket the track


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _iso(epoch):
    return datetime.fromtimestamp(epoch, tz=timezone.utc).isoformat()


def _polygons(geom):
    if geom.is_empty:
        return []
    if geom.geom_type == "Polygon":
        return [geom]
    if geom.geom_type in ("MultiPolygon", "GeometryCollection"):
        out = []
        for g in geom.geoms:
            out.extend(_polygons(g))
        return out
    return []


# --------------------------------------------------------------------------
# request validation: CRS, time order, units, missing data
# --------------------------------------------------------------------------

def _parse_time(value, ctx, errors):
    if _is_num(value):
        return float(value)
    if isinstance(value, str):
        try:
            dt = datetime.fromisoformat(value.replace("Z", "+00:00"))
        except ValueError:
            dt = None
        if dt is not None:
            if dt.tzinfo is None:
                dt = dt.replace(tzinfo=timezone.utc)
            return dt.timestamp()
    errors.append(
        f"{ctx}: unparseable timestamp {value!r}; use ISO 8601 or epoch seconds"
    )
    return None


def _norm_pos(p, ctx, errors):
    pos = p.get("pos")
    if not (isinstance(pos, (list, tuple)) and len(pos) == 2 and all(_is_num(v) for v in pos)):
        label = "missing data" if pos is None else "must be [x, y] numbers"
        errors.append(f"{ctx}.pos: {label}")
        return None
    return [float(pos[0]), float(pos[1])]


def _norm_alt(p, ctx, errors):
    alt = p.get("alt_m")
    if not _is_num(alt):
        label = "missing data (metres AMSL)" if alt is None else "must be a number (metres AMSL)"
        errors.append(f"{ctx}.alt_m: {label}")
        return None
    if not (ALT_RANGE_M[0] <= float(alt) <= ALT_RANGE_M[1]):
        errors.append(
            f"{ctx}.alt_m: {alt} outside {ALT_RANGE_M[0]:g}..{ALT_RANGE_M[1]:g} m; "
            "check units (metres AMSL)"
        )
        return None
    return float(alt)


def _norm_battery(p, ctx, plan_battery, errors):
    bat = p.get("battery_wh")
    if not _is_num(bat):
        label = "missing data (Wh remaining)" if bat is None else "must be a number (Wh remaining)"
        errors.append(f"{ctx}.battery_wh: {label}")
        return None
    bat = float(bat)
    if bat < 0.0 or bat > plan_battery * BATTERY_UNIT_HEADROOM:
        errors.append(
            f"{ctx}.battery_wh: {bat:g} Wh outside 0..{plan_battery * BATTERY_UNIT_HEADROOM:.0f} Wh "
            f"(plan battery {plan_battery:g} Wh); check units (Wh, not %/mAh)"
        )
        return None
    return bat


def _norm_attitude(p, ctx, errors):
    att = p.get("attitude")
    if not isinstance(att, dict):
        errors.append(f"{ctx}.attitude: missing data (object with roll_deg/pitch_deg/yaw_deg)")
        return None
    out = {}
    ok = True
    for key, lo, hi in (
        ("roll_deg", -ATTITUDE_LIMIT_DEG, ATTITUDE_LIMIT_DEG),
        ("pitch_deg", -ATTITUDE_LIMIT_DEG, ATTITUDE_LIMIT_DEG),
        ("yaw_deg", -360.0, 360.0),
    ):
        v = att.get(key)
        if not _is_num(v):
            label = "missing data" if v is None else "must be a number (degrees)"
            errors.append(f"{ctx}.attitude.{key}: {label}")
            ok = False
        elif not (lo <= float(v) <= hi):
            errors.append(
                f"{ctx}.attitude.{key}: {v} outside {lo:g}..{hi:g} deg; check units"
            )
            ok = False
        else:
            out[key] = float(v)
    if not ok:
        return None
    out["yaw_deg"] %= 360.0
    return out


def _option(opts, key, default, lo, hi, errors):
    v = opts.get(key, default)
    if v is None:
        return None
    if not _is_num(v) or not (lo <= float(v) <= hi):
        errors.append(f"options.{key}: must be a number in [{lo:g}, {hi:g}]")
        return default
    return float(v)


def validate_flight_body(body, plan_req):
    """Structural/unit validation of an as-flown submission.

    ``plan_req`` is the stored plan request (provides the default CRS and
    the nominal battery for unit checks).  Returns the normalised track,
    photos and options; raises :class:`ValidationError` listing every
    problem found.
    """
    if not isinstance(body, dict):
        raise ValidationError(["request body must be a JSON object"])
    errors = []

    crs_in = body.get("crs", plan_req.get("crs", "EPSG:4326"))
    crs = None
    try:
        crs = CRS.from_user_input(crs_in)
    except (CRSError, TypeError, ValueError):
        errors.append(f"crs: cannot parse {crs_in!r} (use e.g. 'EPSG:4326')")
    if crs is not None and not crs.is_geographic:
        unit = ""
        if crs.axis_info:
            unit = (crs.axis_info[0].unit_name or "").lower()
        if unit not in ("metre", "meter", "m"):
            errors.append(
                f"crs: projected CRS must use metre units, got {unit or 'unknown'}"
            )

    plan_battery = float(plan_req.get("battery_wh", 0.0))

    raw_track = body.get("track")
    track = []
    if not isinstance(raw_track, list) or len(raw_track) < 2:
        errors.append("track: required list of at least 2 GNSS points")
    elif len(raw_track) > MAX_TRACK_POINTS:
        errors.append(f"track: {len(raw_track)} points exceeds the {MAX_TRACK_POINTS} limit")
    else:
        for i, p in enumerate(raw_track):
            ctx = f"track[{i}]"
            if not isinstance(p, dict):
                errors.append(f"{ctx}: must be an object with t/pos/alt_m/battery_wh")
                continue
            t = _parse_time(p.get("t"), f"{ctx}.t", errors)
            pos = _norm_pos(p, ctx, errors)
            alt = _norm_alt(p, ctx, errors)
            bat = _norm_battery(p, ctx, plan_battery, errors)
            if None in (t, pos, alt, bat):
                continue
            track.append({"i": i, "t": t, "pos": pos, "alt": alt, "battery": bat})
        for a, b in zip(track, track[1:]):
            if b["t"] <= a["t"]:
                errors.append(
                    f"track[{b['i']}].t: timestamps must be strictly increasing "
                    f"(time order broken after track[{a['i']}])"
                )
                break
        for a, b in zip(track, track[1:]):
            if b["battery"] > a["battery"] + BATTERY_TOLERANCE_WH:
                errors.append(
                    f"track[{b['i']}].battery_wh: remaining energy increases "
                    f"({a['battery']:g} -> {b['battery']:g} Wh); check units/order"
                )
                break

    raw_photos = body.get("photos")
    photos = []
    if not isinstance(raw_photos, list):
        errors.append("photos: required list of trigger records (may be empty)")
    else:
        seen_ids = set()
        for i, p in enumerate(raw_photos):
            ctx = f"photos[{i}]"
            if not isinstance(p, dict):
                errors.append(f"{ctx}: must be an object with t/pos/alt_m/attitude")
                continue
            pid = p.get("id")
            pid = str(pid) if pid is not None else f"P{i + 1:04d}"
            if pid in seen_ids:
                errors.append(f"{ctx}.id: duplicate photo id {pid!r}")
            seen_ids.add(pid)
            t = _parse_time(p.get("t"), f"{ctx}.t", errors)
            pos = _norm_pos(p, ctx, errors)
            alt = _norm_alt(p, ctx, errors)
            att = _norm_attitude(p, ctx, errors)
            if None in (t, pos, alt, att):
                continue
            photos.append({"i": i, "id": pid, "t": t, "pos": pos, "alt": alt, "att": att})
        for a, b in zip(photos, photos[1:]):
            if b["t"] < a["t"]:
                errors.append(f"photos[{b['i']}].t: trigger times must be non-decreasing")
                break
    if track and photos:
        t0, t1 = track[0]["t"], track[-1]["t"]
        outside = [
            p["id"] for p in photos
            if not (t0 - PHOTO_TIME_SLACK_S <= p["t"] <= t1 + PHOTO_TIME_SLACK_S)
        ]
        if outside:
            errors.append(
                "photos: trigger times outside the track time span "
                f"(e.g. {outside[0]}); check time units/base"
            )

    opts = body.get("options") or {}
    if not isinstance(opts, dict):
        errors.append("options: must be an object")
        opts = {}
    options = {
        "deviation_threshold_m": _option(opts, "deviation_threshold_m", None, 0.1, 1000.0, errors),
        "gsd_tolerance": _option(opts, "gsd_tolerance", 0.15, 0.0, 5.0, errors),
        "min_forward_overlap": _option(opts, "min_forward_overlap", None, 0.0, 0.95, errors),
        "max_tilt_deg": _option(opts, "max_tilt_deg", 5.0, 0.0, ATTITUDE_LIMIT_DEG, errors),
        "gap_area_threshold_m2": _option(
            opts, "gap_area_threshold_m2", GAP_AREA_THRESHOLD_M2, 0.01, 100000.0, errors
        ),
    }

    if errors:
        raise ValidationError(errors)
    return {"crs_obj": crs, "track": track, "photos": photos, "options": options}


# --------------------------------------------------------------------------
# verification
# --------------------------------------------------------------------------

def verify_flight(plan_req, plan_result, body):
    """Verify an as-flown sortie against a stored plan.

    Returns ``(result, geojson)``; raises :class:`ValidationError` for
    malformed submissions (400 at the API surface).
    """
    n = validate_flight_body(body, plan_req)
    mission = prepare(plan_req)
    fwd = Transformer.from_crs(n["crs_obj"], mission.crs.metric, always_xy=True)

    track = n["track"]
    for p in track:
        p["xy"] = fwd.transform(p["pos"][0], p["pos"][1])
    photos = n["photos"]
    for p in photos:
        p["xy"] = fwd.transform(p["pos"][0], p["pos"][1])

    # unit/datum checks that need metric distances and terrain
    errors = []
    for a, b in zip(track, track[1:]):
        dt = b["t"] - a["t"]
        speed = math.hypot(b["xy"][0] - a["xy"][0], b["xy"][1] - a["xy"][1]) / dt
        if speed > MAX_IMPLIED_SPEED_MPS:
            errors.append(
                f"track[{a['i']}]..track[{b['i']}]: implied speed {speed:.0f} m/s "
                f"exceeds {MAX_IMPLIED_SPEED_MPS:g} m/s; check position/timestamp units"
            )
            break
    missing, datum = [], []
    for p in track:
        z = mission.terrain.height(*p["xy"])
        if z is None:
            missing.append(f"track[{p['i']}]")
        else:
            p["z"] = z
            if p["alt"] < z - 50.0:
                datum.append(f"track[{p['i']}] (alt {p['alt']:.0f} m vs terrain {z:.0f} m)")
    for p in photos:
        z = mission.terrain.height(*p["xy"])
        if z is None:
            missing.append(f"photo {p['id']}")
        else:
            p["z"] = z
            if p["alt"] < z - 50.0:
                datum.append(f"photo {p['id']} (alt {p['alt']:.0f} m vs terrain {z:.0f} m)")
    if missing:
        errors.append(
            "elevation gap: no terrain data at " + ", ".join(missing[:5])
            + (" ..." if len(missing) > 5 else "")
        )
    if datum:
        errors.append(
            "altitude more than 50 m below terrain at " + ", ".join(datum[:3])
            + "; check altitude datum/units (metres AMSL)"
        )
    if errors:
        raise ValidationError(errors)

    # planned segments in the metric CRS
    segs = []
    for s in plan_result.get("segments", []):
        a = mission.crs.fwd.transform(*s["start"])
        b = mission.crs.fwd.transform(*s["end"])
        segs.append({
            "id": s["id"],
            "order": s["order"],
            "line": LineString([a, b]),
            "length": float(s["length_m"]),
            "gsd_max": float(s["gsd_cm_max"]),
            "points": [],  # matched track points: (t, cross-track deviation, along-line m)
        })
    seg_order = {s["id"]: s["order"] for s in segs}

    opts = n["options"]
    dev_thr = opts["deviation_threshold_m"]
    if dev_thr is None:
        dev_thr = max(10.0, 0.35 * mission.line_spacing)
    assign_tol = max(2.0 * dev_thr, 1.5 * mission.line_spacing)

    def nearest_seg(xy):
        pt = Point(xy)
        best, best_d = None, None
        for s in segs:
            d = s["line"].distance(pt)
            if best_d is None or d < best_d:
                best, best_d = s, d
        if best is None or best_d > assign_tol:
            return None, None, None
        return best, best_d, best["line"].project(pt)

    # --- track: matching, deviation, return margin, NFZ intrusion ---------
    home = mission.home
    unmatched = 0
    min_agl = None
    min_margin = None
    margin_event = None
    nfz_event = None
    nfz_intrusion_m = 0.0
    track_dist = 0.0
    dev_events = {}  # first exceedance per segment

    prev = None
    for p in track:
        x, y = p["xy"]
        agl = p["alt"] - p["z"]
        min_agl = agl if min_agl is None else min(min_agl, agl)
        seg, dev, along = nearest_seg((x, y))
        p["seg"] = seg["id"] if seg else None
        if seg is None:
            unmatched += 1
        else:
            seg["points"].append((p["t"], dev, along))
            if dev > dev_thr and seg["id"] not in dev_events:
                dev_events[seg["id"]] = {
                    "type": "cross_track_deviation",
                    "t": p["t"],
                    "segment_id": seg["id"],
                    "point": Point(x, y),
                    "message": f"track leaves {seg['id']} by {dev:.1f} m "
                               f"(threshold {dev_thr:.1f} m)",
                }
        dist_home = math.hypot(x - home.x, y - home.y)
        ret = mission.energy.leg(
            dist_home, mission.speed, max(0.0, mission.home_z - p["alt"])
        )
        margin = p["battery"] - ret - mission.reserve_wh
        if min_margin is None or margin < min_margin:
            min_margin = margin
        if margin < 0.0 and margin_event is None:
            margin_event = {
                "type": "return_margin_low",
                "t": p["t"],
                "point": Point(x, y),
                "message": f"remaining {p['battery']:.1f} Wh cannot cover the return "
                           f"leg ({ret:.1f} Wh) plus reserve ({mission.reserve_wh:.1f} Wh)",
            }
        if prev is not None:
            leg = math.hypot(x - prev["xy"][0], y - prev["xy"][1])
            track_dist += leg
            if leg > 1e-6 and not mission.nfz.is_empty:
                link = LineString([prev["xy"], (x, y)])
                intr = link.intersection(mission.nfz).length
                nfz_intrusion_m += intr
                if intr > NFZ_TOLERANCE_M and nfz_event is None:
                    nfz_event = {
                        "type": "nfz_intrusion",
                        "t": p["t"],
                        "point": link.interpolate(0.5, normalized=True),
                        "message": f"flown track crosses a no-fly zone for {intr:.1f} m",
                    }
        prev = p

    # --- photos: GSD, tilt, footprints, exposure spacing ------------------
    cam = mission.camera
    target_gsd = float(plan_req.get("target_gsd_cm"))
    min_fwd = opts["min_forward_overlap"]
    if min_fwd is None:
        min_fwd = float(plan_req.get("forward_overlap", 0.0))
    max_tilt = opts["max_tilt_deg"]
    # GSD is judged against what the plan promised per segment (constant-AMSL
    # segments legitimately exceed the target GSD over lower terrain)
    plan_gsd_range = (plan_result.get("metrics") or {}).get("gsd_cm_range") or []
    plan_gsd_max = float(plan_gsd_range[1]) if len(plan_gsd_range) == 2 else target_gsd

    for p in photos:
        agl = p["alt"] - p["z"]
        p["agl"] = agl
        p["gsd"] = cam.gsd(agl) * 100.0 if agl > 0 else float("inf")
        w, l = cam.footprint(max(agl, 0.1))
        p["fp_len"] = l
        rect = Polygon([
            (-w / 2.0, -l / 2.0), (w / 2.0, -l / 2.0),
            (w / 2.0, l / 2.0), (-w / 2.0, l / 2.0),
        ])
        fp = translate(
            rotate(rect, -p["att"]["yaw_deg"], origin=(0.0, 0.0)),
            xoff=p["xy"][0], yoff=p["xy"][1],
        )
        p["footprint"] = fp
        seg, _, along = nearest_seg(p["xy"])
        p["seg"] = seg["id"] if seg else None
        p["along"] = along
        gsd_limit = (seg["gsd_max"] if seg else plan_gsd_max) * (1.0 + opts["gsd_tolerance"])
        issues = []
        if agl <= 0.5:
            issues.append("below_terrain")
        if p["gsd"] > gsd_limit:
            issues.append("gsd_breach")
        if abs(p["att"]["roll_deg"]) > max_tilt or abs(p["att"]["pitch_deg"]) > max_tilt:
            issues.append("tilt_exceeded")
        if not mission.nfz.is_empty and fp.intersection(mission.nfz).area > 0.5:
            issues.append("nfz_intrusion")
        p["issues"] = issues

    ordered = sorted(photos, key=lambda p: (p["t"], p["i"]))
    spacing = []
    fwd_overlap_min = None
    gap_events = []
    for a, b in zip(ordered, ordered[1:]):
        if a["seg"] is None or a["seg"] != b["seg"]:
            continue
        d = abs(b["along"] - a["along"])
        spacing.append(d)
        half = (a["fp_len"] + b["fp_len"]) / 2.0
        ratio = (half - d) / half if half > 0 else 0.0
        fwd_overlap_min = ratio if fwd_overlap_min is None else min(fwd_overlap_min, ratio)
        if ratio < 0.0:
            gap_events.append({
                "type": "exposure_gap",
                "t": b["t"],
                "segment_id": a["seg"],
                "photo_id": b["id"],
                "point": Point((a["xy"][0] + b["xy"][0]) / 2.0,
                               (a["xy"][1] + b["xy"][1]) / 2.0),
                "message": f"exposure spacing {d:.1f} m exceeds the footprint length "
                           f"{half:.1f} m between {a['id']} and {b['id']}",
            })
            b["issues"].append("exposure_gap")
        elif ratio < min_fwd:
            b["issues"].append("low_forward_overlap")

    # --- coverage union and uncovered patches -----------------------------
    if photos:
        union = unary_union([p["footprint"] for p in photos])
        covered = union.intersection(mission.buildable).area
        coverage_pct = 100.0 * covered / mission.buildable.area if mission.buildable.area > 0 else 0.0
        leftover = mission.buildable.difference(union)
    else:
        coverage_pct = 0.0
        leftover = mission.buildable
    gap_thr = opts["gap_area_threshold_m2"]
    uncovered = []
    for g in _polygons(leftover):
        if g.area <= gap_thr:
            continue
        c = g.centroid
        nearest = min(segs, key=lambda s: s["line"].distance(c), default=None)
        uncovered.append({
            "area_m2": round(g.area, 1),
            "centroid": list(mission.crs.inv.transform(c.x, c.y)),
            "nearest_segment": nearest["id"] if nearest else None,
            "geom": g,
        })
    uncovered.sort(key=lambda u: -u["area_m2"])
    uncovered = uncovered[:MAX_UNCOVERED]

    # --- per-segment flown status -----------------------------------------
    buf = max(2.0, mission.photo_spacing / 2.0)
    seg_reports = []
    refly = {}

    def flag_refly(seg_id, reason):
        entry = refly.setdefault(seg_id, {"id": seg_id, "reasons": []})
        if reason not in entry["reasons"]:
            entry["reasons"].append(reason)

    for s in segs:
        pts = s["points"]
        if pts:
            devs = [d for _, d, _ in pts]
            alongs = sorted(a for _, _, a in pts)
            covered_len, cur_lo, cur_hi = 0.0, None, None
            for a in alongs:
                lo, hi = max(0.0, a - buf), min(s["length"], a + buf)
                if cur_lo is None:
                    cur_lo, cur_hi = lo, hi
                elif lo <= cur_hi:
                    cur_hi = max(cur_hi, hi)
                else:
                    covered_len += cur_hi - cur_lo
                    cur_lo, cur_hi = lo, hi
            covered_len += cur_hi - cur_lo
            frac = covered_len / s["length"] if s["length"] > 0 else 0.0
            status = "flown" if frac >= 0.9 else ("partial" if frac >= 0.3 else "not_flown")
            seg_reports.append({
                "id": s["id"],
                "status": status,
                "flown_fraction": round(frac, 3),
                "matched_points": len(pts),
                "cross_track_max_m": round(max(devs), 2),
                "cross_track_mean_m": round(sum(devs) / len(devs), 2),
            })
        else:
            status = "not_flown"
            seg_reports.append({
                "id": s["id"],
                "status": status,
                "flown_fraction": 0.0,
                "matched_points": 0,
                "cross_track_max_m": None,
                "cross_track_mean_m": None,
            })
        if status == "not_flown":
            flag_refly(s["id"], "not_flown")
        elif status == "partial":
            flag_refly(s["id"], "partial_track")
    for u in uncovered:
        if u["nearest_segment"]:
            flag_refly(u["nearest_segment"], "uncovered_area")
    for e in gap_events:
        flag_refly(e["segment_id"], "exposure_gap")
    refly_list = sorted(refly.values(), key=lambda r: seg_order.get(r["id"], 0))

    # --- events, affected photos, metrics ---------------------------------
    events = list(dev_events.values()) + gap_events
    if nfz_event:
        events.append(nfz_event)
    if margin_event:
        events.append(margin_event)
    events.sort(key=lambda e: e["t"])
    out_events = []
    for e in events:
        ex, ey = mission.crs.inv.transform(e["point"].x, e["point"].y)
        oe = {"type": e["type"], "t": _iso(e["t"]), "point": [ex, ey],
              "message": e["message"]}
        if e.get("segment_id"):
            oe["segment_id"] = e["segment_id"]
        if e.get("photo_id"):
            oe["photo_id"] = e["photo_id"]
        out_events.append(oe)

    affected = []
    for p in ordered:
        if not p["issues"]:
            continue
        affected.append({
            "id": p["id"],
            "t": _iso(p["t"]),
            "pos": p["pos"],
            "segment_id": p["seg"],
            "agl_m": round(p["agl"], 1),
            "gsd_cm": round(p["gsd"], 2) if math.isfinite(p["gsd"]) else None,
            "issues": p["issues"],
        })

    devs_all = [d for s in segs for _, d, _ in s["points"]]
    gsds = [p["gsd"] for p in photos if math.isfinite(p["gsd"])]
    actual_energy = track[0]["battery"] - track[-1]["battery"]
    gsd_breaches = sum(1 for p in photos if "gsd_breach" in p["issues"])
    tilt_events = sum(1 for p in photos if "tilt_exceeded" in p["issues"])
    metrics = {
        "track_points": len(track),
        "photo_count": len(photos),
        "duration_s": round(track[-1]["t"] - track[0]["t"], 1),
        "track_distance_m": round(track_dist, 1),
        "actual_energy_wh": round(actual_energy, 2),
        "coverage_pct": round(coverage_pct, 2),
        "uncovered_area_m2": round(sum(u["area_m2"] for u in uncovered), 1),
        "uncovered_count": len(uncovered),
        "cross_track_max_m": round(max(devs_all), 2) if devs_all else None,
        "cross_track_mean_m": (
            round(sum(devs_all) / len(devs_all), 2) if devs_all else None
        ),
        "deviation_threshold_m": round(dev_thr, 2),
        "points_beyond_threshold": sum(1 for d in devs_all if d > dev_thr),
        "unmatched_track_points": unmatched,
        "gsd_cm_min": round(min(gsds), 2) if gsds else None,
        "gsd_cm_max": round(max(gsds), 2) if gsds else None,
        "gsd_cm_mean": round(sum(gsds) / len(gsds), 2) if gsds else None,
        "gsd_breach_count": gsd_breaches,
        "exposure_spacing_m": (
            {
                "min": round(min(spacing), 1),
                "max": round(max(spacing), 1),
                "mean": round(sum(spacing) / len(spacing), 1),
            }
            if spacing else None
        ),
        "forward_overlap_min": (
            round(fwd_overlap_min, 3) if fwd_overlap_min is not None else None
        ),
        "exposure_gaps": len(gap_events),
        "nfz_intrusion_m": round(nfz_intrusion_m, 1),
        "return_margin_min_wh": round(min_margin, 1),
        "return_margin_ok": min_margin >= 0.0,
        "min_agl_m": round(min_agl, 1),
        "segments_flown": sum(1 for r in seg_reports if r["status"] == "flown"),
        "segments_partial": sum(1 for r in seg_reports if r["status"] == "partial"),
        "segments_not_flown": sum(1 for r in seg_reports if r["status"] == "not_flown"),
        "affected_photo_count": len(affected),
    }

    pm = plan_result.get("metrics", {})
    comparison = {
        "coverage_pct": {
            "planned": pm.get("coverage_pct"),
            "actual": metrics["coverage_pct"],
            "delta": round(metrics["coverage_pct"] - (pm.get("coverage_pct") or 0.0), 2),
        },
        "distance_m": {
            "planned": pm.get("total_distance_m"),
            "actual": metrics["track_distance_m"],
            "delta": round(metrics["track_distance_m"] - (pm.get("total_distance_m") or 0.0), 1),
        },
        "energy_wh": {
            "planned": pm.get("total_energy_wh"),
            "actual": metrics["actual_energy_wh"],
            "delta": round(metrics["actual_energy_wh"] - (pm.get("total_energy_wh") or 0.0), 2),
        },
        "photo_count": {
            "planned": pm.get("photo_count"),
            "actual": metrics["photo_count"],
            "delta": metrics["photo_count"] - (pm.get("photo_count") or 0),
        },
        "risk": {
            "planned": pm.get("risk"),
            "actual": {
                "cross_track_deviations": len(dev_events),
                "exposure_gaps": len(gap_events),
                "nfz_intrusions": 1 if nfz_event else 0,
                "return_margin_events": 1 if margin_event else 0,
                "gsd_breaches": gsd_breaches,
                "tilt_events": tilt_events,
            },
        },
    }

    unmatched_photos = sum(1 for p in photos if p["seg"] is None)
    warnings = []
    if unmatched:
        warnings.append(
            f"{unmatched} track points are farther than {assign_tol:.0f} m from any "
            "planned line (transit/turns); excluded from deviation statistics"
        )
    if not photos:
        warnings.append("no photo triggers submitted; coverage is zero")
    if unmatched_photos:
        warnings.append(
            f"{unmatched_photos} photos could not be matched to a planned line; "
            "excluded from exposure-spacing checks"
        )

    result = {
        "rules_version": VERIFY_VERSION,
        "events": out_events,
        "first_event": out_events[0] if out_events else None,
        "segments": seg_reports,
        "uncovered_areas": [
            {k: u[k] for k in ("area_m2", "centroid", "nearest_segment")}
            for u in uncovered
        ],
        "affected_photos": affected,
        "refly_segments": refly_list,
        "comparison": comparison,
        "metrics": metrics,
        "warnings": warnings,
    }

    # --- GeoJSON report (plan source CRS) ----------------------------------
    features = []

    def add(geom, props):
        features.append({
            "type": "Feature",
            "properties": props,
            "geometry": mapping(mission.crs.to_source(geom)),
        })

    add(mission.survey, {"feature": "survey_area"})
    for i, g in enumerate(_polygons(mission.nfz)):
        add(g, {"feature": "no_fly_zone", "index": i})
    status_by_id = {r["id"]: r["status"] for r in seg_reports}
    for s in segs:
        add(s["line"], {
            "feature": "planned_segment", "id": s["id"], "status": status_by_id[s["id"]],
        })
    add(LineString([p["xy"] for p in track]), {"feature": "track"})
    for p in ordered:
        add(Point(p["xy"]), {
            "feature": "photo",
            "id": p["id"],
            "affected": bool(p["issues"]),
            "gsd_cm": round(p["gsd"], 2) if math.isfinite(p["gsd"]) else None,
        })
        if p["issues"]:
            add(p["footprint"], {
                "feature": "photo_footprint", "id": p["id"], "issues": p["issues"],
            })
    for u in uncovered:
        add(u["geom"], {
            "feature": "uncovered_area",
            "area_m2": u["area_m2"],
            "nearest_segment": u["nearest_segment"],
        })
    for e in events:
        add(e["point"], {"feature": "event", "type": e["type"], "message": e["message"]})
    add(home, {"feature": "home"})
    geojson = {"type": "FeatureCollection", "features": features}
    return result, geojson
