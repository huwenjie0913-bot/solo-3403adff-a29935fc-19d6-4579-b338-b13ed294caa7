"""Core mission planning.

Pipeline: validate -> reproject to a metric CRS -> clip the survey area by
the no-fly zones -> sweep boustrophedon lines at the spacing implied by the
target GSD and side overlap -> fly each segment at a constant AMSL height
that clears the highest terrain sample by the required AGL height -> derive
per-segment footprint/GSD/overlap/energy -> accumulate turns, ferry and
return-to-home legs, checking the battery and reserve at every segment.

Reported events (coverage_gap / nfz_intrusion / battery_low) are ordered by
flight sequence so the caller sees the *first* failure that would occur.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from shapely.affinity import rotate
from shapely.geometry import GeometryCollection, LineString, Point, mapping
from shapely.ops import unary_union

from . import ALGO_VERSION
from .camera import Camera
from .energy import EnergyModel
from .errors import ValidationError
from .geo import build_crs_context
from .terrain import TerrainGrid
from .validation import validate_request

MIN_SEGMENT_M = 0.5          # shorter slivers from polygon clipping are dropped
GAP_AREA_THRESHOLD_M2 = 1.0  # smaller uncovered patches are ignored
NFZ_TOLERANCE_M = 0.5        # intrusion shorter than this is treated as zero


# --------------------------------------------------------------------------
# preparation
# --------------------------------------------------------------------------

@dataclass
class Mission:
    crs: object
    survey: object
    nfz: object
    buildable: object
    terrain: TerrainGrid
    camera: Camera
    home: Point
    home_z: float
    heading: float
    h_agl: float
    footprint_w: float
    footprint_l: float
    line_spacing: float
    photo_spacing: float
    speed: float
    turn_radius: float
    battery_wh: float
    reserve_wh: float
    energy: EnergyModel


def prepare(req) -> Mission:
    n = validate_request(req)
    crs_ctx = build_crs_context(n["crs_obj"], n["survey_area"])
    survey = crs_ctx.to_metric(n["survey_area"])
    nfz_geoms = [crs_ctx.to_metric(g) for g in n["no_fly_zones"]]
    nfz = unary_union(nfz_geoms) if nfz_geoms else GeometryCollection()
    terrain = TerrainGrid.from_request(n["terrain"], crs_ctx)

    errors = []
    if not terrain.covers_bounds(survey.bounds):
        errors.append("elevation gap: terrain grid extent does not cover the survey area")
    else:
        missing = terrain.missing_within(survey)
        if missing:
            errors.append(
                f"elevation gap: terrain cells without data inside the survey area "
                f"(e.g. near {missing[0]})"
            )
    hx, hy = crs_ctx.fwd.transform(*n["home"])
    home_z = terrain.height(hx, hy)
    if home_z is None:
        errors.append("elevation gap: no terrain data at the home point")
    if errors:
        raise ValidationError(errors)

    buildable = survey if nfz.is_empty else survey.difference(nfz)
    if buildable.is_empty or buildable.area <= 0:
        raise ValidationError(["survey area is fully covered by no-fly zones"])

    cam = Camera(**n["camera"])
    gsd_m = n["target_gsd_cm"] / 100.0
    h_agl = cam.height_for_gsd(gsd_m)
    fw, fl = cam.footprint(h_agl)
    opts = n["options"]
    return Mission(
        crs=crs_ctx,
        survey=survey,
        nfz=nfz,
        buildable=buildable,
        terrain=terrain,
        camera=cam,
        home=Point(hx, hy),
        home_z=home_z,
        heading=n["heading_deg"] % 180.0,
        h_agl=h_agl,
        footprint_w=fw,
        footprint_l=fl,
        line_spacing=fw * (1.0 - n["side_overlap"]),
        photo_spacing=fl * (1.0 - n["forward_overlap"]),
        speed=n["speed_mps"],
        turn_radius=n["turn_radius_m"],
        battery_wh=n["battery_wh"],
        reserve_wh=n["reserve_wh"],
        energy=EnergyModel(opts["cruise_power_w"], opts["climb_wh_per_m"]),
    )


# --------------------------------------------------------------------------
# segment records
# --------------------------------------------------------------------------

@dataclass
class Record:
    line: LineString          # metric CRS
    alt: float                # AMSL flight height of the segment
    agl: float                # height above the highest terrain in the segment
    width: float              # minimum footprint width along the segment
    length: float
    leg_energy: float         # energy of the segment itself (Wh)
    photos: int
    gsd_min: float            # cm/px at the highest terrain
    gsd_max: float            # cm/px at the lowest terrain
    fwd_overlap_min: float | None
    nfz_intrusion: float
    frozen: bool = False
    fwd_gaps: list = field(default_factory=list)
    # filled in by finalize()
    order: int = 0
    seg_id: str = ""
    energy: float = 0.0       # leg energy + connector energy
    cumulative: float = 0.0
    remaining: float = 0.0
    return_energy: float = 0.0
    side_overlap_min: float | None = None


def _perp_distance(line_a, line_b):
    """Distance from line_b's start to the infinite line through line_a."""
    (x1, y1), (x2, y2) = line_a.coords[0], line_a.coords[-1]
    qx, qy = line_b.coords[0]
    dx, dy = x2 - x1, y2 - y1
    norm = math.hypot(dx, dy)
    if norm == 0:
        return math.hypot(qx - x1, qy - y1)
    return abs(dx * (y1 - qy) - (x1 - qx) * dy) / norm


def _lines(geom):
    if geom.is_empty:
        return []
    if geom.geom_type == "LineString":
        return [geom]
    if geom.geom_type in ("MultiLineString", "GeometryCollection"):
        out = []
        for g in geom.geoms:
            out.extend(_lines(g))
        return out
    return []


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


def _terrain_or(mission, x, y, fallback):
    z = mission.terrain.height(x, y)
    return fallback if z is None else z


def make_record(mission, line) -> Record:
    length = line.length
    step = max(2.0, min(mission.photo_spacing, mission.footprint_w) / 2.0)
    n = max(2, int(length // step) + 1)
    zs = [
        _terrain_or(mission, p.x, p.y, 0.0)
        for p in (line.interpolate(float(d)) for d in np.linspace(0.0, length, n))
    ]
    zmax, zmin = max(zs), min(zs)
    alt = zmax + mission.h_agl  # constant AMSL over the segment

    # exposure stations along the segment
    ps = mission.photo_spacing
    pos = list(np.arange(0.0, length, ps))
    if not pos:
        pos = [0.0]
    if length - pos[-1] > 0.25 * ps:
        pos.append(length)
    if len(pos) == 1:
        pos.append(length)

    cam = mission.camera
    pzs = []
    for d in pos:
        p = line.interpolate(d)
        pzs.append(_terrain_or(mission, p.x, p.y, zmax))
    fp_len = [(alt - z) * cam.sensor_height_mm / cam.focal_length_mm for z in pzs]
    widths = [(alt - z) * cam.sensor_width_mm / cam.focal_length_mm for z in pzs]

    fwd_min = None
    fwd_gaps = []
    for i in range(len(pos) - 1):
        d = pos[i + 1] - pos[i]
        half = (fp_len[i] + fp_len[i + 1]) / 2.0
        overlap = half - d
        ratio = overlap / half if half > 0 else 0.0
        fwd_min = ratio if fwd_min is None else min(fwd_min, ratio)
        if overlap < 0:
            fwd_gaps.append(line.interpolate((pos[i] + pos[i + 1]) / 2.0))

    nfz_intrusion = 0.0
    if not mission.nfz.is_empty:
        nfz_intrusion = line.intersection(mission.nfz).length

    return Record(
        line=line,
        alt=alt,
        agl=alt - zmax,
        width=min(widths),
        length=length,
        leg_energy=mission.energy.leg(length, mission.speed, 0.0),
        photos=len(pos),
        gsd_min=cam.gsd(alt - zmax) * 100.0,
        gsd_max=cam.gsd(alt - zmin) * 100.0,
        fwd_overlap_min=fwd_min,
        nfz_intrusion=nfz_intrusion,
        fwd_gaps=fwd_gaps,
    )


def generate_records(mission, area) -> list:
    """Boustrophedon sweep over ``area`` (metric CRS)."""
    if area.is_empty:
        return []
    origin = mission.survey.centroid
    rot = rotate(area, -mission.heading, origin=origin)
    minx, miny, maxx, maxy = rot.bounds
    records = []
    y = miny + mission.line_spacing / 2.0
    row = 0
    while y <= maxy + 1e-9:
        sweep = LineString([(minx - 1.0, y), (maxx + 1.0, y)])
        parts = [ln for ln in _lines(rot.intersection(sweep)) if ln.length >= MIN_SEGMENT_M]
        parts.sort(key=lambda ln: ln.bounds[0], reverse=(row % 2 == 1))
        for ln in parts:
            coords = list(ln.coords)
            if row % 2 == 1:
                coords = coords[::-1]
            line = rotate(LineString(coords), mission.heading, origin=origin)
            records.append(make_record(mission, line))
        y += mission.line_spacing
        row += 1
    return records


def record_from_spec(mission, spec) -> Record:
    """Rebuild a frozen segment from its serialized form."""
    line = mission.crs.to_metric(LineString([spec["start"], spec["end"]]))
    return Record(
        line=line,
        alt=float(spec["height_amsl_m"]),
        agl=float(spec["height_agl_m"]),
        width=float(spec["footprint_width_m"]),
        length=line.length,
        leg_energy=float(spec["leg_energy_wh"]),
        photos=int(spec["photo_count"]),
        gsd_min=float(spec["gsd_cm_min"]),
        gsd_max=float(spec["gsd_cm_max"]),
        fwd_overlap_min=spec.get("forward_overlap_min"),
        nfz_intrusion=float(spec["nfz_intrusion_m"]),
        frozen=True,
    )


# --------------------------------------------------------------------------
# finalize: sequencing, energy, battery, coverage, events
# --------------------------------------------------------------------------

def finalize(mission, records):
    events = []
    warnings = []
    speed = mission.speed
    r_turn = mission.turn_radius
    home = mission.home

    prev_pt, prev_alt, prev_home = home, mission.home_z, True
    cumulative = 0.0
    connector_dist = 0.0
    battery_flagged = False
    turn_warned = False

    for idx, rec in enumerate(records):
        rec.order = idx
        rec.seg_id = f"S{idx + 1:03d}"
        start = Point(rec.line.coords[0])
        end = Point(rec.line.coords[-1])
        d = prev_pt.distance(start)

        if prev_home or r_turn == 0:
            conn = d
        elif d >= 2.0 * r_turn:
            conn = math.pi * r_turn + (d - 2.0 * r_turn)
        else:
            conn = math.pi * r_turn
            if not turn_warned:
                warnings.append(
                    f"turn radius {r_turn:g} m exceeds half the line spacing; "
                    "turns will overshoot into adjacent swaths"
                )
                turn_warned = True
        connector_dist += conn
        rec.energy = rec.leg_energy + mission.energy.leg(
            conn, speed, max(0.0, rec.alt - prev_alt)
        )
        cumulative += rec.energy
        rec.cumulative = cumulative
        rec.remaining = mission.battery_wh - cumulative

        if d > 1e-6 and not mission.nfz.is_empty:
            link = LineString([prev_pt, start])
            intr = link.intersection(mission.nfz).length
            if intr > NFZ_TOLERANCE_M:
                events.append({
                    "type": "nfz_intrusion",
                    "order": idx,
                    "point": link.interpolate(0.5, normalized=True),
                    "message": f"connector into {rec.seg_id} crosses a no-fly zone "
                               f"for {intr:.1f} m",
                })
        if rec.nfz_intrusion > NFZ_TOLERANCE_M:
            events.append({
                "type": "nfz_intrusion",
                "order": idx,
                "point": rec.line.interpolate(0.5, normalized=True),
                "message": f"{rec.seg_id} crosses a no-fly zone for "
                           f"{rec.nfz_intrusion:.1f} m",
            })
        for g in rec.fwd_gaps:
            events.append({
                "type": "coverage_gap",
                "order": idx,
                "point": g,
                "message": f"forward overlap lost on {rec.seg_id} "
                           "(terrain rises between exposures)",
            })

        rec.return_energy = mission.energy.leg(
            end.distance(home), speed, max(0.0, mission.home_z - rec.alt)
        )
        if not battery_flagged and rec.remaining < rec.return_energy + mission.reserve_wh:
            events.append({
                "type": "battery_low",
                "order": idx,
                "point": end,
                "message": f"after {rec.seg_id} the remaining {rec.remaining:.1f} Wh "
                           f"cannot cover the return leg ({rec.return_energy:.1f} Wh) "
                           f"plus reserve ({mission.reserve_wh:.1f} Wh)",
            })
            battery_flagged = True
        prev_pt, prev_alt, prev_home = end, rec.alt, False

    final_return = 0.0
    if records:
        final_return = mission.energy.leg(
            prev_pt.distance(home), speed, max(0.0, mission.home_z - prev_alt)
        )
    total_energy = cumulative + final_return

    # side overlap between consecutive segments on adjacent sweep lines
    # (perpendicular line distance ~= line spacing; collinear parts of the
    # same swath split by a no-fly zone are skipped)
    for a, b in zip(records, records[1:]):
        perp = _perp_distance(a.line, b.line)
        if perp < 0.3 * mission.line_spacing or perp > 2.0 * mission.line_spacing:
            continue
        half = (a.width + b.width) / 2.0
        if half <= 0:
            continue
        ratio = (half - perp) / half
        for rec in (a, b):
            if rec.side_overlap_min is None or ratio < rec.side_overlap_min:
                rec.side_overlap_min = ratio

    # coverage and uncovered patches
    buildable = mission.buildable
    if records:
        cover = unary_union(
            [r.line.buffer(r.width / 2.0, cap_style="square") for r in records]
        )
        covered = cover.intersection(buildable).area
        coverage_pct = 100.0 * covered / buildable.area if buildable.area > 0 else 0.0
        leftover = buildable.difference(cover)
        gap_polys = [g for g in _polygons(leftover) if g.area > GAP_AREA_THRESHOLD_M2]
    else:
        coverage_pct = 0.0
        gap_polys = _polygons(buildable)
    for g in gap_polys:
        c = g.centroid
        nearest = min(records, key=lambda r: r.line.distance(c)) if records else None
        events.append({
            "type": "coverage_gap",
            "order": nearest.order if nearest else 0,
            "point": c,
            "message": f"uncovered patch of {g.area:.1f} m²"
                       + (f" (nearest segment {nearest.seg_id})" if nearest else ""),
        })
    events.sort(key=lambda e: e["order"])

    flight_dist = sum(r.length for r in records)
    total_dist = flight_dist + connector_dist + (prev_pt.distance(home) if records else 0.0)
    risk = {
        "coverage_gaps": sum(1 for e in events if e["type"] == "coverage_gap"),
        "nfz_intrusions": sum(1 for e in events if e["type"] == "nfz_intrusion"),
        "battery_events": sum(1 for e in events if e["type"] == "battery_low"),
    }
    metrics = {
        "coverage_pct": round(coverage_pct, 2),
        "gap_area_m2": round(sum(g.area for g in gap_polys), 1),
        "num_segments": len(records),
        "num_frozen": sum(1 for r in records if r.frozen),
        "photo_count": sum(r.photos for r in records),
        "flight_distance_m": round(flight_dist, 1),
        "total_distance_m": round(total_dist, 1),
        "flight_time_min": round(total_dist / speed / 60.0, 2),
        "total_energy_wh": round(total_energy, 1),
        "battery_wh": mission.battery_wh,
        "battery_ok": risk["battery_events"] == 0,
        "height_amsl_range_m": (
            [round(min(r.alt for r in records), 1), round(max(r.alt for r in records), 1)]
            if records else None
        ),
        "gsd_cm_range": (
            [round(min(r.gsd_min for r in records), 2), round(max(r.gsd_max for r in records), 2)]
            if records else None
        ),
        "risk": risk,
    }

    result = {
        "algo_version": ALGO_VERSION,
        "segments": [_serialize_record(mission, r) for r in records],
        "events": [_serialize_event(mission, e) for e in events],
        "first_event": _serialize_event(mission, events[0]) if events else None,
        "warnings": warnings,
        "metrics": metrics,
    }
    geojson = _to_geojson(mission, records, gap_polys)
    return result, geojson


def _serialize_event(mission, e):
    x, y = mission.crs.inv.transform(e["point"].x, e["point"].y)
    return {
        "type": e["type"],
        "segment_order": e["order"],
        "position": [x, y],
        "message": e["message"],
    }


def _serialize_record(mission, rec):
    coords = [mission.crs.inv.transform(x, y) for x, y in rec.line.coords]
    return {
        "id": rec.seg_id,
        "order": rec.order,
        "frozen": rec.frozen,
        "start": list(coords[0]),
        "end": list(coords[-1]),
        "length_m": round(rec.length, 1),
        "height_amsl_m": round(rec.alt, 1),
        "height_agl_m": round(rec.agl, 1),
        "footprint_width_m": round(rec.width, 1),
        "gsd_cm_min": round(rec.gsd_min, 2),
        "gsd_cm_max": round(rec.gsd_max, 2),
        "forward_overlap_min": (
            None if rec.fwd_overlap_min is None else round(rec.fwd_overlap_min, 3)
        ),
        "side_overlap_min": (
            None if rec.side_overlap_min is None else round(rec.side_overlap_min, 3)
        ),
        "photo_count": rec.photos,
        "leg_energy_wh": round(rec.leg_energy, 2),
        "energy_wh": round(rec.energy, 2),
        "cumulative_energy_wh": round(rec.cumulative, 2),
        "battery_remaining_wh": round(rec.remaining, 2),
        "return_energy_wh": round(rec.return_energy, 2),
        "nfz_intrusion_m": round(rec.nfz_intrusion, 2),
    }


def _to_geojson(mission, records, gap_polys):
    features = []

    def add(geom, props):
        features.append({
            "type": "Feature",
            "properties": props,
            "geometry": mapping(mission.crs.to_source(geom)),
        })

    add(mission.survey, {"feature": "survey_area"})
    for i, poly in enumerate(_polygons(mission.nfz)):
        add(poly, {"feature": "no_fly_zone", "index": i})
    for rec in records:
        add(rec.line, {
            "feature": "segment",
            "id": rec.seg_id,
            "order": rec.order,
            "frozen": rec.frozen,
            "height_amsl_m": round(rec.alt, 1),
            "gsd_cm_max": round(rec.gsd_max, 2),
            "energy_wh": round(rec.energy, 2),
        })
    for g in gap_polys:
        add(g, {"feature": "coverage_gap", "area_m2": round(g.area, 1)})
    add(mission.home, {"feature": "home"})
    return {"type": "FeatureCollection", "features": features}


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------

def build_plan(req):
    """Validate ``req`` and return ``(result, geojson)``."""
    mission = prepare(req)
    records = []
    frozen_cover = []
    for spec in req.get("frozen_segments") or []:
        rec = record_from_spec(mission, spec)
        records.append(rec)
        frozen_cover.append(rec.line.buffer(rec.width / 2.0, cap_style="square"))
    area = mission.buildable
    if frozen_cover:
        area = area.difference(unary_union(frozen_cover))
    records.extend(generate_records(mission, area))
    return finalize(mission, records)
