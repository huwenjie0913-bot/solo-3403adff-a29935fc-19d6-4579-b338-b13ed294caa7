"""Request validation: CRS, units/ranges, geometry validity, structure."""
from pyproj import CRS
from pyproj.exceptions import CRSError
from shapely.geometry import shape
from shapely.validation import explain_validity

from .errors import ValidationError

_CAMERA_KEYS = (
    "sensor_width_mm",
    "sensor_height_mm",
    "focal_length_mm",
    "image_width_px",
    "image_height_px",
)

_FROZEN_KEYS = (
    "start",
    "end",
    "height_amsl_m",
    "height_agl_m",
    "footprint_width_m",
    "leg_energy_wh",
    "photo_count",
    "gsd_cm_min",
    "gsd_cm_max",
    "nfz_intrusion_m",
)


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _num(req, key, errors, lo=None, hi=None, lo_inclusive=False):
    v = req.get(key)
    if not _is_num(v):
        errors.append(f"{key}: required number (metres/seconds/units as documented)")
        return None
    v = float(v)
    if lo is not None and (v < lo if lo_inclusive else v <= lo):
        errors.append(f"{key}: must be {'>=' if lo_inclusive else '>'} {lo}, got {v}")
    if hi is not None and v > hi:
        errors.append(f"{key}: must be <= {hi}, got {v}")
    return v


def _polygon(raw, name, errors):
    if raw is None:
        errors.append(f"{name}: required GeoJSON Polygon")
        return None
    try:
        geom = shape(raw)
    except Exception as exc:
        errors.append(f"{name}: unparseable geometry ({exc})")
        return None
    if geom.geom_type != "Polygon" or geom.is_empty:
        errors.append(f"{name}: must be a non-empty Polygon, got {geom.geom_type}")
        return None
    if not geom.is_valid:
        errors.append(
            f"{name}: self-intersecting or invalid geometry ({explain_validity(geom)})"
        )
        return None
    return geom


def _terrain(raw, errors):
    if not isinstance(raw, dict):
        errors.append("terrain: required object {origin, cell_size, values}")
        return None
    origin = raw.get("origin")
    cell = raw.get("cell_size")
    values = raw.get("values")
    ok = True
    if not (isinstance(origin, (list, tuple)) and len(origin) == 2 and all(_is_num(v) for v in origin)):
        errors.append("terrain.origin: must be [x, y] in the request CRS")
        ok = False
    if not (
        isinstance(cell, (list, tuple))
        and len(cell) == 2
        and all(_is_num(v) and v > 0 for v in cell)
    ):
        errors.append("terrain.cell_size: must be two positive numbers")
        ok = False
    if not (isinstance(values, list) and len(values) >= 2 and all(isinstance(r, list) for r in values)):
        errors.append("terrain.values: must be a list of at least 2 rows")
        ok = False
    if not ok:
        return None
    ncols = len(values[0])
    if ncols < 2 or any(len(r) != ncols for r in values):
        errors.append("terrain.values: rows must all have the same length (>= 2)")
        return None
    for r, row in enumerate(values):
        for c, v in enumerate(row):
            if v is None:
                continue  # explicit elevation gap, checked against survey area later
            if not _is_num(v) or not (-1000.0 <= float(v) <= 10000.0):
                errors.append(f"terrain.values[{r}][{c}]: must be null or -1000..10000 m")
                return None
    return {
        "origin": [float(origin[0]), float(origin[1])],
        "cell_size": [float(cell[0]), float(cell[1])],
        "values": values,
    }


def _camera(raw, errors):
    if not isinstance(raw, dict):
        errors.append("camera: required object with intrinsics")
        return None
    cam = {}
    for key in _CAMERA_KEYS:
        v = raw.get(key)
        if not _is_num(v) or v <= 0:
            errors.append(f"camera.{key}: must be a positive number")
            return None
        cam[key] = int(v) if key.endswith("_px") else float(v)
    return cam


def validate_request(req):
    if not isinstance(req, dict):
        raise ValidationError(["request body must be a JSON object"])
    errors = []

    crs = None
    crs_in = req.get("crs", "EPSG:4326")
    try:
        crs = CRS.from_user_input(crs_in)
    except (CRSError, TypeError, ValueError):
        errors.append(f"crs: cannot parse {crs_in!r} (use e.g. 'EPSG:4326')")

    survey = _polygon(req.get("survey_area"), "survey_area", errors)
    nfz_raw = req.get("no_fly_zones", [])
    if not isinstance(nfz_raw, list):
        errors.append("no_fly_zones: must be a list of GeoJSON Polygons")
        nfz_raw = []
    nfz = [_polygon(g, f"no_fly_zones[{i}]", errors) for i, g in enumerate(nfz_raw)]
    nfz = [g for g in nfz if g is not None]

    terrain = _terrain(req.get("terrain"), errors)
    camera = _camera(req.get("camera"), errors)

    gsd = _num(req, "target_gsd_cm", errors, lo=0.1, hi=100.0)
    fwd = _num(req, "forward_overlap", errors, lo=0.0, hi=0.95, lo_inclusive=True)
    side = _num(req, "side_overlap", errors, lo=0.0, hi=0.95, lo_inclusive=True)
    speed = _num(req, "speed_mps", errors, lo=0.1, hi=60.0)
    turn = _num(req, "turn_radius_m", errors, lo=0.0, hi=10000.0, lo_inclusive=True)
    battery = _num(req, "battery_wh", errors, lo=1.0)
    reserve = _num(req, "reserve_wh", errors, lo=0.0, lo_inclusive=True)
    if battery is not None and reserve is not None and reserve >= battery:
        errors.append("reserve_wh: must be smaller than battery_wh")

    heading = req.get("heading_deg", 0.0)
    if not _is_num(heading):
        errors.append("heading_deg: must be a number")
        heading = 0.0

    home = req.get("home")
    if not (isinstance(home, (list, tuple)) and len(home) == 2 and all(_is_num(v) for v in home)):
        errors.append("home: must be [x, y] in the request CRS")
        home = None

    options = req.get("options") or {}
    if not isinstance(options, dict):
        errors.append("options: must be an object")
        options = {}
    cruise = options.get("cruise_power_w", 260.0)
    climb = options.get("climb_wh_per_m", 0.03)
    if not _is_num(cruise) or cruise <= 0:
        errors.append("options.cruise_power_w: must be a positive number")
    if not _is_num(climb) or climb < 0:
        errors.append("options.climb_wh_per_m: must be a non-negative number")

    frozen = req.get("frozen_segments") or []
    if not isinstance(frozen, list):
        errors.append("frozen_segments: must be a list")
        frozen = []
    else:
        for i, spec in enumerate(frozen):
            if not isinstance(spec, dict) or any(k not in spec for k in _FROZEN_KEYS):
                errors.append(f"frozen_segments[{i}]: missing one of {sorted(_FROZEN_KEYS)}")

    if errors:
        raise ValidationError(errors)

    return {
        "crs_obj": crs,
        "survey_area": survey,
        "no_fly_zones": nfz,
        "terrain": terrain,
        "camera": camera,
        "target_gsd_cm": gsd,
        "forward_overlap": fwd,
        "side_overlap": side,
        "speed_mps": speed,
        "turn_radius_m": turn,
        "battery_wh": battery,
        "reserve_wh": reserve,
        "heading_deg": float(heading),
        "home": [float(home[0]), float(home[1])],
        "options": {"cruise_power_w": float(cruise), "climb_wh_per_m": float(climb)},
        "frozen_segments": frozen,
    }
