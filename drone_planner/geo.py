"""CRS handling: everything is planned in a local metric CRS.

Caller geometries arrive in any CRS pyproj understands (default EPSG:4326).
Geographic inputs are reprojected to the UTM zone of the survey centroid;
projected inputs are accepted as-is provided their axis unit is the metre.
"""
from dataclasses import dataclass

from pyproj import CRS, Transformer
from shapely.ops import transform as shp_transform

from .errors import ValidationError

_METRE_UNITS = {"metre", "meter", "m"}


@dataclass
class CrsContext:
    source: CRS
    metric: CRS
    fwd: Transformer  # source -> metric
    inv: Transformer  # metric -> source

    def to_metric(self, geom):
        return shp_transform(self.fwd.transform, geom)

    def to_source(self, geom):
        return shp_transform(self.inv.transform, geom)


def build_crs_context(crs: CRS, hint_geom) -> CrsContext:
    if crs.is_geographic:
        c = hint_geom.centroid
        zone = min(60, max(1, int((c.x + 180.0) // 6) + 1))
        proj = f"+proj=utm +zone={zone} +datum=WGS84 +units=m +no_defs"
        if c.y < 0:
            proj += " +south"
        metric = CRS.from_proj4(proj)
    else:
        unit = ""
        if crs.axis_info:
            unit = (crs.axis_info[0].unit_name or "").lower()
        if unit not in _METRE_UNITS:
            raise ValidationError(
                [f"projected CRS must use metre units, got {unit or 'unknown'}; "
                 "supply a metre-based CRS or a geographic one"]
            )
        metric = crs
    return CrsContext(
        source=crs,
        metric=metric,
        fwd=Transformer.from_crs(crs, metric, always_xy=True),
        inv=Transformer.from_crs(metric, crs, always_xy=True),
    )
