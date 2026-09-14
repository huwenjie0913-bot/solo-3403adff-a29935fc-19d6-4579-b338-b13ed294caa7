"""Regular-grid terrain sampler.

The grid is supplied in the request CRS and converted to the metric working
CRS by transforming the origin and one cell offset (adequate for survey-scale
areas).  ``values[r][c]`` is the elevation (m AMSL) at the centre of cell
(r, c); row 0 is the southernmost row.  ``null`` marks an elevation gap.
"""
import math

import numpy as np
from shapely.geometry import Point


class TerrainGrid:
    def __init__(self, x0, y0, dx, dy, values):
        self.x0 = float(x0)
        self.y0 = float(y0)
        self.dx = float(dx)
        self.dy = float(dy)
        self.values = values  # np.ndarray (nrows, ncols), NaN = no data
        self.nrows, self.ncols = values.shape

    @classmethod
    def from_request(cls, t, crs_ctx):
        ox, oy = t["origin"]
        dx, dy = t["cell_size"]
        x0, y0 = crs_ctx.fwd.transform(ox, oy)
        x1, y1 = crs_ctx.fwd.transform(ox + dx, oy + dy)
        vals = np.array(
            [[math.nan if v is None else float(v) for v in row] for row in t["values"]],
            dtype=float,
        )
        return cls(x0, y0, abs(x1 - x0), abs(y1 - y0), vals)

    # -- queries ---------------------------------------------------------

    def covers_bounds(self, bounds, tol=1e-6):
        minx, miny, maxx, maxy = bounds
        return (
            minx >= self.x0 - tol
            and maxx <= self.x0 + self.ncols * self.dx + tol
            and miny >= self.y0 - tol
            and maxy <= self.y0 + self.nrows * self.dy + tol
        )

    def height(self, x, y):
        """Bilinear elevation at (x, y); None when outside or data missing."""
        gx = (x - self.x0) / self.dx - 0.5
        gy = (y - self.y0) / self.dy - 0.5
        if gx < -0.5 or gy < -0.5 or gx > self.ncols - 0.5 or gy > self.nrows - 0.5:
            return None
        gx = min(max(gx, 0.0), self.ncols - 1.0)
        gy = min(max(gy, 0.0), self.nrows - 1.0)
        c0, r0 = int(math.floor(gx)), int(math.floor(gy))
        c1, r1 = min(c0 + 1, self.ncols - 1), min(r0 + 1, self.nrows - 1)
        q = self.values[r0, c0], self.values[r0, c1], self.values[r1, c0], self.values[r1, c1]
        if any(math.isnan(v) for v in q):
            nearest = self.values[int(round(gy)), int(round(gx))]
            return None if math.isnan(nearest) else float(nearest)
        fx, fy = gx - c0, gy - r0
        top = q[0] * (1 - fx) + q[1] * fx
        bot = q[2] * (1 - fx) + q[3] * fx
        return float(top * (1 - fy) + bot * fy)

    def missing_within(self, polygon, limit=5):
        """Centres of no-data cells inside ``polygon`` (first ``limit``)."""
        minx, miny, maxx, maxy = polygon.bounds
        c0 = max(0, int((minx - self.x0) / self.dx))
        c1 = min(self.ncols - 1, int((maxx - self.x0) / self.dx))
        r0 = max(0, int((miny - self.y0) / self.dy))
        r1 = min(self.nrows - 1, int((maxy - self.y0) / self.dy))
        out = []
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                if not np.isnan(self.values[r, c]):
                    continue
                p = Point(self.x0 + (c + 0.5) * self.dx, self.y0 + (r + 0.5) * self.dy)
                if polygon.covers(p):
                    out.append((round(p.x, 1), round(p.y, 1)))
                    if len(out) >= limit:
                        return out
        return out
