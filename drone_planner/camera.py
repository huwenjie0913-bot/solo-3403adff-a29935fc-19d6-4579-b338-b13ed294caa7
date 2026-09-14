"""Camera model: thin-lens footprint / GSD relations."""
from dataclasses import dataclass


@dataclass
class Camera:
    sensor_width_mm: float
    sensor_height_mm: float
    focal_length_mm: float
    image_width_px: int
    image_height_px: int

    def height_for_gsd(self, gsd_m):
        """AGL height (m) that yields ``gsd_m`` ground sampling distance."""
        return gsd_m * self.focal_length_mm * self.image_width_px / self.sensor_width_mm

    def footprint(self, height_agl_m):
        """(cross-track width, along-track length) on the ground, in metres."""
        return (
            height_agl_m * self.sensor_width_mm / self.focal_length_mm,
            height_agl_m * self.sensor_height_mm / self.focal_length_mm,
        )

    def gsd(self, height_agl_m):
        """GSD (m/px) at the given AGL height."""
        return (
            height_agl_m
            * self.sensor_width_mm
            / (self.focal_length_mm * self.image_width_px)
        )
