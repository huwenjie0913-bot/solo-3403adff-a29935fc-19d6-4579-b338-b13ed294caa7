"""Simple energy model.

Cruise energy is proportional to flight time; climbing costs a fixed
amount per metre gained.  Descents are treated as energy-neutral.
"""
from dataclasses import dataclass


@dataclass
class EnergyModel:
    cruise_power_w: float = 260.0
    climb_wh_per_m: float = 0.03

    def leg(self, distance_m, speed_mps, climb_m=0.0):
        wh = distance_m / speed_mps * self.cruise_power_w / 3600.0
        return wh + max(0.0, climb_m) * self.climb_wh_per_m
