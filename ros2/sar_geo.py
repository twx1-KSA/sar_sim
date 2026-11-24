"""Shared geometry for the SAR simulation: world <-> GPS conversion and terrain heights.

World frame = Gazebo world of sar_mountain.sdf: x east, y north, z up, metres, origin at
the spherical_coordinates reference (24.7136 N, 46.6753 E, 600 m AMSL).
"""

import math
import os

import numpy as np
from PIL import Image as PILImage

EARTH_RADIUS = 6378137.0
PX4_DIR = os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))
PX4_GZ_DIR = f"{PX4_DIR}/Tools/simulation/gz"
REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_DEM = f"{REPO_DIR}/gz/models/sar_terrain/materials/textures/heightmap.png"

ORIGIN_LAT = 24.7136
ORIGIN_LON = 46.6753
ORIGIN_ALT = 600.0


class GeoRef:
    """Flat-earth conversion around the world origin (accurate to cm over a few hundred metres)."""

    def __init__(self, lat0=ORIGIN_LAT, lon0=ORIGIN_LON, alt0=ORIGIN_ALT):
        self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
        self._m_per_deg_lat = math.radians(1) * EARTH_RADIUS
        self._m_per_deg_lon = self._m_per_deg_lat * math.cos(math.radians(lat0))

    def enu_from_geo(self, lat, lon):
        return (lon - self.lon0) * self._m_per_deg_lon, (lat - self.lat0) * self._m_per_deg_lat

    def geo_from_enu(self, east, north):
        return self.lat0 + north / self._m_per_deg_lat, self.lon0 + east / self._m_per_deg_lon


class Terrain:
    """Terrain height in metres AMSL at a world (east, north) position, from the Gazebo heightmap.

    If the DEM is missing, height_amsl() returns fallback_alt_fn() (e.g. the home altitude).
    """

    def __init__(self, dem_path=DEFAULT_DEM, size_m=500.0, height_m=90.0, origin_alt=ORIGIN_ALT,
                 fallback_alt_fn=None):
        self.origin_alt = origin_alt
        self.fallback_alt_fn = fallback_alt_fn or (lambda: origin_alt)
        self.dem = None
        self.size = size_m
        if dem_path and os.path.isfile(dem_path):
            raw = np.asarray(PILImage.open(dem_path)).astype(np.float64)
            if raw.ndim == 3:
                raw = raw[..., 0]
            full = 65535.0 if raw.max() > 255 else 255.0
            self.dem = raw / full * height_m  # Gazebo maps pixel value linearly to [0, height]
            self.n = self.dem.shape[0] - 1

    def height_amsl(self, east, north):
        if self.dem is None:
            return self.fallback_alt_fn()
        # Image row 0 is the +north edge, column 0 the -east edge (verified against the running sim).
        col = (east + self.size / 2) / self.size * self.n
        row = (self.size / 2 - north) / self.size * self.n
        col = min(max(col, 0.0), self.n - 1e-6)
        row = min(max(row, 0.0), self.n - 1e-6)
        c0, r0 = int(col), int(row)
        fc, fr = col - c0, row - r0
        d = self.dem
        h = (d[r0, c0] * (1 - fc) * (1 - fr) + d[r0, c0 + 1] * fc * (1 - fr)
             + d[r0 + 1, c0] * (1 - fc) * fr + d[r0 + 1, c0 + 1] * fc * fr)
        return self.origin_alt + h
