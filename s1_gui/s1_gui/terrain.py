"""The offline map: Matthew's USGS terrain, rendered for an operator.

The GUI has to show the MDRS area with no internet. The elevation model is
already in the navigation package (2 km square, 1 m USGS data, centred on the
site), so this reads that rather than shipping a second copy which could
disagree with the terrain the rover is actually driving on.

Two renderings come out of the same array:

hillshade   what the ground looks like, so an operator can recognise the mesa
            and the wash they are driving into.
slope       the traversability layer, coloured by how close each cell is to
            the 30 degree limit the rover cannot climb. Computed the same way
            as elevation_map_publisher.py: the magnitude of the elevation
            gradient, clipped at 30 degrees.

Both are cached as .npy next to nothing: in the user's cache directory, keyed
by the source file's modification time, because computing them takes a few
seconds and the GUI should start quickly.
"""

import hashlib
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image

MAX_TRAVERSABLE_SLOPE_DEGREES = 30.0
RENDER_SIZE = 1024


class Terrain:
    """The elevation model, its geo-referencing, and the rendered layers."""

    def __init__(self, worlds_dir, size=RENDER_SIZE):
        self.worlds = Path(worlds_dir)
        with open(self.worlds / 'usgs_utah_metadata.json') as handle:
            self.metadata = json.load(handle)
        self.center = (self.metadata['center_wgs84']['latitude'],
                       self.metadata['center_wgs84']['longitude'])
        self.bounds_utm = tuple(self.metadata['bounds_utm_m'])   # minx, miny, maxx, maxy
        self.extent_m = tuple(self.metadata['extent_m'])
        self.size = size

        heightmap = ET.parse(self.worlds / 'usgs_utah.sdf').find(
            ".//visual[@name='terrain_visual']/geometry/heightmap")
        self.size_x, self.size_y, self.size_z = map(float, heightmap.findtext('size').split())
        _, _, self.offset_z = map(float, heightmap.findtext('pos').split())
        self.image_path = self.worlds / heightmap.findtext('uri')

        self.elevation = self._elevation()
        self.slope_degrees = self._slope(self.elevation)
        self.hillshade_rgb = self._hillshade(self.elevation)
        self.slope_rgb = self._slope_colors(self.slope_degrees)

    # Loading

    def _cache_path(self, kind):
        stamp = f'{self.image_path}:{os.path.getmtime(self.image_path)}:{self.size}'
        key = hashlib.sha1(stamp.encode()).hexdigest()[:16]
        cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 's1_gui'
        cache.mkdir(parents=True, exist_ok=True)
        return cache / f'{kind}-{key}.npy'

    def _elevation(self):
        """Metres above the world's zero plane, one cell per rendered pixel."""
        cached = self._cache_path('elevation')
        if cached.exists():
            return np.load(cached)
        with Image.open(self.image_path) as image:
            raw = np.asarray(image, dtype=np.float32) / 65535.0
        # Heightmap samples are vertices; average each four-vertex cell, the
        # same correction elevation_map_publisher makes, so cell centres line
        # up with the metres the metadata promises.
        cells = (raw[:-1, :-1] + raw[1:, :-1] + raw[:-1, 1:] + raw[1:, 1:]) * 0.25
        cells = self.offset_z + cells * self.size_z
        if cells.shape[0] != self.size:
            factor = cells.shape[0] // self.size
            trimmed = cells[:factor * self.size, :factor * self.size]
            cells = trimmed.reshape(self.size, factor, self.size, factor).mean(axis=(1, 3))
        np.save(cached, cells.astype(np.float32))
        return cells.astype(np.float32)

    def _slope(self, elevation):
        resolution = self.size_x / elevation.shape[1]
        gradient_y, gradient_x = np.gradient(elevation, resolution)
        return np.degrees(np.arctan(np.hypot(gradient_x, gradient_y))).astype(np.float32)

    # Rendering

    @staticmethod
    def _hillshade(elevation, azimuth_deg=315.0, altitude_deg=45.0):
        """Standard hillshade, tinted the colour the ground actually is."""
        gradient_y, gradient_x = np.gradient(elevation)
        slope = np.arctan(np.hypot(gradient_x, gradient_y))
        aspect = np.arctan2(-gradient_x, gradient_y)
        azimuth = np.radians(360.0 - azimuth_deg + 90.0)
        altitude = np.radians(altitude_deg)
        shaded = (np.sin(altitude) * np.cos(slope)
                  + np.cos(altitude) * np.sin(slope) * np.cos(azimuth - aspect))
        shaded = np.clip((shaded + 1.0) / 2.0, 0.0, 1.0)

        # Height also tints it, so high ground reads as high ground.
        low, high = np.percentile(elevation, [2, 98])
        height = np.clip((elevation - low) / max(high - low, 1e-6), 0.0, 1.0)
        base = np.stack([0.52 + 0.30 * height, 0.42 + 0.26 * height, 0.33 + 0.20 * height], axis=-1)
        rgb = base * (0.35 + 0.85 * shaded[..., None])
        return np.clip(rgb * 255.0, 0, 255).astype(np.uint8)

    @staticmethod
    def _slope_colors(slope_degrees):
        """Green to amber to red, saturating at the 30 degree limit."""
        cost = np.clip(slope_degrees / MAX_TRAVERSABLE_SLOPE_DEGREES, 0.0, 1.0)
        red = np.clip(cost * 2.0, 0, 1)
        green = np.clip(1.6 - cost * 1.9, 0, 1)
        blue = np.clip(0.35 - cost * 0.35, 0, 1)
        rgb = np.stack([red, green, blue], axis=-1) * (0.45 + 0.55 * cost[..., None])
        return np.clip(rgb * 255.0, 0, 255).astype(np.uint8)

    # Geo-referencing
    #
    # Image row 0 is the north edge and column 0 is the west edge, so northing
    # decreases as rows increase. Everything below goes through UTM, which is
    # the grid the DEM was cut on.

    def utm_to_pixel(self, easting, northing):
        min_x, min_y, max_x, max_y = self.bounds_utm
        px = (easting - min_x) / (max_x - min_x) * self.size
        py = (max_y - northing) / (max_y - min_y) * self.size
        return px, py

    def pixel_to_utm(self, px, py):
        min_x, min_y, max_x, max_y = self.bounds_utm
        easting = min_x + px / self.size * (max_x - min_x)
        northing = max_y - py / self.size * (max_y - min_y)
        return easting, northing

    def contains_utm(self, easting, northing):
        min_x, min_y, max_x, max_y = self.bounds_utm
        return min_x <= easting <= max_x and min_y <= northing <= max_y

    def elevation_at_pixel(self, px, py):
        row = int(np.clip(py, 0, self.size - 1))
        column = int(np.clip(px, 0, self.size - 1))
        return float(self.elevation[row, column])

    def slope_at_pixel(self, px, py):
        row = int(np.clip(py, 0, self.size - 1))
        column = int(np.clip(px, 0, self.size - 1))
        return float(self.slope_degrees[row, column])


def find_worlds_dir(explicit=None):
    """Where Matthew's terrain lives: the installed package, or a source tree."""
    if explicit:
        path = Path(explicit)
        if (path / 'usgs_utah_metadata.json').exists():
            return path
        raise FileNotFoundError(f'no terrain metadata in {path}')
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('s1_navigation')) / 'worlds'
        if (share / 'usgs_utah_metadata.json').exists():
            return share
    except Exception:                                            # noqa: BLE001
        pass
    for candidate in (Path.home() / 'loop_ws/src/Challenge_Week/ros2_ws/src'
                      / 's1_navigation/worlds',):
        if (candidate / 'usgs_utah_metadata.json').exists():
            return candidate
    raise FileNotFoundError('cannot find the s1_navigation terrain files')
