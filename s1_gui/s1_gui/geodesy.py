"""Coordinates: parsing, formatting, and the two conversions that matter.

There are three coordinate systems in play and it is worth being precise
about them, because two of them nearly agree and the difference is the sort
of thing that quietly puts a rover ten metres off a post.

WGS 84 geographic
    Latitude and longitude, what the rules hand you and what the operator
    types. Accepted in three formats (see ``parse_latlon``).

The local ROS frame (REP 103: x east, y north, z up, right handed)
    What ``s1_navigation`` drives in. Matthew's ``gps_odometry`` builds it by
    taking the WGS 84 geodesic from a fixed origin to the fix, then splitting
    that distance by its initial azimuth:

        azimuth, distance = Geod.inv(origin, fix)
        east  = distance * sin(azimuth)
        north = distance * cos(azimuth)

    This module uses exactly the same call, so a target the GUI sends lands
    where his planner thinks it is. Doing it any other way, even a "better"
    way, would put the GUI and the navigation stack in different frames.

UTM zone 12N (EPSG:26912)
    What the USGS elevation model is in, and therefore the grid the offline
    map image is pixel-aligned to.

The local frame and the UTM grid are not the same frame. UTM north is grid
north, which differs from true north by the grid convergence, about 0.5
degrees here. Over a 1 km leg that is an 8 m difference in where a point
lands. So the map draws things by converting geographic to UTM, and the rover
is commanded by converting geographic to the local geodesic frame, and
nothing is ever converted by assuming the two are interchangeable.
"""

import math
import re

from pyproj import CRS, Geod, Transformer

GEOD = Geod(ellps='WGS84')
UTM_CRS = CRS.from_epsg(26912)
WGS84_CRS = CRS.from_epsg(4326)
_TO_UTM = Transformer.from_crs(WGS84_CRS, UTM_CRS, always_xy=True)
_FROM_UTM = Transformer.from_crs(UTM_CRS, WGS84_CRS, always_xy=True)


class CoordinateError(ValueError):
    """A coordinate the operator typed that cannot be used."""


# --------------------------------------------------------------------------
# Geographic <-> local ROS frame. Matthew's convention, deliberately.
# --------------------------------------------------------------------------

def latlon_to_local(latitude, longitude, origin_lat, origin_lon):
    """Metres east and north of the origin, along the WGS 84 geodesic."""
    azimuth, _, distance = GEOD.inv(origin_lon, origin_lat, longitude, latitude)
    radians = math.radians(azimuth)
    return distance * math.sin(radians), distance * math.cos(radians)


def local_to_latlon(east, north, origin_lat, origin_lon):
    """The inverse: where a local point is on the ellipsoid."""
    distance = math.hypot(east, north)
    if distance == 0.0:
        return origin_lat, origin_lon
    azimuth = math.degrees(math.atan2(east, north))
    longitude, latitude, _ = GEOD.fwd(origin_lon, origin_lat, azimuth, distance)
    return latitude, longitude


# --------------------------------------------------------------------------
# Geographic <-> UTM, for drawing on the elevation model's own grid.
# --------------------------------------------------------------------------

def latlon_to_utm(latitude, longitude):
    return _TO_UTM.transform(longitude, latitude)


def utm_to_latlon(easting, northing):
    longitude, latitude = _FROM_UTM.transform(easting, northing)
    return latitude, longitude


def grid_convergence_deg(latitude, longitude):
    """Angle from grid north to true north at a point, in degrees.

    Reported in the GUI because it is the reason the local frame and the map
    grid disagree, and an operator who sees a small rotation between the two
    should be able to find out why without reading the source.
    """
    east, north = latlon_to_utm(latitude, longitude)
    north_lat, north_lon = utm_to_latlon(east, north + 100.0)
    azimuth, _, _ = GEOD.inv(longitude, latitude, north_lon, north_lat)
    return (azimuth + 180.0) % 360.0 - 180.0


# --------------------------------------------------------------------------
# The three input formats.
# --------------------------------------------------------------------------

_DD = re.compile(r'^\s*([+-]?\d+(?:\.\d+)?)\s*°?\s*([NSEW])?\s*$', re.I)
_DDM = re.compile(
    r'^\s*([+-]?\d+)\s*(?:°|\s)\s*(\d+(?:\.\d+)?)\s*(?:\'|\u2032)?\s*([NSEW])?\s*$', re.I)
_DMS = re.compile(
    r'^\s*([+-]?\d+)\s*(?:°|\s)\s*(\d+)\s*(?:\'|′|\s)\s*(\d+(?:\.\d+)?)\s*'
    r'(?:"|″)?\s*([NSEW])?\s*$', re.I)

FORMATS = ('DD', 'DDM', 'DMS')


def parse_angle(text, axis):
    """Parse one latitude or longitude in any of the three accepted formats.

    Returns (degrees, format_name). ``axis`` is 'lat' or 'lon' and decides the
    valid range and which hemisphere letters are allowed.

    Accepted:
        DD    38.42287, -110.78496, 38.42287 N
        DDM   38 25.372 N, -110 47.097
        DMS   38 25 22.3 N, 110°47'05.8"W
    """
    if text is None or not str(text).strip():
        raise CoordinateError('empty')
    raw = str(text).strip().replace('º', '°')
    limit = 90.0 if axis == 'lat' else 180.0
    letters = 'NS' if axis == 'lat' else 'EW'

    for name, pattern in (('DMS', _DMS), ('DDM', _DDM), ('DD', _DD)):
        match = pattern.match(raw)
        if not match:
            continue
        groups = match.groups()
        hemisphere = groups[-1]
        numbers = [g for g in groups[:-1] if g is not None]
        degrees = abs(float(numbers[0]))
        sign = -1.0 if numbers[0].startswith('-') else 1.0
        if name == 'DDM':
            minutes = float(numbers[1])
            if minutes >= 60.0:
                raise CoordinateError(f'{minutes} minutes is not a minute value')
            degrees += minutes / 60.0
        elif name == 'DMS':
            minutes, seconds = float(numbers[1]), float(numbers[2])
            if minutes >= 60.0 or seconds >= 60.0:
                raise CoordinateError('minutes and seconds must be under 60')
            degrees += minutes / 60.0 + seconds / 3600.0
        if hemisphere:
            hemisphere = hemisphere.upper()
            if hemisphere not in letters:
                raise CoordinateError(
                    f'{hemisphere} is not a {"latitude" if axis == "lat" else "longitude"}')
            if sign < 0:
                raise CoordinateError('a sign and a hemisphere letter contradict each other')
            sign = -1.0 if hemisphere in 'SW' else 1.0
        value = sign * degrees
        if not -limit <= value <= limit:
            raise CoordinateError(f'{value:g} is outside +/-{limit:g}')
        return value, name

    raise CoordinateError(f'{raw!r} is not a coordinate in DD, DDM or DMS')


def parse_latlon(latitude_text, longitude_text):
    """Both halves. Returns (lat, lon, format) and raises on anything invalid."""
    latitude, lat_format = parse_angle(latitude_text, 'lat')
    longitude, lon_format = parse_angle(longitude_text, 'lon')
    return latitude, longitude, (lat_format if lat_format == lon_format
                                 else f'{lat_format}/{lon_format}')


def format_angle(degrees, axis, style='DD'):
    """Render a coordinate back out in any of the three formats."""
    letter = ('N' if degrees >= 0 else 'S') if axis == 'lat' else ('E' if degrees >= 0 else 'W')
    value = abs(degrees)
    if style == 'DD':
        return f'{value:.6f}° {letter}'
    minutes = (value - int(value)) * 60.0
    if style == 'DDM':
        return f'{int(value)}° {minutes:.4f}′ {letter}'
    if style == 'DMS':
        seconds = (minutes - int(minutes)) * 60.0
        return f'{int(value)}° {int(minutes)}′ {seconds:.2f}″ {letter}'
    raise ValueError(f'unknown style {style}')


def format_latlon(latitude, longitude, style='DD'):
    return f'{format_angle(latitude, "lat", style)}, {format_angle(longitude, "lon", style)}'
