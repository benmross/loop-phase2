"""Tests for the parts that do not need ROS, Qt or a simulator.

Coordinates, the mission state machine, target files and the freshness rules.
These are the places where a bug is invisible until a rover is driving.
"""

import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from s1_gui import geodesy                                            # noqa: E402
from s1_gui.mission import (ABORTED, COMPLETE, Freshness, HOLD, Mission, MissionLog,  # noqa: E402
                            REACHED, RUNNING, Target, load_targets, save_targets)

ORIGIN = (38.42287240335025, -110.78495572815902)


# --------------------------------------------------------------- coordinates

@pytest.mark.parametrize('text,expected', [
    ('38.42287240', 38.42287240),
    ('38.42287240 N', 38.42287240),
    ('-38.42287240', -38.42287240),
    ('38 25.372344', 38 + 25.372344 / 60),
    ('38 25.372344 N', 38 + 25.372344 / 60),
    ('38 25 22.34', 38 + 25 / 60 + 22.34 / 3600),
    ('38°25\'22.34" N', 38 + 25 / 60 + 22.34 / 3600),
])
def test_latitude_formats(text, expected):
    value, _ = geodesy.parse_angle(text, 'lat')
    assert value == pytest.approx(expected, abs=1e-9)


def test_south_and_west_letters_make_it_negative():
    assert geodesy.parse_angle('38 25 22.3 S', 'lat')[0] < 0
    assert geodesy.parse_angle('110 47 05.8 W', 'lon')[0] < 0


@pytest.mark.parametrize('text,axis', [
    ('', 'lat'), ('north', 'lat'), ('91.5', 'lat'), ('-181', 'lon'),
    ('38 75.0 N', 'lat'),            # minutes must be under 60
    ('38 25 61.0 N', 'lat'),         # and so must seconds
    ('38.4 E', 'lat'),               # east is not a latitude
    ('-38.4 N', 'lat'),              # a sign and a letter contradicting
])
def test_invalid_coordinates_are_rejected(text, axis):
    with pytest.raises(geodesy.CoordinateError):
        geodesy.parse_angle(text, axis)


def test_local_frame_round_trip():
    for east, north in [(0, 0), (250, -400), (-999, 999), (12.5, 3.25)]:
        latitude, longitude = geodesy.local_to_latlon(east, north, *ORIGIN)
        back_east, back_north = geodesy.latlon_to_local(latitude, longitude, *ORIGIN)
        assert back_east == pytest.approx(east, abs=1e-6)
        assert back_north == pytest.approx(north, abs=1e-6)


def test_local_frame_matches_matthews_convention():
    """East is +x, north is +y, computed from the geodesic, as gps_odometry does."""
    east, north = geodesy.latlon_to_local(ORIGIN[0] + 0.001, ORIGIN[1], *ORIGIN)
    assert north > 100 and abs(east) < 1e-3
    east, north = geodesy.latlon_to_local(ORIGIN[0], ORIGIN[1] + 0.001, *ORIGIN)
    assert east > 80 and abs(north) < 1e-3


def test_utm_and_local_frames_differ_by_the_grid_convergence():
    convergence = geodesy.grid_convergence_deg(*ORIGIN)
    assert 0.1 < abs(convergence) < 2.0, 'MDRS is about half a degree off grid north'
    # 1 km north in the local frame is not 1 km north on the UTM grid.
    latitude, longitude = geodesy.local_to_latlon(0.0, 1000.0, *ORIGIN)
    east0, north0 = geodesy.latlon_to_utm(*ORIGIN)
    east1, north1 = geodesy.latlon_to_utm(latitude, longitude)
    assert abs(east1 - east0) > 1.0, 'the difference is metres, not nothing'


def test_formatting_round_trips_through_parsing():
    for style in geodesy.FORMATS:
        text = geodesy.format_angle(ORIGIN[0], 'lat', style)
        value, detected = geodesy.parse_angle(text, 'lat')
        assert detected == style
        assert value == pytest.approx(ORIGIN[0], abs=1e-6)


# ------------------------------------------------------------------- mission

def make_mission(count=3):
    mission = Mission()
    targets = []
    for index in range(count):
        target = Target(name=f'Post {index + 1}', latitude=ORIGIN[0] + 0.001 * (index + 1),
                        longitude=ORIGIN[1], tolerance_m=2.0)
        target.east_m, target.north_m = geodesy.latlon_to_local(
            target.latitude, target.longitude, *ORIGIN)
        targets.append(target)
    assert mission.set_targets(targets)[0]
    return mission


def test_mission_runs_through_its_targets_in_order():
    mission = make_mission()
    assert mission.start()[0]
    assert mission.state == RUNNING and mission.active_index == 0
    for index in range(3):
        target = mission.targets[index]
        assert mission.arrived(target.east_m + 0.5, target.north_m) is not None
    assert mission.state == COMPLETE
    assert all(t.status == REACHED for t in mission.targets)


def test_arrival_needs_to_be_inside_the_tolerance():
    mission = make_mission()
    mission.start()
    target = mission.targets[0]
    assert mission.arrived(target.east_m + 5.0, target.north_m) is None
    assert mission.active_index == 0


def test_hold_and_resume_keep_the_target():
    mission = make_mission()
    mission.start()
    assert mission.hold()[0] and mission.state == HOLD
    assert not mission.arrived(*(mission.targets[0].east_m, mission.targets[0].north_m)), \
        'a held mission does not count arrivals'
    assert mission.resume()[0] and mission.state == RUNNING
    assert mission.active_index == 0


def test_abort_clears_the_active_target_and_blocks_progress():
    mission = make_mission()
    mission.start()
    assert mission.abort()[0]
    assert mission.state == ABORTED and mission.active_index == -1
    assert mission.arrived(0.0, 0.0) is None
    assert mission.start()[0], 'a new start after abort is allowed'


def test_targets_cannot_change_while_running():
    mission = make_mission()
    mission.start()
    accepted, detail = mission.set_targets([])
    assert not accepted and 'abort' in detail
    accepted, _ = mission.move_target(0, 1)
    assert not accepted


def test_reordering_and_skipping():
    mission = make_mission()
    first = mission.targets[0].name
    assert mission.move_target(0, 1)[0]
    assert mission.targets[1].name == first
    mission.start()
    assert mission.skip_active()[0]
    assert mission.targets[0].status == 3          # SKIPPED
    assert mission.active_index == 1


def test_out_of_range_targets_are_refused():
    mission = Mission()
    accepted, detail = mission.set_targets([Target('bad', 95.0, 0.0)])
    assert not accepted and 'latitude' in detail


# ---------------------------------------------------------------- files, log

def test_target_files_round_trip(tmp_path):
    mission = make_mission()
    path = tmp_path / 'targets.json'
    save_targets(path, mission.targets, ORIGIN)
    loaded = load_targets(path)
    assert [t.name for t in loaded] == [t.name for t in mission.targets]
    assert loaded[0].latitude == pytest.approx(mission.targets[0].latitude)


def test_loading_a_bad_file_says_why(tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text('{"version": 1, "targets": [{"latitude": 1.0}]}')
    with pytest.raises(ValueError, match='malformed'):
        load_targets(path)
    path.write_text('{"version": 99, "targets": []}')
    with pytest.raises(ValueError, match='version'):
        load_targets(path)


def test_log_exports_json_and_csv(tmp_path):
    log = MissionLog()
    log.add('command', 'START by operator', accepted=True)
    log.add('conversion', 'Post 1 converted', east_m=1.0, north_m=2.0)
    json_path, csv_path = log.export(tmp_path / 'mission.json')
    assert os.path.exists(json_path) and os.path.exists(csv_path)
    assert 'START by operator' in open(json_path).read()
    assert 'east_m' in open(csv_path).read()


# ----------------------------------------------------------------- freshness

def test_freshness_thresholds():
    now = [100.0]
    fresh = Freshness({'odometry': (1.0, 3.0)}, clock=lambda: now[0])
    assert fresh.status('odometry')[0] == Freshness.NEVER
    fresh.mark('odometry')
    assert fresh.status('odometry')[0] == Freshness.LIVE
    now[0] += 1.5
    assert fresh.status('odometry')[0] == Freshness.STALE
    now[0] += 2.0
    assert fresh.status('odometry')[0] == Freshness.LOST
    assert fresh.worst() == ('odometry', Freshness.LOST)
