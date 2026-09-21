"""The mission itself: targets, the state machine, and the log.

No ROS and no Qt in this file. The rules a mission obeys are worth testing
on their own, and they are the part most likely to be wrong in a way that
only shows up once the rover is moving.
"""

import csv
import json
import math
import time
from dataclasses import asdict, dataclass, field

PENDING, ACTIVE, REACHED, SKIPPED, UNREACHABLE = 0, 1, 2, 3, 4
STATUS_NAMES = {PENDING: 'PENDING', ACTIVE: 'ACTIVE', REACHED: 'REACHED',
                SKIPPED: 'SKIPPED', UNREACHABLE: 'UNREACHABLE'}

IDLE, RUNNING, HOLD, ABORTED, COMPLETE = 0, 1, 2, 3, 4
STATE_NAMES = {IDLE: 'IDLE', RUNNING: 'RUNNING', HOLD: 'HOLD',
               ABORTED: 'ABORTED', COMPLETE: 'COMPLETE'}

START, CMD_HOLD, RESUME, ABORT, RESET = 0, 1, 2, 3, 4
COMMAND_NAMES = {START: 'START', CMD_HOLD: 'HOLD', RESUME: 'RESUME',
                 ABORT: 'ABORT', RESET: 'RESET'}

FILE_VERSION = 1


@dataclass
class Target:
    name: str
    latitude: float
    longitude: float
    tolerance_m: float = 1.5
    status: int = PENDING
    east_m: float = 0.0
    north_m: float = 0.0

    def as_dict(self):
        return asdict(self)


class MissionLog:
    """Everything that happened, in the order it happened.

    One list, because a mission log split by category is a log nobody can
    reconstruct a timeline from. Each entry says what kind of thing it was so
    it can still be filtered.
    """

    KINDS = ('command', 'state', 'target', 'conversion', 'fault', 'info')

    def __init__(self, clock=None):
        self.entries = []
        self._clock = clock or (lambda: None)

    def add(self, kind, detail, **data):
        if kind not in self.KINDS:
            raise ValueError(f'unknown log kind {kind}')
        entry = {
            'wall_time': time.strftime('%Y-%m-%d %H:%M:%S'),
            'ros_time_s': self._clock(),
            'kind': kind,
            'detail': detail,
        }
        entry.update(data)
        self.entries.append(entry)
        return entry

    def export(self, path):
        """Write JSON, and a CSV beside it. Returns the JSON path."""
        path = str(path)
        with open(path, 'w') as handle:
            json.dump({'exported': time.strftime('%Y-%m-%d %H:%M:%S'),
                       'entries': self.entries}, handle, indent=2)
        columns = []
        for entry in self.entries:
            for key in entry:
                if key not in columns:
                    columns.append(key)
        csv_path = path[:-5] + '.csv' if path.endswith('.json') else path + '.csv'
        with open(csv_path, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            writer.writerows(self.entries)
        return path, csv_path


@dataclass
class Mission:
    """The target list and the state machine over it.

    Transitions return (accepted, detail) rather than raising, because every
    one of them is something an operator pressed and the answer has to end up
    in front of them either way.
    """

    targets: list = field(default_factory=list)
    state: int = IDLE
    active_index: int = -1
    started_at: float = 0.0

    # Targets

    def set_targets(self, targets):
        if self.state == RUNNING:
            return False, 'stop or abort the mission before changing targets'
        if not targets:
            self.targets = []
            self.active_index = -1
            return True, 'target list cleared'
        for target in targets:
            if not -90.0 <= target.latitude <= 90.0:
                return False, f'{target.name}: latitude out of range'
            if not -180.0 <= target.longitude <= 180.0:
                return False, f'{target.name}: longitude out of range'
            if target.tolerance_m <= 0.0:
                return False, f'{target.name}: tolerance must be positive'
        self.targets = list(targets)
        for target in self.targets:
            target.status = PENDING
        self.active_index = -1
        self.state = IDLE
        return True, f'{len(self.targets)} targets set'

    def move_target(self, index, delta):
        new = index + delta
        if not (0 <= index < len(self.targets) and 0 <= new < len(self.targets)):
            return False, 'cannot move past the ends of the list'
        if self.state == RUNNING:
            return False, 'stop the mission before reordering'
        self.targets[index], self.targets[new] = self.targets[new], self.targets[index]
        return True, f'moved {self.targets[new].name}'

    # Commands

    def start(self):
        if not self.targets:
            return False, 'no targets'
        if self.state == RUNNING:
            return False, 'already running'
        if all(t.status == REACHED for t in self.targets):
            return False, 'every target already reached, reset first'
        self.state = RUNNING
        if self.active_index < 0:
            self.active_index = self._next_pending()
        self._mark_active()
        return True, f'running, heading for {self.targets[self.active_index].name}'

    def hold(self):
        if self.state != RUNNING:
            return False, f'not running (state {STATE_NAMES[self.state]})'
        self.state = HOLD
        return True, 'holding, rover commanded to stop'

    def resume(self):
        if self.state != HOLD:
            return False, 'not holding'
        self.state = RUNNING
        return True, 'resumed'

    def abort(self):
        if self.state in (IDLE, ABORTED):
            return False, 'nothing to abort'
        self.state = ABORTED
        if 0 <= self.active_index < len(self.targets):
            self.targets[self.active_index].status = PENDING
        self.active_index = -1
        return True, 'aborted, rover commanded to stop'

    def reset(self):
        for target in self.targets:
            target.status = PENDING
        self.state = IDLE
        self.active_index = -1
        return True, 'mission reset'

    def skip_active(self):
        if self.active_index < 0:
            return False, 'no active target'
        target = self.targets[self.active_index]
        target.status = SKIPPED
        name = target.name
        self.active_index = self._next_pending()
        if self.active_index < 0:
            self.state = COMPLETE
            return True, f'{name} skipped, mission complete'
        self._mark_active()
        return True, f'{name} skipped, heading for {self.targets[self.active_index].name}'

    # Progress

    def arrived(self, east, north):
        """Has the rover reached the active target? Advances if so."""
        if self.state != RUNNING or self.active_index < 0:
            return None
        target = self.targets[self.active_index]
        distance = math.hypot(target.east_m - east, target.north_m - north)
        if distance > target.tolerance_m:
            return None
        target.status = REACHED
        reached = target
        self.active_index = self._next_pending()
        if self.active_index < 0:
            self.state = COMPLETE
        else:
            self._mark_active()
        return reached, distance

    def distance_to_active(self, east, north):
        if self.active_index < 0:
            return None
        target = self.targets[self.active_index]
        return math.hypot(target.east_m - east, target.north_m - north)

    def _next_pending(self):
        for index, target in enumerate(self.targets):
            if target.status in (PENDING, ACTIVE):
                return index
        return -1

    def _mark_active(self):
        for index, target in enumerate(self.targets):
            if target.status == ACTIVE and index != self.active_index:
                target.status = PENDING
        if 0 <= self.active_index < len(self.targets):
            self.targets[self.active_index].status = ACTIVE


# --------------------------------------------------------------------------
# Saving and loading a target list
# --------------------------------------------------------------------------

def save_targets(path, targets, origin=None):
    payload = {'version': FILE_VERSION,
               'saved': time.strftime('%Y-%m-%d %H:%M:%S'),
               'origin': list(origin) if origin else None,
               'targets': [{'name': t.name, 'latitude': t.latitude,
                            'longitude': t.longitude, 'tolerance_m': t.tolerance_m}
                           for t in targets]}
    with open(path, 'w') as handle:
        json.dump(payload, handle, indent=2)
    return path


def load_targets(path):
    with open(path) as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict) or 'targets' not in payload:
        raise ValueError('not a target file')
    if payload.get('version') != FILE_VERSION:
        raise ValueError(f'unsupported target file version {payload.get("version")!r}')
    targets = []
    for index, item in enumerate(payload['targets'], start=1):
        try:
            targets.append(Target(name=str(item.get('name') or f'Target {index}'),
                                  latitude=float(item['latitude']),
                                  longitude=float(item['longitude']),
                                  tolerance_m=float(item.get('tolerance_m', 1.5))))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'target {index} is malformed: {exc}') from exc
    return targets


# --------------------------------------------------------------------------
# Is what is on screen still true?
# --------------------------------------------------------------------------

class Freshness:
    """How old each stream is, and what that means.

    One place for the thresholds, because "is this number still true" is
    asked about six different topics on this screen and the answer has to be
    consistent.
    """

    LIVE, STALE, LOST, NEVER = 'live', 'stale', 'lost', 'never'

    def __init__(self, thresholds, clock=time.monotonic):
        self.thresholds = dict(thresholds)   # name -> (stale_s, lost_s)
        self._clock = clock
        self._seen = {}

    def mark(self, name):
        self._seen[name] = self._clock()

    def age(self, name):
        seen = self._seen.get(name)
        return None if seen is None else self._clock() - seen

    def status(self, name):
        age = self.age(name)
        if age is None:
            return self.NEVER, None
        stale, lost = self.thresholds.get(name, (1.0, 4.0))
        if age >= lost:
            return self.LOST, age
        if age >= stale:
            return self.STALE, age
        return self.LIVE, age

    def worst(self):
        """The unhappiest stream, for a single headline indicator."""
        order = {self.LIVE: 0, self.STALE: 1, self.NEVER: 2, self.LOST: 3}
        worst_name, worst_state = None, self.LIVE
        for name in self.thresholds:
            state, _ = self.status(name)
            if order[state] > order[worst_state]:
                worst_name, worst_state = name, state
        return worst_name, worst_state
