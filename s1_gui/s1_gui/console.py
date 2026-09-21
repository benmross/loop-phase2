"""The operator console: a PyQt window that is a ROS 2 node.

It does not touch the rover. Every action goes to the mission manager as a
service call, and everything it shows comes from topics: the manager's
mission state, and the navigation stack's own odometry, path and goal. That
is the separation the challenge asks for, and it has a practical payoff: the
console can be closed and reopened mid-mission and picks the mission back up
from the latched state topic, because the mission never lived in the GUI.

Qt owns the main loop. ROS is pumped from a Qt timer with spin_once, so
callbacks and widgets share one thread and no locking is needed. If the ROS
context dies the window closes itself rather than sitting there looking
connected.
"""

import functools
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout,
                             QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow,
                             QPlainTextEdit, QPushButton, QShortcut, QTableWidget,
                             QTableWidgetItem, QVBoxLayout, QWidget)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from s1_gui import geodesy, widgets
from s1_gui.mission import (ABORT, CMD_HOLD, Freshness, RESET, RESUME, START, STATE_NAMES,
                            STATUS_NAMES, Target, load_targets, save_targets)
from s1_gui.terrain import Terrain, find_worlds_dir
from s1_gui_msgs.msg import MissionState
from s1_gui_msgs.msg import Target as TargetMsg
from s1_gui_msgs.srv import ExportLog, MissionCommand, SetStart, SetTargets

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

FRESHNESS = {'mission_state': (2.0, 6.0), 'odometry': (1.0, 3.0),
             'planned_path': (3.0, 10.0), 'current_goal': (3.0, 10.0)}

WATCHED_NODES = ('mission_manager', 'gps_odometry', 'gps_waypoint_converter',
                 'astar_planner', 'path_controller', 'lidar_mapper')

STATE_COLOURS = {0: widgets.DIM, 1: widgets.OK, 2: widgets.WARN,
                 3: widgets.FAULT, 4: widgets.INFO}


class ConsoleNode(Node):
    """The ROS half of the console: read topics, call the manager."""

    def __init__(self):
        super().__init__('operator_console')
        self.state = None
        self.pose = None
        self.heading = 0.0
        self.path = []
        self.goal = None
        self.trail = []
        self.fresh = Freshness(FRESHNESS)
        self.events = []

        self.create_subscription(MissionState, '/s1_mission/state', self._on_state, LATCHED)
        self.create_subscription(Odometry, '/model/robot/odometry', self._on_odom, SENSOR)
        self.create_subscription(PathMsg, '/planned_path', self._on_path, SENSOR)
        self.create_subscription(PoseStamped, '/current_goal', self._on_goal, SENSOR)

        self.set_targets = self.create_client(SetTargets, '/s1_mission/set_targets')
        self.command = self.create_client(MissionCommand, '/s1_mission/command')
        self.export_log = self.create_client(ExportLog, '/s1_mission/export_log')
        self.set_start = self.create_client(SetStart, '/s1_mission/set_start')

    def event(self, text, level='info'):
        self.events.append((time.strftime('%H:%M:%S'), text, level))

    def _on_state(self, msg):
        self.fresh.mark('mission_state')
        self.state = msg

    def _on_odom(self, msg):
        self.fresh.mark('odometry')
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.pose = (position.x, position.y)
        self.heading = math.degrees(math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)))
        if not self.trail or math.dist(self.trail[-1], self.pose) > 1.0:
            self.trail.append(self.pose)
            del self.trail[:-4000]

    def _on_path(self, msg):
        if msg.poses:
            self.fresh.mark('planned_path')
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_goal(self, msg):
        self.fresh.mark('current_goal')
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def missing_nodes(self):
        running = set(self.get_node_names())
        return [name for name in WATCHED_NODES if name not in running]

    def call(self, client, request, description, done):
        """Call a service without blocking the GUI, and report the answer."""
        if not client.service_is_ready():
            self.event(f'{description}: mission manager is not available', 'fault')
            return False
        future = client.call_async(request)
        future.add_done_callback(functools.partial(self._finished, description, done))
        return True

    def _finished(self, description, done, future):
        try:
            response = future.result()
        except Exception as exc:                                  # noqa: BLE001
            self.event(f'{description} failed: {exc}', 'fault')
            return
        accepted = getattr(response, 'accepted', getattr(response, 'ok', True))
        detail = getattr(response, 'detail', getattr(response, 'path', ''))
        self.event(f'{description}: {detail}', 'ok' if accepted else 'fault')
        if done:
            done(response)


class Console(QMainWindow):

    def __init__(self, node, terrain):
        super().__init__()
        self.node = node
        self.terrain = terrain
        self.origin = terrain.center
        self.targets = []
        self._shown_events = 0
        self._last_state_key = None

        self.setWindowTitle('S1 Operator Console - URC Autonomous Navigation')
        self.resize(1680, 980)
        self.setStyleSheet(widgets.STYLESHEET)
        self._build()

        refresh = QTimer(self)
        refresh.timeout.connect(self._refresh)
        refresh.start(100)
        spin = QTimer(self)
        spin.timeout.connect(self._spin)
        spin.start(10)
        self.node.event(f'console up. Origin {self.origin[0]:.6f}, {self.origin[1]:.6f}, '
                        f'grid convergence {geodesy.grid_convergence_deg(*self.origin):+.3f} deg')

    # ------------------------------------------------------------------ build

    def _build(self):
        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(8)
        outer.addWidget(self._header())

        middle = QHBoxLayout()
        middle.setSpacing(8)
        middle.addWidget(self._targets_panel(), 5)
        middle.addWidget(self._map_panel(), 7)
        middle.addWidget(self._status_panel(), 3)
        outer.addLayout(middle, 1)

        log_panel, log_layout = widgets.panel('MISSION LOG')
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setMinimumHeight(110)
        log_layout.addWidget(self.log)
        outer.addWidget(log_panel)

    def _header(self):
        frame, layout = widgets.panel()
        row = QHBoxLayout()
        row.setSpacing(14)

        title = QLabel('S1 OPERATOR CONSOLE')
        title.setStyleSheet('font-size: 15px; font-weight: 700; letter-spacing: 1px;')
        row.addWidget(title)

        self.state_label = QLabel('STATE --')
        self.state_label.setObjectName('big')
        row.addWidget(self.state_label)
        self.elapsed_label = QLabel('00:00')
        self.elapsed_label.setObjectName('big')
        row.addWidget(self.elapsed_label)
        row.addStretch(1)

        # Keyboard as well as mouse. An operator watching the map should not
        # have to find a button to stop the rover, and the space bar is the
        # one key everyone hits under pressure.
        for text, command, style, key in (
                ('START  (F5)', START, 'go', 'F5'),
                ('STOP / HOLD  (space)', CMD_HOLD, '', 'Space'),
                ('RESUME  (F6)', RESUME, '', 'F6'),
                ('RESET  (F8)', RESET, '', 'F8'),
                ('ABORT  (Esc)', ABORT, 'stop', 'Escape')):
            button = QPushButton(text)
            if style:
                button.setObjectName(style)
            button.clicked.connect(functools.partial(self._send_command, command))
            row.addWidget(button)
            QShortcut(QKeySequence(key), self,
                      functools.partial(self._send_command, command))
        layout.addLayout(row)
        return frame

    def _targets_panel(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        entry, entry_layout = widgets.panel('TARGET ENTRY (WGS 84: DD, DDM or DMS)')
        grid = QGridLayout()
        grid.setSpacing(5)
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText('name, optional')
        self.lat_edit = QLineEdit()
        self.lat_edit.setPlaceholderText('38.424 340   |   38 25.460 N   |   38 25 27.6 N')
        self.lon_edit = QLineEdit()
        self.lon_edit.setPlaceholderText('-110.782 100  |  110 46.926 W  |  110 46 55.6 W')
        self.tolerance_spin = QDoubleSpinBox()
        self.tolerance_spin.setRange(0.3, 50.0)
        self.tolerance_spin.setValue(2.0)
        self.tolerance_spin.setSuffix(' m tolerance')
        for row, (label, widget) in enumerate((('Name', self.name_edit),
                                               ('Latitude', self.lat_edit),
                                               ('Longitude', self.lon_edit),
                                               ('Arrival', self.tolerance_spin))):
            caption = QLabel(label.upper())
            caption.setObjectName('title')
            grid.addWidget(caption, row, 0)
            grid.addWidget(widget, row, 1)
        entry_layout.addLayout(grid)

        self.parse_label = QLabel('type a coordinate')
        self.parse_label.setStyleSheet(f'color: {widgets.DIM}; font-size: 11px;')
        self.parse_label.setWordWrap(True)
        entry_layout.addWidget(self.parse_label)
        self.lat_edit.textChanged.connect(self._preview)
        self.lon_edit.textChanged.connect(self._preview)

        buttons = QHBoxLayout()
        for text, handler in (('Add target', self._add_target),
                              ('Update selected', self._update_selected),
                              ('Use as start fix', self._use_as_start)):
            button = QPushButton(text)
            button.clicked.connect(handler)
            buttons.addWidget(button)
        entry_layout.addLayout(buttons)
        layout.addWidget(entry)

        table_panel, table_layout = widgets.panel('TARGETS')
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(['#', 'NAME', 'LATITUDE', 'LONGITUDE', 'TOL', 'STATUS'])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.itemSelectionChanged.connect(self._selection_changed)
        table_layout.addWidget(self.table)

        row = QHBoxLayout()
        for text, handler in (('Up', lambda: self._move(-1)), ('Down', lambda: self._move(1)),
                              ('Delete', self._delete), ('Save...', self._save),
                              ('Load...', self._load)):
            button = QPushButton(text)
            button.clicked.connect(handler)
            row.addWidget(button)
        table_layout.addLayout(row)

        send = QPushButton('SEND TARGETS TO MISSION')
        send.setObjectName('go')
        send.clicked.connect(self._send_targets)
        table_layout.addWidget(send)
        layout.addWidget(table_panel, 1)
        return container

    def _map_panel(self):
        frame, layout = widgets.panel('MDRS AREA, OFFLINE (USGS 1 m elevation model)')
        controls = QHBoxLayout()
        self.layer_box = QComboBox()
        self.layer_box.addItems(['Terrain (hillshade)', 'Slope / traversability'])
        self.layer_box.currentIndexChanged.connect(
            lambda index: self.map.set_layer('hillshade' if index == 0 else 'slope'))
        controls.addWidget(self.layer_box)
        self.zoom_box = QComboBox()
        self.zoom_box.addItems(['Whole 2 km square', '500 m around rover',
                                '150 m around rover', '40 m around rover'])
        self.zoom_box.currentIndexChanged.connect(
            lambda index: self.map.set_view_span([None, 500.0, 150.0, 40.0][index]))
        controls.addWidget(self.zoom_box)
        controls.addStretch(1)
        self.cursor_label = QLabel('')
        self.cursor_label.setStyleSheet(f'color: {widgets.DIM}; font-family: {widgets.MONO};'
                                        'font-size: 10px;')
        controls.addWidget(self.cursor_label)
        layout.addLayout(controls)

        self.map = widgets.MapView(self.terrain, self.origin)
        self.map.cursor_moved.connect(self._cursor)
        layout.addWidget(self.map, 1)
        return frame

    def _status_panel(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        mission_panel, mission_layout = widgets.panel('MISSION')
        self.active_tile = widgets.Tile('active target', 'name and distance')
        self.position_tile = widgets.Tile('rover position', 'WGS 84')
        self.local_tile = widgets.Tile('local frame', 'east, north (REP 103)')
        self.heading_tile = widgets.Tile('heading', 'degrees, 0 = north')
        for tile in (self.active_tile, self.position_tile, self.local_tile, self.heading_tile):
            mission_layout.addWidget(tile)
        layout.addWidget(mission_panel)

        link_panel, link_layout = widgets.panel('ROS DATA')
        self.fresh_labels = {}
        for name in FRESHNESS:
            label = QLabel(name)
            label.setStyleSheet(f'font-family: {widgets.MONO}; font-size: 11px;')
            link_layout.addWidget(label)
            self.fresh_labels[name] = label
        self.nodes_label = QLabel('nodes: checking')
        self.nodes_label.setWordWrap(True)
        self.nodes_label.setStyleSheet(f'font-size: 11px; color: {widgets.DIM};')
        link_layout.addWidget(self.nodes_label)
        layout.addWidget(link_panel)

        export = QPushButton('EXPORT MISSION LOG')
        export.clicked.connect(self._export)
        layout.addWidget(export)
        layout.addStretch(1)
        return container

    # ----------------------------------------------------------------- inputs

    def _parse_entry(self):
        return geodesy.parse_latlon(self.lat_edit.text(), self.lon_edit.text())

    def _preview(self):
        if not self.lat_edit.text().strip() or not self.lon_edit.text().strip():
            self.parse_label.setText('type a coordinate')
            self.parse_label.setStyleSheet(f'color: {widgets.DIM}; font-size: 11px;')
            return
        try:
            latitude, longitude, style = self._parse_entry()
        except geodesy.CoordinateError as exc:
            self.parse_label.setText(f'rejected: {exc}')
            self.parse_label.setStyleSheet(f'color: {widgets.FAULT}; font-size: 11px;')
            return
        east, north = geodesy.latlon_to_local(latitude, longitude, *self.origin)
        inside = max(abs(east), abs(north)) <= self.terrain.extent_m[0] / 2
        self.parse_label.setText(
            f'{style}: {geodesy.format_latlon(latitude, longitude, "DD")}  |  '
            f'{geodesy.format_latlon(latitude, longitude, "DMS")}  ->  '
            f'east {east:+.1f} m, north {north:+.1f} m'
            + ('' if inside else '  OUTSIDE THE MAPPED SQUARE'))
        self.parse_label.setStyleSheet(
            f'color: {widgets.OK if inside else widgets.WARN}; font-size: 11px;')

    def _entry_target(self):
        try:
            latitude, longitude, style = self._parse_entry()
        except geodesy.CoordinateError as exc:
            self.node.event(f'coordinate rejected: {exc}', 'fault')
            self._preview()
            return None
        name = self.name_edit.text().strip() or f'Target {len(self.targets) + 1}'
        self.node.event(f'{name}: accepted {style} coordinate '
                        f'{geodesy.format_latlon(latitude, longitude)}')
        return Target(name=name, latitude=latitude, longitude=longitude,
                      tolerance_m=self.tolerance_spin.value())

    def _add_target(self):
        target = self._entry_target()
        if target is None:
            return
        self.targets.append(target)
        self._fill_table()
        self.name_edit.clear()

    def _update_selected(self):
        row = self.table.currentRow()
        if row < 0 or row >= len(self.targets):
            self.node.event('no target selected', 'warn')
            return
        target = self._entry_target()
        if target is None:
            return
        self.targets[row] = target
        self._fill_table()

    def _selection_changed(self):
        row = self.table.currentRow()
        if 0 <= row < len(self.targets):
            target = self.targets[row]
            self.name_edit.setText(target.name)
            self.lat_edit.setText(f'{target.latitude:.8f}')
            self.lon_edit.setText(f'{target.longitude:.8f}')
            self.tolerance_spin.setValue(target.tolerance_m)

    def _move(self, delta):
        row = self.table.currentRow()
        new = row + delta
        if row < 0 or not 0 <= new < len(self.targets):
            return
        self.targets[row], self.targets[new] = self.targets[new], self.targets[row]
        self._fill_table()
        self.table.selectRow(new)

    def _delete(self):
        row = self.table.currentRow()
        if 0 <= row < len(self.targets):
            self.node.event(f'{self.targets.pop(row).name} removed')
            self._fill_table()

    def _save(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Save targets',
                                              os.path.expanduser('~/targets.json'),
                                              'Target lists (*.json)')
        if not path:
            return
        save_targets(path, self.targets, self.origin)
        self.node.event(f'saved {len(self.targets)} targets to {path}', 'ok')

    def _load(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Load targets',
                                              os.path.expanduser('~'), 'Target lists (*.json)')
        if not path:
            return
        try:
            self.targets = load_targets(path)
        except (OSError, ValueError) as exc:
            self.node.event(f'could not load {path}: {exc}', 'fault')
            return
        self._fill_table()
        self.node.event(f'loaded {len(self.targets)} targets from {path}', 'ok')

    def _fill_table(self):
        self.table.setRowCount(len(self.targets))
        for row, target in enumerate(self.targets):
            values = [str(row + 1), target.name, f'{target.latitude:.7f}',
                      f'{target.longitude:.7f}', f'{target.tolerance_m:.1f}',
                      STATUS_NAMES.get(target.status, '?')]
            for column, value in enumerate(values):
                item = self.table.item(row, column) or QTableWidgetItem()
                item.setText(value)
                self.table.setItem(row, column, item)

    # --------------------------------------------------------------- commands

    def _send_targets(self):
        request = SetTargets.Request()
        request.issued_by = 'console'
        for target in self.targets:
            message = TargetMsg()
            message.name = target.name
            message.latitude = target.latitude
            message.longitude = target.longitude
            message.tolerance_m = float(target.tolerance_m)
            request.targets.append(message)
        self.node.call(self.node.set_targets, request, f'send {len(self.targets)} targets', None)

    def _send_command(self, command):
        request = MissionCommand.Request()
        request.command = command
        request.issued_by = 'console'
        names = {START: 'START', CMD_HOLD: 'STOP', RESUME: 'RESUME', ABORT: 'ABORT',
                 RESET: 'RESET'}
        self.node.call(self.node.command, request, names.get(command, 'command'), None)

    def _use_as_start(self):
        target = self._entry_target()
        if target is None:
            return
        request = SetStart.Request()
        request.latitude = target.latitude
        request.longitude = target.longitude
        request.altitude = 0.0
        request.issued_by = 'console'
        self.node.call(self.node.set_start, request, 'set start fix',
                       lambda response: self.node.trail.clear())

    def _export(self):
        request = ExportLog.Request()
        request.path = ''
        self.node.call(self.node.export_log, request, 'export log', None)

    def _cursor(self, latitude, longitude, elevation, slope):
        self.cursor_label.setText(
            f'{geodesy.format_latlon(latitude, longitude)}   {elevation:7.1f} m   '
            f'slope {slope:4.1f} deg')

    # ---------------------------------------------------------------- refresh

    def _spin(self):
        if not rclpy.ok():
            QApplication.quit()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:                                  # noqa: BLE001
            print(f'console closing: {exc}', file=sys.stderr)
            QApplication.quit()

    def _refresh(self):
        for stamp, text, level in self.node.events[self._shown_events:]:
            colour = {'ok': widgets.OK, 'warn': widgets.WARN, 'fault': widgets.FAULT}.get(
                level, widgets.TEXT)
            self.log.appendHtml(f'<span style="color:{widgets.DIM}">{stamp}</span> '
                                f'<span style="color:{colour}">{text}</span>')
        self._shown_events = len(self.node.events)

        state = self.node.state
        odom_state, odom_age = self.node.fresh.status('odometry')
        rover_stale = odom_state in (Freshness.LOST, Freshness.NEVER)

        if state is not None:
            self.state_label.setText(f'STATE {STATE_NAMES.get(state.state, "?")}')
            self.state_label.setStyleSheet(
                f'color: {STATE_COLOURS.get(state.state, widgets.TEXT)};'
                'font-family: DejaVu Sans Mono; font-size: 20px; font-weight: 700;')
            self.elapsed_label.setText(f'{int(state.elapsed_s // 60):02d}:'
                                       f'{int(state.elapsed_s % 60):02d}')
            key = (state.state, state.active_index,
                   tuple((t.name, t.status) for t in state.targets))
            if key != self._last_state_key:
                self._last_state_key = key
                if state.targets:
                    # The manager is the authority on status once targets are sent.
                    for row, target in enumerate(state.targets):
                        if row < len(self.targets):
                            self.targets[row].status = target.status
                            self.targets[row].east_m = target.east_m
                            self.targets[row].north_m = target.north_m
                    self._fill_table()
            if 0 <= state.active_index < len(state.targets) and self.node.pose and not rover_stale:
                target = state.targets[state.active_index]
                distance = math.hypot(target.east_m - self.node.pose[0],
                                      target.north_m - self.node.pose[1])
                self.active_tile.set(f'{distance:0.1f} m', widgets.INFO,
                                     f'{target.name}, tolerance {target.tolerance_m:.1f} m')
            else:
                self.active_tile.set(STATE_NAMES.get(state.state, '--'), widgets.DIM,
                                     state.detail[:48])

        if self.node.pose and not rover_stale:
            east, north = self.node.pose
            latitude, longitude = geodesy.local_to_latlon(east, north, *self.origin)
            self.position_tile.set(f'{latitude:.6f}', widgets.TEXT,
                                   f'{longitude:.6f}  ({geodesy.format_angle(latitude, "lat", "DMS")})')
            self.local_tile.set(f'{east:+.1f}, {north:+.1f}', widgets.TEXT, 'metres east, north')
            self.heading_tile.set(f'{(self.node.heading + 360) % 360:05.1f}', widgets.TEXT,
                                  'degrees from north')
            self.map.set_rover(east, north, 90.0 - self.node.heading, stale=False)
        else:
            for tile in (self.position_tile, self.local_tile, self.heading_tile):
                tile.blank()
            if self.node.pose:
                self.map.set_rover(self.node.pose[0], self.node.pose[1],
                                   90.0 - self.node.heading, stale=True)

        self.map.set_mission(self.targets if not state or not state.targets else
                             [Target(name=t.name, latitude=t.latitude, longitude=t.longitude,
                                     tolerance_m=t.tolerance_m, status=t.status,
                                     east_m=t.east_m, north_m=t.north_m)
                              for t in state.targets],
                             self.node.goal)
        self.map.set_path(self.node.path)
        self.map.set_trail(self.node.trail)

        for name, label in self.fresh_labels.items():
            status, age = self.node.fresh.status(name)
            colour = {Freshness.LIVE: widgets.OK, Freshness.STALE: widgets.WARN,
                      Freshness.LOST: widgets.FAULT, Freshness.NEVER: widgets.DIM}[status]
            age_text = 'never received' if age is None else f'{age:4.1f} s'
            label.setText(f'{name:<14} {status.upper():<5} {age_text}')
            label.setStyleSheet(f'color: {colour}; font-family: {widgets.MONO}; font-size: 11px;')

        missing = self.node.missing_nodes()
        if missing:
            self.nodes_label.setText('NOT RUNNING: ' + ', '.join(missing))
            self.nodes_label.setStyleSheet(f'color: {widgets.FAULT}; font-size: 11px;')
        else:
            self.nodes_label.setText('all expected nodes running')
            self.nodes_label.setStyleSheet(f'color: {widgets.OK}; font-size: 11px;')


def main():
    rclpy.init()
    node = ConsoleNode()
    try:
        terrain = Terrain(find_worlds_dir(os.environ.get('S1_TERRAIN_DIR')))
    except FileNotFoundError as exc:
        print(f'Cannot load the offline map: {exc}', file=sys.stderr)
        raise SystemExit(2)
    app = QApplication(sys.argv)
    window = Console(node, terrain)
    window.show()
    try:
        app.exec_()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
