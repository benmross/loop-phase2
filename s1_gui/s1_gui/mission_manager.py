"""Mission management: the node between the operator and the navigation stack.

The challenge asks for separation between the GUI, mission management and
rover control. This is the middle one. It owns the target list and the
mission state, and it is the only thing in this package that talks to
``s1_navigation``. The GUI never publishes a rover topic: it calls services
here and reads one state topic back. That way the mission survives the GUI
being closed, restarted, or run on another machine, which is what "the
operator console is the only view" has to mean if the console can crash.

What it says to Matthew's stack, and why each one:

/robot_gps_start_location   NavSatFix, latched, repeated at 1 Hz
    The supplied start coordinate. His gps_odometry converts it and teleports
    the Gazebo model there, which is what associates the start coordinate
    with the spawn position. Publishing it here replaces his
    gps_start_publisher; run one or the other, never both, or the two fight
    over where the rover is.

/gps_waypoints              Float64MultiArray, latched, repeated at 1 Hz
    The targets, as degrees, in his layout: three latitude/longitude pairs.
    His gps_waypoint_converter turns them into /waypoints in the local frame,
    spawns the posts in Gazebo, and his planner drives them. The conversion
    to metres deliberately stays on his side; this node converts too, but
    only to check his answer and to have something to draw and measure
    arrival against.

/model/robot/cmd_vel + /planned_path   only while holding or aborted
    There is no stop interface in the navigation stack yet, so a hold has to
    be enforced from outside: an empty path, which makes his path_controller
    stop following, plus zero velocity at 20 Hz so nothing that is still
    publishing can creep the rover forward. It is an override, and it is
    written down as one. integration/astar_planner.patch offers the clean
    version for his side.
"""

import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from s1_gui import geodesy
from s1_gui.mission import (ABORT, ABORTED, CMD_HOLD, COMMAND_NAMES, COMPLETE, Freshness, HOLD,
                            Mission, MissionLog, RESET, RESUME, RUNNING, START, STATE_NAMES,
                            Target, UNREACHABLE)
from s1_gui_msgs.msg import MissionState
from s1_gui_msgs.msg import Target as TargetMsg
from s1_gui_msgs.srv import ExportLog, MissionCommand, SetStart, SetTargets

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

# name -> (stale after, lost after), in seconds
FRESHNESS = {
    'odometry': (1.0, 3.0),
    'planned_path': (2.0, 6.0),
    'current_goal': (2.0, 6.0),
    'costmap': (5.0, 15.0),
    'scan': (1.0, 3.0),
    'waypoints': (999.0, 9999.0),     # latched and rare: only "never" matters
}

REQUIRED_NODES = ('gps_odometry', 'astar_planner', 'path_controller', 'lidar_mapper')


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        param = self.declare_parameter
        self.origin = (param('origin_latitude', 38.42287240335025).value,
                       param('origin_longitude', -110.78495572815902).value)
        self.square_m = param('terrain_half_width_m', 1000.0).value
        self.expected_targets = param('expected_targets', 3).value
        # The flat obstacle world has no GPS chain: gps_waypoint_converter is
        # not running there, so nothing would turn our degrees into metres.
        # With this on, the manager publishes the local PoseArray itself, in
        # the same frame and QoS his planner already subscribes to.
        self.publish_local = param('publish_local_waypoints', False).value
        self.default_tolerance = param('default_tolerance_m', 2.0).value
        # The planner refuses to plan outside its own world bounds, so a
        # target beyond them is unreachable no matter how good the coordinate
        # is. Defaults to the terrain square; the flat world is +/-10 m.
        self.plan_limit = param('planner_world_limit_m', 1000.0).value
        # How long a running mission may go with no path before the active
        # target is called unreachable. Long enough to cover a replan, short
        # enough that an operator is not left watching a stationary rover.
        self.unreachable_after = param('unreachable_after_s', 15.0).value
        self.log_dir = Path(os.path.expanduser(param('log_dir', '~/.ros/s1_gui_logs').value))
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.mission = Mission()
        self.log = MissionLog(clock=self._ros_seconds)
        self.fresh = Freshness(FRESHNESS)
        self.pose = None
        self.heading = 0.0
        self.trail = []
        self.start_fix = None
        self.waypoint_message = None
        self.converter_poses = None
        self.current_goal = None
        self.detail = 'waiting for targets'
        self.started_at = None
        self.override_until = 0.0
        self.last_path_at = None

        group = ReentrantCallbackGroup()
        self.start_pub = self.create_publisher(NavSatFix, '/robot_gps_start_location', LATCHED)
        self.waypoint_pub = self.create_publisher(Float64MultiArray, '/gps_waypoints', LATCHED)
        self.local_waypoint_pub = self.create_publisher(PoseArray, '/waypoints', LATCHED)
        self.state_pub = self.create_publisher(MissionState, '/s1_mission/state', LATCHED)
        self.cmd_pub = self.create_publisher(Twist, '/model/robot/cmd_vel', 10)
        self.path_override_pub = self.create_publisher(PathMsg, '/planned_path', 10)

        self.create_subscription(Odometry, '/model/robot/odometry', self._on_odom, SENSOR)
        self.create_subscription(PoseArray, '/waypoints', self._on_converter_waypoints, LATCHED)
        self.create_subscription(PathMsg, '/planned_path', self._on_path, SENSOR)
        self.create_subscription(PoseStamped, '/current_goal', self._on_goal, SENSOR)
        self.create_subscription(OccupancyGrid, '/costmap', self._on_costmap, LATCHED)
        self.create_subscription(LaserScan, '/scan', self._on_scan, SENSOR)

        self.create_service(SetTargets, '/s1_mission/set_targets', self._set_targets, callback_group=group)
        self.create_service(MissionCommand, '/s1_mission/command', self._command, callback_group=group)
        self.create_service(ExportLog, '/s1_mission/export_log', self._export, callback_group=group)
        self.create_service(SetStart, '/s1_mission/set_start', self._set_start, callback_group=group)

        self.create_timer(0.2, self._publish_state)
        self.create_timer(1.0, self._repeat_latched)
        self.create_timer(0.05, self._enforce_stop)
        self.create_timer(1.0, self._check_progress)

        self.log.add('info', f'mission manager up, origin {self.origin[0]:.8f}, '
                             f'{self.origin[1]:.8f}')
        self.get_logger().info(
            f'Mission manager ready. Origin {self.origin[0]:.8f}, {self.origin[1]:.8f}; '
            f'grid convergence {geodesy.grid_convergence_deg(*self.origin):+.3f} deg')

    def _ros_seconds(self):
        return round(self.get_clock().now().nanoseconds * 1e-9, 3)

    # ---------------------------------------------------------------- inputs

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

        arrival = self.mission.arrived(*self.pose)
        if arrival:
            target, distance = arrival
            self.log.add('target', f'{target.name} reached', name=target.name,
                         distance_m=round(distance, 3), tolerance_m=target.tolerance_m,
                         east_m=round(target.east_m, 3), north_m=round(target.north_m, 3))
            self.get_logger().info(f'{target.name} reached, {distance:.2f} m from it')
            if self.mission.state == COMPLETE:
                self.detail = 'all targets reached'
                self.log.add('state', 'mission complete',
                             elapsed_s=self._elapsed())
            else:
                self.detail = f'heading for {self.mission.targets[self.mission.active_index].name}'

    def _on_converter_waypoints(self, msg):
        """What his converter made of our coordinates. Checked, not trusted."""
        self.fresh.mark('waypoints')
        if self.publish_local:
            return      # this is our own message coming back
        self.converter_poses = [(p.position.x, p.position.y) for p in msg.poses]
        for index, (east, north) in enumerate(self.converter_poses):
            if index >= len(self.mission.targets):
                break
            target = self.mission.targets[index]
            delta = math.hypot(target.east_m - east, target.north_m - north)
            self.log.add('conversion', f'{target.name}: converter agrees to {delta:.3f} m',
                         name=target.name, gui_east_m=round(target.east_m, 3),
                         gui_north_m=round(target.north_m, 3),
                         converter_east_m=round(east, 3), converter_north_m=round(north, 3),
                         difference_m=round(delta, 3))
            if delta > 0.5:
                self.get_logger().warning(
                    f'{target.name}: our conversion and the converter differ by {delta:.2f} m')

    def _on_path(self, msg):
        if msg.poses:                      # our own empty overrides do not count
            self.fresh.mark('planned_path')
            self.last_path_at = time.monotonic()

    def _on_goal(self, msg):
        self.fresh.mark('current_goal')
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)

    def _on_costmap(self, msg):
        self.fresh.mark('costmap')

    def _on_scan(self, msg):
        self.fresh.mark('scan')

    # -------------------------------------------------------------- services

    def _set_targets(self, request, response):
        targets = []
        outside = []
        for index, item in enumerate(request.targets, start=1):
            name = item.name or f'Target {index}'
            tolerance = item.tolerance_m or self.default_tolerance
            target = Target(name=name, latitude=item.latitude, longitude=item.longitude,
                            tolerance_m=tolerance)
            east, north = geodesy.latlon_to_local(target.latitude, target.longitude, *self.origin)
            target.east_m, target.north_m = east, north
            if max(abs(east), abs(north)) > self.square_m:
                response.accepted = False
                response.detail = (f'{name} is {max(abs(east), abs(north)):.0f} m from the origin, '
                                   f'outside the {self.square_m:.0f} m terrain square')
                self.log.add('fault', response.detail, name=name)
                return response
            self.log.add('conversion', f'{name}: {target.latitude:.8f}, {target.longitude:.8f} '
                                       f'-> east {east:.2f} m, north {north:.2f} m',
                         name=name, latitude=target.latitude, longitude=target.longitude,
                         east_m=round(east, 3), north_m=round(north, 3))
            if max(abs(east), abs(north)) > self.plan_limit:
                warning = (f'{name} is {max(abs(east), abs(north)):.0f} m out, beyond the '
                           f'planner limit of {self.plan_limit:.0f} m: it will not be planned to')
                self.log.add('fault', warning, name=name)
                self.get_logger().warning(warning)
                outside.append(name)
            targets.append(target)

        accepted, detail = self.mission.set_targets(targets)
        if accepted and outside:
            detail += f'. Outside the planner limit: {", ".join(outside)}'
        if accepted and targets and len(targets) != self.expected_targets:
            detail += (f'. Note: gps_waypoint_converter expects exactly '
                       f'{self.expected_targets} pairs and will reject this list')
        response.accepted, response.detail = accepted, detail
        self.detail = detail
        self.log.add('command', f'set_targets by {request.issued_by or "operator"}: {detail}',
                     count=len(targets), accepted=accepted)
        self._publish_state()
        return response

    def _command(self, request, response):
        handlers = {START: self._start, CMD_HOLD: self._hold, RESUME: self._resume,
                    ABORT: self._abort, RESET: self._reset}
        handler = handlers.get(request.command)
        if handler is None:
            response.accepted, response.detail = False, f'unknown command {request.command}'
            return response
        accepted, detail = handler()
        response.accepted, response.detail = accepted, detail
        self.detail = detail
        self.log.add('command',
                     f'{COMMAND_NAMES.get(request.command, "?")} by '
                     f'{request.issued_by or "operator"}: {detail}',
                     accepted=accepted, state=STATE_NAMES[self.mission.state])
        self.get_logger().info(f'{COMMAND_NAMES.get(request.command)}: {detail}')
        self._publish_state()
        return response

    def _export(self, request, response):
        path = request.path or str(self.log_dir / f'mission-{time.strftime("%Y%m%d-%H%M%S")}.json')
        try:
            json_path, csv_path = self.log.export(path)
        except OSError as exc:
            response.ok, response.path, response.entries = False, str(exc), 0
            return response
        response.ok, response.path = True, json_path
        response.entries = len(self.log.entries)
        self.get_logger().info(f'Mission log exported: {json_path} and {csv_path}')
        return response

    def _set_start(self, request, response):
        if not (-90.0 <= request.latitude <= 90.0 and -180.0 <= request.longitude <= 180.0):
            response.accepted, response.detail = False, 'coordinate out of range'
            return response
        if self.mission.state == RUNNING:
            response.accepted, response.detail = False, 'stop the mission before moving the rover'
            return response
        east, north = self.set_start_fix(request.latitude, request.longitude, request.altitude)
        response.accepted = True
        response.east_m, response.north_m = east, north
        response.detail = (f'start fix published, local east {east:.1f} m, north {north:.1f} m')
        self.detail = response.detail
        self.log.add('command', f'set_start by {request.issued_by or "operator"}: '
                                f'{request.latitude:.8f}, {request.longitude:.8f}')
        self._publish_state()
        return response

    # -------------------------------------------------------------- commands

    def _start(self):
        accepted, detail = self.mission.start()
        if accepted:
            self.started_at = time.monotonic()
            self.override_until = 0.0
            self.last_path_at = time.monotonic()
            self._publish_waypoints()
        return accepted, detail

    def _hold(self):
        accepted, detail = self.mission.hold()
        if accepted:
            self._begin_override()
        return accepted, detail

    def _resume(self):
        accepted, detail = self.mission.resume()
        if accepted:
            self.override_until = 0.0
        return accepted, detail

    def _abort(self):
        accepted, detail = self.mission.abort()
        if accepted:
            self._begin_override()
        return accepted, detail

    def _reset(self):
        accepted, detail = self.mission.reset()
        self.started_at = None
        self.trail.clear()
        self.override_until = 0.0
        return accepted, detail

    def _begin_override(self):
        """Hold the rover: empty path first, then zero velocity until resumed."""
        self.override_until = math.inf
        self.path_override_pub.publish(PathMsg(header=self._header()))

    def _check_progress(self):
        """A running mission with no path is a target the planner cannot reach.

        The planner clears the path when the goal is outside its world bounds
        or no route exists. From outside, that looks exactly like a rover that
        has stopped for no reason, so the manager names it and moves on
        instead of leaving the operator watching a stationary rover.
        """
        if self.mission.state != RUNNING or self.mission.active_index < 0:
            return
        if self.last_path_at is None:
            self.last_path_at = time.monotonic()
            return
        if time.monotonic() - self.last_path_at < self.unreachable_after:
            return
        target = self.mission.targets[self.mission.active_index]
        target.status = UNREACHABLE
        detail = (f'{target.name}: no path for {self.unreachable_after:.0f} s, '
                  f'marking it unreachable and moving on')
        self.log.add('fault', detail, name=target.name,
                     east_m=round(target.east_m, 2), north_m=round(target.north_m, 2))
        self.get_logger().warning(detail)
        self.mission.active_index = self.mission._next_pending()
        self.last_path_at = time.monotonic()
        if self.mission.active_index < 0:
            self.mission.state = COMPLETE
            self.detail = 'mission finished, with targets the planner could not reach'
        else:
            self.mission._mark_active()
            self.detail = f'heading for {self.mission.targets[self.mission.active_index].name}'
            self._publish_waypoints()
        self._publish_state()

    def _enforce_stop(self):
        # 20 Hz of zero velocity while held or aborted. His path_controller
        # publishes at the same rate, so this has to be as frequent to win.
        if self.mission.state in (HOLD, ABORTED) and self.override_until:
            self.cmd_pub.publish(Twist())

    # --------------------------------------------------------------- outputs

    def set_start_fix(self, latitude, longitude, altitude=0.0):
        fix = NavSatFix()
        fix.header.frame_id = 'gps'
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.latitude, fix.longitude, fix.altitude = latitude, longitude, altitude
        fix.status.status = 0
        fix.status.service = 1
        self.start_fix = fix
        self.start_pub.publish(fix)
        east, north = geodesy.latlon_to_local(latitude, longitude, *self.origin)
        self.log.add('conversion', f'start fix {latitude:.8f}, {longitude:.8f} '
                                   f'-> east {east:.2f} m, north {north:.2f} m',
                     latitude=latitude, longitude=longitude,
                     east_m=round(east, 3), north_m=round(north, 3))
        self.trail.clear()
        return east, north

    def _publish_waypoints(self):
        """Targets as his converter wants them: degrees, lat/lon pairs."""
        if not self.mission.targets:
            return
        data = []
        for target in self.mission.targets:
            data.extend([target.latitude, target.longitude])
        message = Float64MultiArray()
        message.layout.dim = [
            MultiArrayDimension(label='waypoints', size=len(self.mission.targets),
                                stride=len(data)),
            MultiArrayDimension(label='latitude_longitude_degrees', size=2, stride=2)]
        message.data = data
        self.waypoint_message = message
        self.waypoint_pub.publish(message)
        self.log.add('info', f'published {len(self.mission.targets)} targets on /gps_waypoints')

        if self.publish_local:
            poses = PoseArray()
            poses.header.frame_id = 'odom'
            poses.header.stamp = self.get_clock().now().to_msg()
            for target in self.mission.targets:
                pose = Pose()
                pose.position.x = target.east_m
                pose.position.y = target.north_m
                pose.orientation.w = 1.0
                poses.poses.append(pose)
            self.local_waypoint_pub.publish(poses)
            self.log.add('info', f'published {len(poses.poses)} targets on /waypoints '
                                 f'(local frame, no GPS converter in this world)')

    def _repeat_latched(self):
        # His nodes are volatile subscribers in places, and repeat their own
        # messages for the same reason: a subscriber that joins late still
        # needs the mission.
        if self.start_fix is not None:
            self.start_fix.header.stamp = self.get_clock().now().to_msg()
            self.start_pub.publish(self.start_fix)
        if self.waypoint_message is not None and self.mission.state == RUNNING:
            self.waypoint_pub.publish(self.waypoint_message)
        if self.override_until:
            self.path_override_pub.publish(PathMsg(header=self._header()))

    def _header(self):
        message = PathMsg().header
        message.frame_id = 'odom'
        message.stamp = self.get_clock().now().to_msg()
        return message

    def _elapsed(self):
        return 0.0 if self.started_at is None else round(time.monotonic() - self.started_at, 1)

    def _publish_state(self):
        message = MissionState()
        message.header.frame_id = 'odom'
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = self.mission.state
        message.active_index = self.mission.active_index
        message.elapsed_s = float(self._elapsed())
        message.origin_latitude, message.origin_longitude = self.origin
        message.detail = self.detail
        for target in self.mission.targets:
            out = TargetMsg()
            out.name = target.name
            out.latitude, out.longitude = target.latitude, target.longitude
            out.tolerance_m = float(target.tolerance_m)
            out.status = target.status
            out.east_m, out.north_m = target.east_m, target.north_m
            message.targets.append(out)
        self.state_pub.publish(message)


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
