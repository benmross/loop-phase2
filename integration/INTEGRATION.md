# Working against `s1_navigation`

Notes for Matthew, and for anyone reading the repository later. This is the
operator half; the rover, planner and controller are in
`MatthewChoulas/Challenge_Week`, branch `phase2`. Nothing here modifies that
repository.

## What this half sends

| Topic | Type | QoS | Consumed by |
|---|---|---|---|
| `/robot_gps_start_location` | `sensor_msgs/NavSatFix` | reliable, transient local, repeated 1 Hz | `gps_odometry`, which converts it and moves the model |
| `/gps_waypoints` | `std_msgs/Float64MultiArray` | reliable, transient local, repeated 1 Hz | `gps_waypoint_converter`, which converts and spawns the posts |
| `/waypoints` | `geometry_msgs/PoseArray` | reliable, transient local | `astar_planner`, **only** when `publish_local_waypoints` is true |
| `/model/robot/cmd_vel`, `/planned_path` | `Twist`, `Path` | default | override during hold and abort, see below |

The array layout matches `gps_waypoint_publisher` exactly: dimension
`waypoints` then `latitude_longitude_degrees`, data as latitude, longitude
pairs in degrees.

**Run either `gps_start_publisher` or this manager, not both.** Both publish
`/robot_gps_start_location`, and two different fixes will fight over where the
rover is. Commenting that node out of `terrain.launch.py` is the tidiest fix
once the GUI is the source of the start coordinate.

## What this half reads

`/model/robot/odometry` for position and heading, `/waypoints` for what the
converter made of the coordinates, `/planned_path` and `/current_goal` to draw
the plan and to tell whether the planner is making progress, `/costmap` and
`/scan` for freshness only.

The manager logs its own WGS 84 to local conversion and compares it with the
converter's answer on `/waypoints`, and warns above 0.5 m. They currently
agree to the millimetre, which is the point: both sides use
`Geod.inv` with `east = d sin(azimuth)`, `north = d cos(azimuth)` from the
same origin.

## The patch

`s1_navigation.patch` is three changes to two files. Apply with:

```bash
cd ~/loop_ws/src/Challenge_Week/ros2_ws/src/s1_navigation
patch -p1 < ~/loop_ws/src/loop-phase2/integration/s1_navigation.patch
```

**1. `world_min` and `world_max` become parameters** in `astar_planner`,
defaulting to the current -10 and +10. As constants they make the planner
unusable on the terrain world, where targets are hundreds of metres out: it
publishes an empty path and the rover simply sits there.

```bash
ros2 run s1_navigation astar_planner --ros-args -p world_min:=-1000.0 -p world_max:=1000.0
```

**2. A new target list is accepted** by `astar_planner` and
`gps_waypoint_converter`. Both currently keep the first list for the life of
the launch, which is right for a node that generates its own goals once, and
wrong as soon as an operator can re-task the rover. The patch compares the
incoming list with the current one, so a repeated identical message (which
both publishers send for late subscribers) still does not restart anything,
but a different list replaces it.

**3. The converter takes any number of pairs**, not exactly three. The GUI can
send one target or five.

Without the patch everything here still works except re-tasking after the
first list, and terrain-world planning.

## What is still missing on the navigation side

- **A costmap for the terrain world.** `terrain.launch.py` runs no
  `lidar_mapper` and no `astar_planner`, so there is nothing to plan with out
  there. The slope layer already published on `/elevation_map` as
  `traversability_cost` is most of a global costmap: reprojected into an
  `OccupancyGrid` on `/costmap` it would feed the existing planner directly.
- **A stop interface.** Holding or aborting a mission currently means
  publishing an empty `/planned_path` so `path_controller` stops following,
  plus zero `cmd_vel` at 20 Hz so nothing creeps. It works, but it is an
  override, not a handshake. A latched `/mission/enabled` boolean that the
  controller checks, or a service on the planner, would make it a real
  interface and let the GUI stop reaching into topics that are not its own.
- **Arrival tolerance.** The planner advances at a fixed 0.3 m
  (`GOAL_TOLERANCE`). The GUI carries a per-target tolerance because the rules
  do, and reports arrival against it. If those two ever disagree, the planner
  wins on the ground and the GUI is only reporting.

### A wrinkle worth knowing

Re-sending the *same* target list after a mission has finished has to restart
it. The first version of this patch compared lists only, so an operator who
pressed start again on the same three posts got a planner that quietly did
nothing, because as far as it was concerned it had already reached them. The
comparison now also asks whether the planner had run off the end of its list,
and takes the list again if it had. Found by doing exactly that twice in a
row: the console reported the target unreachable after fifteen seconds, which
is the right behaviour for the wrong reason.
