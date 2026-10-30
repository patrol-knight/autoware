# robot_state

Publishes `/robot_status` as `std_msgs/msg/String`. The publisher uses transient-local QoS so
late subscribers receive the current status. Start it with `ros2 launch robot_state
robot_state.launch.py`; the Husky `run_autoware.sh` script also enables it.
The Husky planning RViz configuration shows the topic in a compact color-coded
`RobotStatusPanel`.
The current status is published at 5 Hz and immediately when it changes. `reach` is a
single arrival event and is not repeated at 5 Hz.

| Value | Meaning |
| --- | --- |
| `notready` | Localization has not reported `INITIALIZED`, or LiDAR, IMU, or wheel odometry messages are missing or stale. |
| `ready` | Localization is initialized and the robot is idle, including after arrival. |
| `failure` | Planning could not find a valid route for the RViz goal; the adaptor retries from the current localization pose once per second. An aborted route also reports failure. |
| `navigate` | An assigned route is active and autonomous control is enabled. |
| `reach` | The route entered `ARRIVED` and fresh vehicle velocity confirms a stop; published once per arrival, then replaced by `ready` after a short delivery interval. |
| `manual` | Joystick/RC commands are recent while the Husky interface reports local or remote control, unless planning is in `failure`. |

`ABORTED` is an Autoware route-state enum for an aborted route. The status node maps it
to `failure`, but it does not by itself start the same-goal retry; that retry starts
when the route service reports a planner failure for a submitted goal.

The node reads `/localization/initialization_state`, `/planning/route_state`, and
`/system/operation_mode/state`. The RViz goal adaptor additionally publishes
`/planning/route_request_status` with `pending`, `accepted`, or `rejected` for its
route service requests. While retrying a rejected goal, `failure` remains active even
if a route briefly appears or the joystick is used. The 1 Hz retry continues during
manual control. When a `notready` robot receives joystick or RC input, it displays
`manual` and returns to `notready` when that input stops. Manual takeover from `ready`,
`navigate`, or `reach` requests `/planning/clear_route` when a route exists; the adaptor
retries a failed clear request, and the status returns to `ready` when manual input stops.
The Husky trajectory follower drops its cached trajectory and publishes a stop command
when the route is no longer active.

On a successful retry, the adaptor calls `/planning/clear_route` to delete the recovered route, restores
`enable_auto_with_route` to true, and reports `ready`. The recovered goal therefore does
not start autonomous navigation; send another goal to navigate. The adaptor requires
fresh `/localization/kinematic_state` odometry before each retry. A reached route takes
precedence over the Husky interface's automatic
switch to local mode on arrival. The `reach` event gets a 0.2-second delivery interval
so subscribers with a one-message queue can process it before `ready`; a new goal
received after `ARRIVED` clears the completed route and is submitted immediately, without
requiring a manual mode switch. `manual_command_timeout_s` (default 1 second)
controls how long a joystick or RC command counts as active. Planner unready and invalid-state
service errors do not count as an invalid goal.

The stop confirmation reads `/vehicle/status/velocity_status` after `ARRIVED` and
requires a fresh `autoware_vehicle_msgs/msg/VelocityReport`. Longitudinal and lateral
speeds must each be at most `stop_linear_tolerance_mps` (default 0.01 m/s) in absolute
value, and heading rate at most `stop_angular_tolerance_radps` (default 0.01 rad/s).
`velocity_timeout_s` defaults to 1 second. Missing, stale, or non-finite velocity
does not produce `reach`.

Sensor liveness uses the receipt time of `/sensing/lidar/concatenated/pointcloud`
(`sensor_msgs/msg/PointCloud2`), `/sensing/imu/imu_data` (`sensor_msgs/msg/Imu`),
and `/husky/platform/odom/filtered` (`nav_msgs/msg/Odometry`). All three must
arrive within `sensor_timeout_s` (default 2 seconds); the three topic names are
configurable with `lidar_topic`, `imu_topic`, and `wheel_odometry_topic`.
