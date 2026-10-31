# Husky Autoware packages

These four packages are a starting point for the observed A200, VLP-16 and
Fixposition setup. The sensing launch starts both hardware drivers when
`launch_driver:=true`. The vehicle interface is deliberately empty; keep
`launch_vehicle_interface:=false`.

## Frame convention

The supplied sensor measurements use the midpoint of the front and rear wheel
centers in x, the vehicle centerline in y, and wheel-axle height in z. The
installed Clearpath A200 xacro places `base_link` at the bottom-plate center,
with wheel centers 0.03282 m above it. This sensor model keeps Clearpath's
`base_link` and adds a fixed `husky_wheel_center` child frame at that offset.
The measured sensor transforms are children of `husky_wheel_center`.

Autoware's current vehicle-info geometry assumes a ground-level base frame at
the rear axle midpoint. Supporting the user's centered, wheel-axle-height
`base_link` throughout Autoware is future work; it affects footprint, planning
and control geometry, not just the vehicle-info YAML. The vehicle-info YAML
now contains the measured dimensions and a provisional 1.0 m forward-driving
turn radius. The current `run_autoware.sh` loads `husky_vehicle` parameters
and starts the vehicle description without its control interface. Frame
integration is still needed before planning or control uses Husky geometry.

## Required measurements

1. `husky_vehicle_description/config/vehicle_info.param.yaml` records the
   supplied wheel and body measurements. Its equivalent steering angle is
   `atan(0.510 / 1.0)` radians. Differential drive can turn in place, so this
   value is a forward-driving planning choice. Align the frame convention and
   validate the Autoware footprint before using vehicle geometry.
2. `husky_sensor_kit_description/config/sensors_calibration.yaml` contains the
   supplied measured positions and confirmed zero rotations from
   `husky_wheel_center`.
   The recorded VLP-16 uses `velodyne`, and Fixposition GNSS1 uses `GNSS1`.
   CORRIMU uses `FP_VRTK`. The vehicle module publishes the Husky URDF's identity
   `FP_POI` to `FP_VRTK` transform, matching the confirmed coincident frames.
   The Husky launch disables only this TF in the Fixposition driver; its other
   TF outputs remain enabled.
3. Confirm that no other robot state publisher already owns these sensor TFs.
   Confirm map projector settings and GNSS antenna frame before using GNSS pose.

## Current data paths

- `/velodyne_points` feeds `distortion_corrector_node` directly, which publishes
  `/sensing/lidar/concatenated/pointcloud`. The ring outlier filter is bypassed;
  `/sensing/lidar/top/rectified/pointcloud_ex` is no longer published.
  The corrector accepts at most `lidar_input_rate` clouds per second (default
  5 Hz from `autoware_launch/config/sensing/lidar_input_rate.param.yaml`), so
  its output rate follows the accepted input rate when the sensor supplies at
  least 5 Hz. A slower sensor produces a slower output.
  The best-effort `/sensing/lidar/top/pointcloud_raw_ex` copy is available
  for monitoring and recording; it is not an input to distortion correction.
- `/fixposition/fpa/corrimu` (`fixposition_driver_msgs/FpaImu`) to
  `/sensing/imu/imu_data` (`sensor_msgs/Imu`), preserving the `FP_VRTK` frame.
- `/fixposition/gnss1` through a QoS bridge and `autoware_gnss_poser` to GNSS pose.
- `/husky/platform/odom/filtered` to Autoware twist input.

### LiDAR obstacle stop

The Husky LiDAR processing launch reads the distortion corrector's
`/sensing/lidar/concatenated/pointcloud`. The corrector retains the VLP-16
`channel` field and the `velodyne` sensor frame while correcting `x`, `y`, and
`z`. The obstacle node first keeps points within the configured forward
`half_angle_deg + 5°` sector, then projects `channel` (or `ring`) and the azimuth
computed from `x` and `y` into a 16 by 1800 scan grid, keeping the first valid
return in each cell. For adjacent low rings 0–7 and 1–8 at the same azimuth, it measures
the angle of the line between returns. When that angle is within 10° of the
0° sensor mounting angle, both cells are classified as ground. Missing pairs
and unmatched cells remain non-ground. The grid size, last low ring, mounting
angle, and tolerance are node parameters.
The defaults are in
`autoware_launch/config/sensing/lidar_obstacle_stop.param.yaml`. The LiDAR
processing launch loads that file and accepts `obstacle_stop_param_file` to
select another ROS parameter YAML.
`detection_fps` defaults to 10.0 in the node and is set to 2.0 in the Husky
parameter file. A steady-clock timer processes the newest corrected cloud
at most once per timer period; older unprocessed clouds are
replaced. If the input is slower, each new cloud is processed at the next tick.
The Husky command bridge treats obstacle status older than 1.5 seconds as a
stop condition, so a much lower detection rate also requires a longer bridge
`obstacle_timeout_s`.

It counts non-ground cells within 2 m of the LiDAR and within the configured
`half_angle_deg` sector (±45° in the Husky parameter file). At the configured
40 cells, it publishes `true` on
`/sensing/lidar/obstacle_stop`; it publishes `false` again when the count
falls below 40. `/sensing/lidar/obstacle_count` reports a count capped at 40.
The segmentation, including its 5° margin, also publishes
`/sensing/lidar/ground/pointcloud` and
`/sensing/lidar/nonground/pointcloud` as XYZ `PointCloud2` clouds. These clouds
contain the first valid return per ring and azimuth cell within the sector, and
retain the input timestamp and `velodyne` frame. The 2 m limit applies only to
the stop count, not to these output clouds.
The node packs and publishes each visualization cloud only while that topic
has subscribers; obstacle counting and the stop decision run on every selected scan.
`/sensing/lidar/obstacle_exe_time_ms` publishes processing runtime as a
`std_msgs/Float64` in **milliseconds** for each valid selected scan, from processing start
through publishing the stop decision.
The Husky vehicle interface publishes zero linear and angular velocity while
the stop is active or its LiDAR status is older than 1.5 seconds. Normal
autonomous commands resume when a fresh scan reports fewer than 20 points.

This local slope test needs adjacent ring returns and a correctly calibrated
sensor mounting angle. Body reflections, steep slopes, and sparse returns need
on-robot validation before relying on the detector. The stop applies to the
Autoware vehicle interface; direct manual or platform command publishers have
their own control path.

The self crop and ring outlier filter stages are bypassed, so points reflected
from the Husky body and isolated outliers may remain in the cloud. Motion correction needs the Husky platform's
`/husky/platform/odom/filtered` twist; without it the distortion corrector
forwards the cloud and reports `Twist queue is empty`. Check point cloud fields,
TF and timestamps before using localization. The GNSS poser currently infers
heading from movement when enabled.

## Run Husky Autoware

From the workspace root, build the packages once:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select fixposition_driver_ros2 husky_vehicle_description husky_sensor_kit_description husky_sensor_kit_launch autoware_launch
```

Run `./run_autoware.sh` from the workspace root. It launches
`autoware_launch/autoware.launch.xml` with the Husky vehicle TF, map, sensing
processing, NDT localization, and shared pointcloud container. Sensor drivers,
the vehicle control interface, planning, and perception are off. Simulated time
is enabled for bag replay, and automatic GNSS initialization is disabled because
the current sensing launch does not produce a GNSS pose. RViz opens the map-based
localization view. For live sensors, pass
`launch_sensing_driver:=true use_sim_time:=false`; use
`localization_gnss_enabled:=true` only after enabling a valid GNSS pose source.
Do not run `bring_up.sh` at the same time: it starts the same VLP-16 and
Fixposition drivers. The GNSS poser is off because it needs
`/map/map_projector_info` from the map loader for localization.

For the recorded sensing bags, run `./run_autoware.sh` and, in another terminal,
`./replay_husky_sensing_bag.sh /path/to/bag`. The replay script publishes
`/clock` and the four raw inputs, leaving out recorded `/tf_static` so the
Husky URDF is the only publisher of the sensor transforms.

The compatibility `sensing_only.launch.py` starts the vehicle description
separately from sensing; sensing itself does not publish static TF.
The default VLP-16 network settings are in
`husky_sensor_kit_launch/config/VLP16.param.yaml` and can be overridden with
`vlp16_config_file:=/path/to/VLP16.param.yaml`.

Check the LiDAR input and processed output with `ros2 topic hz` on
`/velodyne_points` and `/sensing/lidar/concatenated/pointcloud`. The
`/sensing/lidar/top/pointcloud_raw_ex` monitoring copy can be checked
separately. Also check
`/sensing/imu/imu_data` and `/sensing/gnss/fix`. Check TF with
`ros2 run tf2_ros tf2_echo base_link velodyne`.

Full vehicle integration remains pending.

## Test LiDAR localization with a recorded bag

The map directory must contain `pointcloud_map.pcd`, `lanelet2_map.osm`, and
`map_projector_info.yaml`. The pointcloud map must cover the place where the
bag was recorded. Start Autoware in one terminal:

```bash
./run_autoware.sh \
  launch_map:=true \
  launch_localization:=true \
  launch_pointcloud_container:=true \
  system_run_mode:=logging_simulation \
  rviz_config_name:=husky_localization.rviz
```

This runs `autoware.launch.xml` with the vehicle TF, map, sensing processing,
shared pointcloud container and NDT localization. It leaves the hardware
drivers and vehicle control interface off, enables simulated time, and opens
`husky_localization.rviz` with `map` as the fixed frame. Start replay in a
second terminal; the optional `--loop` gives more time to set the pose:

```bash
./replay_husky_sensing_bag.sh sensor_bag_20260925_155334 --loop
```

The localization RViz view shows the Lanelet2 vector map, NDT pose as a large
red arrow, EKF pose as a smaller white arrow, and the recent EKF pose history
as a sky-blue line. The green pointcloud is NDT's map-frame aligned output
`/localization/pose_estimator/points_aligned`. During initialization, NDT
publishes it for the first Monte Carlo candidate and each candidate that
improves the best nearest-voxel transformation likelihood score. It also
publishes aligned clouds for matching scans afterwards.
Candidate pose markers are also displayed during initialization. RViz may skip
some short-lived candidate clouds when NDT publishes them faster than it can
render; the display shows the most recently received match. NDT's input
`/localization/util/downsample/pointcloud` remains in `base_link` and needs a
valid map-to-base_link TF for direct viewing in this map-fixed RViz view.

Use RViz's **2D Pose Estimate** on the matching pointcloud map, dragging in
the direction the robot faced. This config sends `/initialpose`; the RViz
initial-pose adaptor converts it to an API initialization request, and the
pose initializer publishes its result on `/initialpose3d`. Check progress on
`/localization/initialization_state`: 1 is uninitialized, 2 is initializing,
and 3 is initialized. The launch terminal also prints state transitions. State
3 means the initialization request completed; confirm that NDT scores are good
and the transformed cloud overlaps the map before treating the pose as valid.
The terminal reports the runtime NDT convergence threshold and the final
initialization nearest-voxel transformation likelihood score. The current
threshold is 2.3, and the score must be greater than it to be marked reliable.
Tune this threshold in
`autoware_launch/config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml`
(`score_estimation.converged_param_type: 1` and
`converged_param_nearest_voxel_transformation_likelihood`). Tune NDT input
voxel leaf size in
`autoware_launch/config/localization/ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_filter.param.yaml`
(`voxel_size_x/y/z`, in meters), and the subsequent random sample count in
`random_downsample_filter.param.yaml` (`sample_num`, number of points rather
than a percentage). Edit the source files under `src/launcher/autoware_launch`,
then restart the launch; use `# {OVERRIDE}` on locally changed settings so
parameter synchronization preserves them.
The recorded bag's GNSS1 fixes have invalid status, so manually set the
initial pose. The green aligned cloud first appears when NDT evaluates the
initialization candidates. If the NDT input cloud is missing, inspect
`/localization/util/downsample/pointcloud` separately in its `base_link`
frame.

Check the data path and output in a sourced ROS terminal:

```bash
ros2 topic hz /sensing/lidar/concatenated/pointcloud
ros2 topic hz /localization/util/downsample/pointcloud
ros2 topic info /map/vector_map_marker
ros2 topic echo --once /localization/pose_estimator/points_aligned --field header
ros2 topic hz /localization/pose_estimator/pose_with_covariance
ros2 topic echo --once /localization/kinematic_state --field pose.pose
ros2 run tf2_ros tf2_echo map base_link
```

The first two topics should publish during bag replay. NDT pose and fused
kinematic state require a matching map and a successful initial pose. In RViz,
the transformed live cloud should overlap the pointcloud map as the bag plays.
The test does not exercise route planning or Husky control.
