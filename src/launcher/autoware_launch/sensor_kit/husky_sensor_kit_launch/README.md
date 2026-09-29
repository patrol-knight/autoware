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
turn radius. The current `run_autoware.sh` uses `sample_vehicle` parameters
because it starts sensing only; frame integration is still needed before
planning or control uses Husky vehicle geometry.

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
   CORRIMU uses `FP_VRTK`; the recorded `FP_POI` to `FP_VRTK` transform is
   identity, matching the confirmed coincident frames.
3. Confirm that no other robot state publisher already owns these sensor TFs.
   Confirm map projector settings and GNSS antenna frame before using GNSS pose.

## Current data paths

- `/velodyne_points` to `/sensing/lidar/top/pointcloud_raw_ex`, then
  `crop_box_filter_self` → `distortion_corrector_node` →
  `ring_outlier_filter` → `/sensing/lidar/concatenated/pointcloud`.
- `/fixposition/fpa/corrimu` (`fixposition_driver_msgs/FpaImu`) to
  `/sensing/imu/imu_data` (`sensor_msgs/Imu`), preserving the `FP_VRTK` frame.
- `/fixposition/gnss1` through a QoS bridge and `autoware_gnss_poser` to GNSS pose.
- `/husky/platform/odom/filtered` to Autoware twist input.

The crop box uses the measured Husky dimensions in Clearpath's current
bottom-plate `base_link` frame. Motion correction needs the Husky platform's
`/husky/platform/odom/filtered` twist; without it the distortion corrector
forwards the cloud and reports `Twist queue is empty`. Check point cloud fields,
TF and timestamps before using localization. The GNSS poser currently infers
heading from movement when enabled.

## Run sensing only

From the workspace root, build the packages once:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select husky_vehicle_description husky_sensor_kit_description husky_sensor_kit_launch autoware_launch
```

Run `./run_autoware.sh` from the workspace root. It launches
`autoware_launch/autoware.launch.xml` with sensing and its hardware drivers
enabled. The vehicle, system, map, localization, perception, planning, control,
API and shared pointcloud container are disabled. RViz opens with a Husky
sensing view using `base_link` as its fixed frame and showing the processed
VLP-16 cloud. The toolbar includes Move Camera, 2D Pose Estimate and 2D Goal
Pose. In this sensing-only launch, the pose and goal tools publish messages but
no localization or planning node consumes them; use a map frame when enabling
those modules. Autoware's global
parameter loader still needs a valid vehicle-info file, so the script loads
`husky_vehicle` parameters while no vehicle nodes are running. The Husky
sensor model publishes the measured static TF because the vehicle module is
off. Keep the Husky platform driver running if you want odometry twist data.
Do not run `bring_up.sh` at the same time: it starts the same VLP-16 and
Fixposition drivers. The GNSS poser is off because it needs
`/map/map_projector_info` from the map loader for localization.

The earlier `run_husky_sensing.sh` remains a direct sensing-launch wrapper.
Use `./run_husky_sensing.sh launch_driver:=false` if the drivers are already
running. If another robot state publisher already owns the same sensor
transforms, that wrapper accepts `publish_sensor_tf:=false`.
The default VLP-16 network settings are in
`husky_sensor_kit_launch/config/VLP16.param.yaml` and can be overridden with
`vlp16_config_file:=/path/to/VLP16.param.yaml`.

Check each LiDAR stage with `ros2 topic hz` on
`/sensing/lidar/top/pointcloud_raw_ex`,
`/sensing/lidar/top/self_cropped/pointcloud_ex`,
`/sensing/lidar/top/rectified/pointcloud_ex` and
`/sensing/lidar/concatenated/pointcloud`. Also check
`/sensing/imu/imu_data` and `/sensing/gnss/fix`. Check TF with
`ros2 run tf2_ros tf2_echo base_link velodyne`.

Full vehicle integration remains pending.
