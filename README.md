# ai_master_checkpoint

[English](README.md) | [한국어](README_kr.md)

A ROS 2 package that drives a TurtleBot4 to a target position in simulation using `/odom` and `/scan`,
saves an image from `/camera/image_raw` on arrival, and automatically generates a result report.

## Run

```bash
cd ~/ros2_ws
colcon build --packages-select ai_master_checkpoint
source install/setup.bash
ros2 launch ai_master_checkpoint checkpoint.launch.py
```

A single launch command starts every node, and the whole launch shuts down automatically when the mission
ends (when `mission_controller` exits). The simulator must already be running and the robot must be at its
start position.

## Layout

```
ai_master_checkpoint/
├── checkpoint_report.txt          # result report, generated on every run
├── checkpoint_result/
│   └── arrival_image.png          # camera image saved on arrival
├── ai_master_checkpoint/
│   ├── mission_controller.py      # navigation, arrival detection, image/report saving
│   ├── robot_monitor.py           # /odom and /scan diagnostics
│   └── navigation_algorithm.py    # provided FastObstacleNavigator (not modified)
├── launch/
│   └── checkpoint.launch.py
├── package.xml
├── setup.cfg
└── setup.py
```

## Nodes

### mission_controller

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/odom` | `nav_msgs/Odometry` |
| Subscribes | `/scan` | `sensor_msgs/LaserScan` |
| Subscribes | `/camera/image_raw` | `sensor_msgs/Image` |
| Publishes | `/cmd_vel` | `geometry_msgs/Twist` |

Behavior:

1. Reads the position (x, y) and yaw from `/odom`.
2. Filters out `inf`, `nan` and values outside `range_min`/`range_max` from `/scan`, then computes the
   minimum distance in each of five sectors:

   | Sector | Angle (positive = left) |
   |---|---|
   | front | -15° to 15° |
   | front_left | 15° to 60° |
   | front_right | -60° to -15° |
   | left | 60° to 120° |
   | right | -120° to -60° |

   A sector with no valid reading is passed as `inf`.
3. Passes the pose, the target and the sector distances to `FastObstacleNavigator.compute()` and publishes the
   returned linear and angular velocity on `/cmd_vel`.
4. Stops when the distance to the target is within `goal_tolerance`.
5. After stopping, saves a camera frame to `checkpoint_result/arrival_image.png`, prints an arrival message,
   writes `checkpoint_report.txt` and exits.

Parameters:

| Name | Default | Description |
|---|---|---|
| `target_x` | -5.62021446228027 | Target x |
| `target_y` | -3.63326478004456 | Target y |
| `goal_tolerance` | 0.3 | Arrival radius (m) |
| `control_rate` | 10.0 | Control loop rate (Hz) |
| `package_dir` | (auto-detected) | Source package directory where result files are written |

The source package directory is found by looking for `package.xml` in the current directory, then in
`ai_master_checkpoint/` or `src/ai_master_checkpoint/` under `~/ros2_ws`.

### robot_monitor

Subscribes to `/odom` and `/scan` and periodically logs position, yaw, velocity, the nearest obstacle distance
and direction, the number of valid ranges, and the receive rate (Hz). It warns when no data has arrived or
when data has not updated for `stale_timeout`.

| Name | Default | Description |
|---|---|---|
| `report_period` | 1.0 | Logging period (s) |
| `stale_timeout` | 2.0 | Time without updates before a warning (s) |

## Results

- `checkpoint_report.txt`: system information, ROS 2 topics and message types, topic frequencies, topic field
  descriptions, the RViz2 Fixed Frame (`odom`), TF frames, TF explanations and the mission result. It is
  overwritten on every run.
- `checkpoint_result/arrival_image.png`: the image saved from `/camera/image_raw` on arrival.

System information, the topic list, topic frequencies and the mission result are measured at run time. The field
descriptions and TF-related entries are fixed text.

## Notes

- Images are converted with numpy and OpenCV instead of `cv_bridge` (`bgr8`, `rgb8`, `bgra8`, `rgba8` and
  `mono8` are supported).
- `navigation_algorithm.py` is provided and was not modified. Its docstrings cause `test_pep257` in
  `colcon test` to fail.
