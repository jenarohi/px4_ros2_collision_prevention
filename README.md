# px4_ros2_collision_prevention

A **ROS 2 Humble** package that bridges Intel RealSense D435/D435i depth data
to PX4's native **Collision Prevention** module via the uXRCE-DDS bridge.

Contains two nodes:

| Node | Input | Purpose |
|---|---|---|
| `realsense_obstacle_node` | Live D435 depth stream | Real-time obstacle avoidance |
| `depth_replay_node` | Recorded `.xlsx` depth frames | Replay & validate without hardware |

publishes `px4_msgs/ObstacleDistance` messages to PX4's native **Collision
Prevention** module via the uXRCE-DDS bridge.

When the drone flies in **Position mode** and an obstacle enters the `CP_DIST`
envelope, PX4 brakes automatically — **no custom controller, no MAVROS, no
pymavlink needed**.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  Companion Computer (Jetson / RPi / UP2 — Ubuntu 22.04 + ROS2 Humble)│
│                                                                       │
│  ┌─────────────────────────────────┐                                  │
│  │  RealSense D435/D435i (USB 3.0) │                                  │
│  └──────────────┬──────────────────┘                                  │
│                 │ pyrealsense2 / librealsense                         │
│  ┌──────────────▼──────────────────┐    /fmu/in/obstacle_distance     │
│  │  realsense_obstacle_node (ROS2) ├──────────────────────────┐       │
│  │  (this package)                 │   px4_msgs/ObstacleDistance│      │
│  └─────────────────────────────────┘       (15 Hz, uXRCE-DDS) │      │
│                                                                 │      │
│  ┌──────────────────────────────────────────────────────────────▼───┐ │
│  │  MicroXRCE-DDS Agent  (UDP / UART)                                │ │
│  └──────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
                           │ UART / UDP
            ┌──────────────▼──────────────┐
            │  Pixhawk (PX4 ≥ v1.14)       │
            │  uxrce_dds_client            │
            │  obstacle_distance uORB ──►  │
            │  Collision Prevention module │
            │  ► brakes in Position mode   │
            └─────────────────────────────┘
```

## Key Design Decisions

| Decision | Reason |
|---|---|
| uXRCE-DDS (not MAVROS, not pymavlink) | Native PX4 ≥v1.14 bridge; no extra translation layer |
| Publishes to `/fmu/in/obstacle_distance` | Direct uORB mapping; PX4 CP reads `obstacle_distance` topic |
| `MAV_FRAME_BODY_FRD` (frame=12) | Required by CP; rotates bins with vehicle heading |
| 72-bin array at 5°/bin | Full 360° coverage; MAVLink spec max |
| `UINT16_MAX` (65535) for unknown bins | PX4 CP treats these as "no obstacle — proceed" |
| Middle-row depth sampling | Gives horizontal obstacle plane; ignores ground/ceiling |
| Watchdog timer | Stops publishing if sensor lost; PX4 will failsafe after ~5 s |

---

## Prerequisites

### 1. System
- Ubuntu 22.04
- ROS 2 Humble
- PX4 firmware **≥ v1.14** on the flight controller

### 2. ROS 2 workspace dependencies
```bash
# px4_msgs — MUST match your PX4 firmware version branch
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git --branch release/1.14

# This package
git clone https://github.com/<your-handle>/px4_ros2_collision_prevention.git

cd ~/ros2_ws
colcon build --packages-select px4_msgs px4_ros2_collision_prevention
source install/setup.bash
```

### 3. Python dependencies
```bash
pip install pyrealsense2 numpy
```

### 4. Micro XRCE-DDS Agent (companion computer)
```bash
# Build from source (once)
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install

# Run (choose one):
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600   # UART
MicroXRCEAgent udp4 --port 8888                       # SITL / UDP
```

---

## PX4 Parameters (set via QGroundControl)

| Parameter | Value | Description |
|---|---|---|
| `CP_DIST` | `3.0` | **Enable CP.** Braking starts at 3 m from obstacle |
| `CP_DELAY` | `0.5` | Sensor+actuator delay estimate (s) |
| `CP_GUIDE_ANG` | `30` | Degrees CP can steer around an obstacle (optional) |
| `MPC_POS_MODE` | `0` | Required: acceleration-based position control |

> **Important:** Collision Prevention **only works in Position mode**. CP does
> NOT override offboard setpoints — it only acts on pilot stick inputs.

---

## Running

### SITL (Gazebo) test
```bash
# Terminal 1: PX4 SITL
cd PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: XRCE-DDS Agent (UDP for SITL)
MicroXRCEAgent udp4 --port 8888

# Terminal 3: Run node (simulated depth, no real camera needed)
ros2 run px4_ros2_collision_prevention realsense_obstacle_node \
  --ros-args -p simulate_depth:=true

# Verify: QGC MAVLink console
listener obstacle_distance
```

### Hardware (real RealSense + Pixhawk)
```bash
# XRCE agent over UART
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600 &

# Run node with real camera
ros2 run px4_ros2_collision_prevention realsense_obstacle_node
```

### Verifying data flow
```bash
# Echo the topic (should publish at ~15 Hz)
ros2 topic echo /fmu/in/obstacle_distance --no-arr

# Check publish rate
ros2 topic hz /fmu/in/obstacle_distance

# Inside QGC → MAVLink console:
listener obstacle_distance
```

---

## Depth Replay Node  (`depth_replay_node`)

Replays **pre-recorded** RealSense depth frames (`.xlsx` files) as
`ObstacleDistance` messages — no camera, no SITL required.
Useful for:
- Validating CP behaviour against a known real-world scenario
- Regression testing after firmware/node changes
- HIL / CI environments without physical hardware

### Python dependency
```bash
pip install openpyxl numpy
```

### Quick start
```bash
# Terminal 1: XRCE-DDS agent
MicroXRCEAgent udp4 --port 8888

# Terminal 2: Replay node
ros2 run px4_ros2_collision_prevention depth_replay_node \
  --ros-args -p data_dir:=/path/to/depthdata_frames

# With loop + custom rate
ros2 run px4_ros2_collision_prevention depth_replay_node \
  --ros-args \
    -p data_dir:=/path/to/depthdata_frames \
    -p loop:=true \
    -p publish_hz:=15.0

# Using launch file
ros2 launch px4_ros2_collision_prevention depth_replay.launch.py \
    data_dir:=/path/to/depthdata_frames loop:=true
```

### Expected depth frame format
- Files named `depthdata_filter_<N>.xlsx`  (N = frame index)
- Shape: **270 rows × 480 cols**  (RealSense half-resolution)
- Units: **millimetres** (float)
- Zero values = invalid / no return

### Replay Node Parameters

| Parameter | Default | Description |
|---|---|---|
| `data_dir` | `''` | **Required.** Path to folder with `depthdata_filter_*.xlsx` |
| `publish_hz` | `10.0` | Publish rate (Hz). PX4 CP needs ≥ 10 Hz |
| `loop` | `false` | Loop the dataset indefinitely |
| `fov_h_deg` | `87.0` | Camera horizontal FOV (degrees) |
| `num_bins` | `9` | Angular bins across the FOV |
| `min_depth_m` | `0.20` | Minimum valid depth (m) |
| `max_depth_m` | `8.00` | Maximum valid depth (m) |
| `depth_percentile` | `10` | Nth-percentile depth per bin (noise robust) |
| `min_pixels_per_bin` | `5` | Min valid pixels to report a bin distance |

### Verify replay is reaching PX4
```bash
# Should publish at publish_hz
ros2 topic hz /fmu/in/obstacle_distance

# See distances without array spam
ros2 topic echo /fmu/in/obstacle_distance --no-arr

# In QGC MAVLink console:
listener obstacle_distance
```

---

## Node Parameters

| Parameter | Default | Description |
|---|---|---|
| `publish_hz` | `15.0` | ObstacleDistance publish rate (Hz) |
| `min_depth_m` | `0.2` | D435 minimum reliable range (m) |
| `max_depth_m` | `10.0` | Maximum range — beyond this is "unknown" |
| `fov_h_deg` | `87.0` | D435/D435i horizontal FOV |
| `num_bins` | `72` | Angular bins (5°/bin, 360° coverage) |
| `use_filters` | `true` | Enable RealSense post-processing filters |
| `simulate_depth` | `false` | Inject synthetic obstacle at 2 m (for SITL) |
| `depth_width` | `640` | Depth stream width |
| `depth_height` | `480` | Depth stream height |
| `depth_fps` | `30` | Depth stream FPS |

---

## Troubleshooting

### PX4 not braking
1. Confirm `CP_DIST > 0` in QGC
2. Run `listener obstacle_distance` in QGC MAVLink console — if empty, data not reaching PX4
3. Check XRCE-DDS agent is running and connected
4. Verify `px4_msgs` branch matches firmware version

### Node crashes on start
- Check USB 3.0 port is used (USB 2.0 = bandwidth issues with D435)
- Try `rs-enumerate-devices` to confirm camera is visible

### "Unknown frame" warnings
- Ensure `frame` field is `12` (`MAV_FRAME_BODY_FRD`), not `0`

### Topic not visible
- Source your workspace: `source ~/ros2_ws/install/setup.bash`
- Confirm `obstacle_distance` is in `dds_topics.yaml` for your PX4 version

---

## Repository Structure
```
px4_ros2_collision_prevention/
├── px4_ros2_collision_prevention/
│   ├── __init__.py
│   ├── realsense_obstacle_node.py    ← Live D435 camera → ObstacleDistance
│   └── depth_replay_node.py          ← Recorded xlsx frames → ObstacleDistance
├── launch/
│   ├── obstacle_avoidance.launch.py  ← Launch live camera node
│   └── depth_replay.launch.py        ← Launch recorded data replay node
├── config/
│   ├── params.yaml                   ← Parameters for live camera node
│   └── replay_params.yaml            ← Parameters for depth replay node
├── test/
│   └── test_depth_to_bins.py         ← Unit tests (no hardware needed)
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```
