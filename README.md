# px4_ros2_collision_prevention

A **ROS 2 Humble** node that reads a RealSense D435/D435i depth stream and
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

## Launch File
```bash
ros2 launch px4_ros2_collision_prevention obstacle_avoidance.launch.py
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
│   └── realsense_obstacle_node.py    ← Main ROS2 node
├── launch/
│   └── obstacle_avoidance.launch.py  ← Launch file
├── config/
│   └── params.yaml                   ← Default parameters
├── test/
│   └── test_depth_to_bins.py         ← Unit tests (no hardware needed)
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```
