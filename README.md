# Robot SLAM

This repository contains the software stack for a DIY security robot powered by ROS 2 Humble. The robot features real-time SLAM mapping, LiDAR-based odometry without wheel encoders, IMU data fusion, autonomous navigation, and a browser-based control interface.

To handle the computational load efficiently, the architecture is split into two layers:
1. **Raspberry Pi 4**: Manages direct hardware interfaces (sensors, motors).
2. **Ubuntu 22 Machine**: Handles intensive algorithms like SLAM and Nav2.

---

## 1. Hardware Components

| Part | Model | Connection Details |
| :--- | :--- | :--- |
| **MCU** | STM32F103 (Blue Pill) | UART GPIO to Pi (115200 baud) |
| **Motor Controller** | L298N H-Bridge | STM32 PWM + direction pins |
| **Drive System** | 4× Mecanum wheels + DC motors | Connected to L298N (no encoders) |
| **SBC** | Raspberry Pi 4 Model B | Central hub |
| **LiDAR** | RPLidar A1M8 (360°, 10m) | USB Micro-B → Pi |
| **IMU** | MPU6050 GY-521 | I2C (SDA/SCL) → Pi |
| **Camera** | IMX219 (CSI) | Ribbon cable → Pi |
| **Power Supply**| 12V LiPo battery | Direct 12V to L298N; DC-DC step-down 5V to Pi |

### Wiring Overview

```text
[12V Battery] ─┬─> L298N (12V) ───> 4× DC Motors
               └─> 5V DC-DC ──────> Raspberry Pi 4

Raspberry Pi Connections:
- GPIO14/15 (ttyAMA0) ──> STM32 Serial1 (115200 baud)
- I2C (bus 1, 0x68) ────> MPU6050 GY-521
- USB ──────────────────> RPLidar A1M8 (/dev/rplidar)
- CSI ──────────────────> IMX219 Camera

Communication:
Pi (WiFi) <─── ROS 2 CycloneDDS ───> Ubuntu 22 Computer
```

---

## 2. System Architecture

```text
[ Raspberry Pi 4 (Hardware Node) ]
  ├── cam_stream.py   ──> MJPEG Stream (:8080/stream.mjpg)
  ├── mpu6050_driver  ──> /imu/data (50Hz yaw rate)
  ├── sllidar_node    ──> /scan (raw data)
  ├── laser_filter    ──> /scan_filtered (0.15m to 12m)
  └── uart_bridge.py  ──> /cmd_vel & /robot_mode to STM32
           |
      (CycloneDDS)
           |
           v
[ Ubuntu 22 Computer (Processing Node) ]
  ├── rf2o_laser_odometry ──> /odom_rf2o (Scan matching odometry)
  ├── robot_localization  ──> /odometry/filtered (EKF)
  ├── slam_toolbox        ──> /map & TF (map -> odom)
  ├── nav2_bringup        ──> /cmd_vel (Navigation commands)
  ├── pose_republisher    ──> /robot_pose (For dashboard)
  ├── draw_handler        ──> /nav/draw_path -> Nav2
  └── rosbridge_suite     ──> WebSocket (:9090)
           |
      (WebSocket)
           |
           v
[ Web Interface (Client Browser) ]
  └── Provides live camera, SLAM map, point/draw navigation, and joystick.
```

---

## 3. Directory Layout

```text
Robot/
├── stm32/
│   └── Wheel+UltraSonic.ino       # STM32 Motor & Ultrasonic firmware
│
├── raspberry_pi/
│   ├── cyclone_dds.xml            # DDS config: vmuser-virtual-machine.local
│   ├── setup.py                   # ROS 2 pkg: web_pose
│   ├── start_pi.sh                # Pi startup script
│   ├── launch/
│   │   └── robot_bringup.launch.py # Launches LiDAR, IMU, and motor bridge
│   ├── lidar/
│   │   └── laser_filter.yaml      # Distance filter: 0.15m - 12m
│   ├── imu/
│   │   └── mpu6050_driver.py      # I2C IMU driver (50Hz)
│   ├── motor/
│   │   └── uart_bridge.py         # Converts /cmd_vel to STM32 UART commands
│   ├── camera/
│   │   └── cam_stream.py          # MJPEG video server
│   └── web_dashboard/
│       └── index.html             # Web UI (roslibjs)
│
└── computer/
    ├── cyclone_dds.xml             # DDS config: robotanninh.local
    ├── setup.py                    # ROS 2 pkg: mac_brain
    ├── start_mac.sh                # Script: Start MAPPING
    ├── start_nav.sh                # Script: Start NAVIGATION
    ├── odometry_ekf/
    │   └── ekf.yaml                # EKF fusion (rf2o + IMU)
    ├── slam/
    │   ├── slam_params.yaml        # Mapping configuration
    │   └── localization_params.yaml# Localization configuration
    ├── navigation/
    │   ├── nav2_params.yaml        # Nav2 (RPP + Costmaps)
    │   └── mac_nav.launch.py       # Nav2 launch with a saved map
    ├── nodes/
    │   ├── pose_republisher.py     # Publishes /robot_pose
    │   └── draw_handler.py         # Processes drawn paths for Nav2
    └── launch/
        └── mac_brain.launch.py     # Main SLAM/Nav2 launch file
```

---

## 4. STM32 Firmware Overview (`stm32/`)

**Source:** `Wheel+UltraSonic.ino`

This firmware is responsible for controlling the 4 mecanum wheels via the L298N driver. It listens for textual commands from the Raspberry Pi over UART (115200 baud).

**Pin Assignments:**
*   `ENA=PA0`, `ENB=PA1`: PWM for motor pairs
*   `IN1=PA2`, `IN2=PA3`: Left motor direction control
*   `IN3=PA4`, `IN4=PA5`: Right motor direction control
*   `TRIG=PB8`, `ECHO=PB9`: Ultrasonic sensor

**UART Command Protocol:**

| Command | Action |
| :--- | :--- |
| `ON` / `UP` | Move forward |
| `BACK` | Move backward |
| `LEFT` | Turn left |
| `RIGHT` | Turn right |
| `UP_LEFT` | Diagonal forward-left |
| `UP_RIGHT` | Diagonal forward-right |
| `DOWN_LEFT` | Diagonal backward-left |
| `DOWN_RIGHT` | Diagonal backward-right |
| `AUTO` | Engage autonomous obstacle avoidance |
| `MANUAL` / `OFF`| Halt movement |

*Note: `AUTO` mode uses a non-blocking `millis()` loop. If an object is detected within 40cm, the robot will reverse and turn right automatically.*

---

## 5. Raspberry Pi Configuration (`raspberry_pi/`)

### Dependencies

```bash
# Install ROS 2 Humble dependencies
sudo apt install ros-humble-desktop ros-humble-sllidar-ros \
  ros-humble-laser-filters ros-humble-rosbridge-suite \
  python3-smbus

# Install python dependencies
pip3 install pyserial

# Enable UART on Pi
sudo raspi-config  # Go to Interface -> Serial -> Disable login shell, Enable UART
```

### Build ROS 2 Package

```bash
mkdir -p ~/ros2_ws/src
cp -r raspberry_pi ~/ros2_ws/src/web_pose
cd ~/ros2_ws
colcon build
```

### Network Config

Copy the DDS profile:
```bash
cp raspberry_pi/cyclone_dds.xml /home/pi/cyclone_dds.xml
```
*(Make sure to update the peer address if the computer's hostname is different from `vmuser-virtual-machine.local`)*

### Running the Pi Node

```bash
chmod +x ~/ros2_ws/start_pi.sh
~/ros2_ws/start_pi.sh
```
This single script initializes the LiDAR, filters, IMU, UART bridge, web server, and camera stream.

**Data Flow (Pi):**
*   **Publishes:** `/scan_filtered` (LaserScan, 10Hz), `/imu/data` (Imu, 50Hz)
*   **Subscribes:** `/cmd_vel` (Twist), `/robot_mode` (String)

---

## 6. Ubuntu Computer Configuration (`computer/`)

### Dependencies

```bash
# Ensure Ubuntu 22.04 with ROS 2 Humble
sudo apt install ros-humble-desktop ros-humble-slam-toolbox \
  ros-humble-nav2-bringup ros-humble-rosbridge-suite \
  ros-humble-robot-localization ros-humble-rf2o-laser-odometry \
  ros-humble-rmw-cyclonedds-cpp
```

### Build & Configure

```bash
mkdir -p ~/ros2_ws/src ~/ros2_ws/config ~/ros2_ws/maps
cp -r computer ~/ros2_ws/src/mac_brain

# Setup configuration files
cp computer/odometry_ekf/ekf.yaml           ~/ros2_ws/config/
cp computer/slam/slam_params.yaml           ~/ros2_ws/config/
cp computer/slam/localization_params.yaml   ~/ros2_ws/config/
cp computer/navigation/nav2_params.yaml     ~/ros2_ws/config/
cp computer/cyclone_dds.xml                 ~/cyclone_dds.xml

cd ~/ros2_ws
colcon build
```
*(Update `~/cyclone_dds.xml` if your Pi's hostname is not `robotanninh.local`)*

---

## 7. Operating Instructions

### 1. Boot up the Raspberry Pi
```bash
~/ros2_ws/start_pi.sh
```

### 2. Launch Mapping Mode (Ubuntu Computer)
```bash
chmod +x ~/ros2_ws/start_mac.sh
~/ros2_ws/start_mac.sh
```

### 3. Access the Web Dashboard
Navigate to `http://<pi-ip-address>:8000` in your web browser.

Control the robot manually to scan the environment. Once mapped, click **💾 Serialize map** and provide a name (e.g., `floor1`).

### 4. Switch to Navigation Mode
```bash
chmod +x ~/ros2_ws/start_nav.sh
~/ros2_ws/start_nav.sh floor1
```

### 5. Using the Dashboard for Navigation

| Interface Control | Function |
| :--- | :--- |
| **📍 PIN** | Click to set a single `NavigateToPose` destination |
| **✏️ PEN** | Draw a multi-point path (`NavigateThroughPoses`) |
| **🗑 ERASE & STOP** | Abort current task and stop |
| **Zoom** | Scroll wheel or ＋ / － |
| **⊡ Fit** | Center and scale map |
| **Spacebar** | Emergency halt |
| **WASD / Arrows** | Manual teleoperation |

---

## 8. Network Setup Reference

Ensure both devices are connected to the same WiFi network.

| Configuration | Recommended Value |
| :--- | :--- |
| `ROS_DOMAIN_ID` | `42` |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| Pi Hostname | `robotanninh.local` |
| Computer Hostname | `vmuser-virtual-machine.local` |
| Rosbridge Socket | `computer:9090` |
| Camera URL | `pi:8080/stream.mjpg` |
| Dashboard URL | `pi:8000` |

---

## 9. Engineering Notes

*   **Odometry without Encoders:** Mecanum wheels are prone to slipping, so physical encoders are omitted. Instead, `rf2o_laser_odometry` is used to estimate movement by comparing sequential LiDAR scans.
*   **Targeted Sensor Fusion:** The system uses `robot_localization` (EKF) to combine linear velocity from the LiDAR (X-axis) with the yaw rate from the MPU6050. This provides sufficient 2D tracking for a flat indoor environment without needing full 6-DOF calculations.
*   **Distributed Computing:** To prevent overheating and lag on the Pi, heavy lifting (SLAM Toolbox, Nav2) is offloaded to an Ubuntu PC over WiFi via CycloneDDS.
*   **UART Deadband Synchronization:** The Nav2 `deadband_velocity` configuration (`[0.05, 0.0, 0.15]`) perfectly mirrors the `LIN_THRESH` and `ANG_THRESH` defined in the Pi's UART bridge. This guarantees Nav2 won't issue micro-commands that the STM32 would ignore.
*   **Startup Timing:** Nav2 initialization is intentionally delayed by 10 seconds in `mac_brain.launch.py`. This gives SLAM Toolbox and the EKF enough time to publish the first `/map` and establish the `odom -> base_link` transform tree before Nav2 requests it.
