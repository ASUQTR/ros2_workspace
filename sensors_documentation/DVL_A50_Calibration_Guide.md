# DVL-A50 Heading Calibration Guide
**ASUQTR AUV — ROS 2 Environment**

---

## Overview

The DVL-A50 uses an integrated Yost Labs 3-Space Nano AHRS (gyro + accelerometer + magnetometer) to produce the heading reported in the `wrp` dead reckoning packet. Three calibrations are required before the heading can be trusted:

| Step | What it corrects | Frequency |
|---|---|---|
| 1. Gyro calibration | Gyro bias drift at rest | Once, stored on DVL flash |
| 2. Mounting rotation offset | DVL X-axis vs sub bow misalignment | Once per mechanical change |
| 3. Magnetic declination | Magnetic north vs true north | Per competition venue |
| 4. Hard iron offset | Permanent magnetic field from sub frame | Per sub configuration change |

**Steps 1 and 2 are sent directly to the DVL and stored persistently on the device. Steps 3 and 4 are set in `params.yaml` and applied by the ROS node.**

---

## Prerequisites

- Sub is fully assembled and all electronics are **powered on** (12V bus, ESCs, buck converters)
- Jetson is running and you are SSH'd in
- The DVL node is **stopped** for steps 1 and 2 (it holds an exclusive serial port lock)
- The DVL node is **running** for steps 3 and 4

**Source your workspace on every new terminal (adjust path to your actual workspace):**
```bash
source ~/ros2_workspace/workspace/install/setup.bash
```

---

## Step 1 — Gyro Calibration (one-time, persistent)

The DVL's internal gyro has a small bias at startup. This command measures and stores the offset in the DVL's flash memory.

**Requirements:**
- Sub must be **completely stationary** — place it on a flat, stable surface
- Takes up to **15 seconds**
- Only needs to be redone if you observe heading drift while the sub is at rest

**Stop the DVL node if running:**
```bash
pkill -f dvl_node.py
```

**Send the gyro calibration command:**
```bash
python3 - << 'EOF'
import serial
import crcmod

crc = crcmod.predefined.mkPredefinedCrcFun('crc-8')
port = serial.Serial('/dev/ttyTHS0', 115200, timeout=20)

cmd = b'wcg'
checksum = f"{crc(cmd):02x}".encode()
packet = cmd + b'*' + checksum + b'\n'
print(f"Sending: {packet}")
port.write(packet)
port.flush()

# wcg can take up to 15 seconds — timeout is set to 20s
response = port.readline()
print(f"Response: {response.decode().strip()}")
port.close()
EOF
```

**Expected response:** `wra*XX` (ACK — the XX is the CRC checksum, it varies).
**If you get `wrn`:** the gyro calibration failed — ensure the sub is completely still and retry.

---

## Step 2 — Mounting Rotation Offset (one-time, persistent)

If the DVL is not mounted with its X-axis (marked on the housing) perfectly aligned with the sub's forward (bow) direction, set this offset so the DVL corrects its heading output internally before the `wrp` packet is sent.

**How to measure:**
1. Point the sub's bow at a known direction (e.g., along a pool wall)
2. Measure the angle between the sub's bow and the DVL's X-axis marking using a protractor
3. The offset is that angle in degrees (0–360, clockwise from bow)

**If the DVL is perfectly aligned with the bow:** skip this step (offset = 0, the factory default).

**Stop the DVL node if running:**
```bash
pkill -f dvl_node.py
```

**Send the mounting offset command:**
```bash
python3 - << 'EOF'
import serial
import crcmod

OFFSET_DEGREES = 0.0   # <-- replace with your measured mechanical offset

crc = crcmod.predefined.mkPredefinedCrcFun('crc-8')
port = serial.Serial('/dev/ttyTHS0', 115200, timeout=5)

# wcs field order: speed_of_sound, mounting_rotation_offset, acoustic_enabled,
#                  dark_mode_enabled, range_mode, periodic_cycling_enabled
# All fields left empty except mounting_rotation_offset
cmd = f"wcs,,{OFFSET_DEGREES},,,,".encode()
checksum = f"{crc(cmd):02x}".encode()
packet = cmd + b'*' + checksum + b'\n'
print(f"Sending: {packet}")
port.write(packet)
port.flush()

response = port.readline()
print(f"Response: {response.decode().strip()}")
port.close()
EOF
```

**Expected response:** `wra*XX` (ACK). This offset is now stored persistently on the DVL — it survives power cycles.

---

## Step 3 — Magnetic Declination (per venue)

Magnetic north ≠ true north. The difference is called declination and depends on your geographic location.

**1. Look up your venue's declination:**

Go to: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
- Enter the competition venue's coordinates
- Use the current date
- Read the **Declination** value (e.g., `-14.5°` West or `+5.2°` East)
- West declination → **negative** value in `params.yaml`

**2. Set it in `params.yaml`:**

Edit `workspace/packages/sub_hardware/config/params.yaml`:
```yaml
dvl_node:
  ros__parameters:
    heading_declination_deg: -14.5   # <-- your venue's value
```

Rebuild and relaunch for the change to take effect:
```bash
cd ~/ros2_workspace/workspace
colcon build --packages-select sub_hardware
source install/setup.bash
```

---

## Step 4 — Hard Iron Offset (per sub configuration)

The sub's own frame, batteries, and connectors create a permanent magnetic field that biases the DVL compass. This is measured as a single constant offset.

**Requirements:**
- **ALL electronics must be powered** — ESCs armed, thrusters at idle, all buck converters running. The magnetic field from operating currents must be present during calibration or the correction will be incomplete.
- Sub must be roughly **level** (horizontal)
- Do this **in the pool** or on a non-metallic surface away from large metal structures

### 4a — Find a reference bearing

You need at least one known absolute heading to compute the offset. Options:
- **Pool wall method:** Align the sub's bow parallel to a pool wall, then measure that wall's bearing with an external handheld compass
- **Two-point method:** Place two markers at known GPS coordinates and compute the bearing between them

### 4b — Launch the DVL node

```bash
ros2 launch sub_launch hardware.launch.yaml
```

### 4c — Watch the raw heading live

Open a second terminal:
```bash
source ~/ros2_workspace/workspace/install/setup.bash
ros2 topic echo /dvl/heading_raw
```

Or record a bag for offline analysis:
```bash
ros2 bag record -o dvl_calibration /dvl/heading_raw
```

### 4d — Rotate the sub

Rotate the sub **slowly** (one full revolution per ~30 seconds) through **at least two full 360° turns** in the pool. Keep it level. This lets you observe the raw heading traverse the full 0–360° range.

### 4e — Compute the offset

Point the sub's bow at your known reference bearing and hold it **completely still**. Collect 10 or more `/dvl/heading_raw` readings and average them.

```
hard_iron_offset_deg = reference_bearing - average(dvl/heading_raw)
```

**Example:**
- Pool wall bearing (measured with handheld compass) = **90.0°**
- DVL raw readings: 102.3, 101.8, 102.5, 102.1, 101.9 → average = **102.1°**
- `hard_iron_offset_deg = 90.0 − 102.1 = **−12.1°**`

**Sanity check:** Repeat the measurement pointing at 2–3 other known bearings. The computed offset should be consistent within ±3°. If it varies by more than 5° between headings, the distortion is sinusoidal (soft iron) and a more advanced calibration is needed.

### 4f — Set the offset in `params.yaml`

```yaml
dvl_node:
  ros__parameters:
    heading_hard_iron_offset_deg: -12.1   # <-- your computed value
```

Rebuild and relaunch:
```bash
cd ~/ros2_workspace/workspace
colcon build --packages-select sub_hardware
source install/setup.bash
ros2 launch sub_launch hardware.launch.yaml
```

---

## Step 5 — Verification

With the DVL node running and both corrections set in `params.yaml`:

**Terminal 1 — watch raw heading:**
```bash
ros2 topic echo /dvl/heading_raw
```

**Terminal 2 — watch corrected ENU yaw in degrees:**
```bash
python3 - << 'EOF'
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu

class YawPrinter(Node):
    def __init__(self):
        super().__init__('yaw_printer')
        self.create_subscription(Imu, '/dvl/orientation', self.cb, 10)

    def cb(self, msg):
        q = msg.orientation
        # Standard ZYX quaternion-to-yaw extraction
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        print(f"Corrected ENU yaw: {math.degrees(yaw_rad):.1f} deg")

rclpy.init()
node = YawPrinter()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
EOF
```

**Pass criteria:**
- Corrected heading matches your reference bearing within **±3°**
- Heading is **stable** (< 1° variation) when the sub is stationary with thrusters at idle
- Heading **recovers correctly** after rotating the sub away and returning to the reference bearing

---

## Quick Reference — Topics

| Topic | Type | Content |
|---|---|---|
| `/dvl/heading_raw` | `std_msgs/Float64` | Raw degrees from AHRS, magnetic north, no corrections applied |
| `/dvl/orientation` | `sensor_msgs/Imu` | Corrected quaternion in ENU frame, ready for EKF |
| `/dvl/velocities` | `geometry_msgs/TwistWithCovarianceStamped` | Body-frame velocities in FLU |
| `/dvl/altitude` | `geometry_msgs/PoseStamped` | Distance to pool bottom |

---

## Quick Reference — params.yaml

```yaml
dvl_node:
  ros__parameters:
    serial_port_name: "/dev/ttyTHS0"
    heading_declination_deg: 0.0        # West = negative. Look up per venue at ngdc.noaa.gov.
    heading_hard_iron_offset_deg: 0.0   # Computed from Step 4 rotation test.
```
