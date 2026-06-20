# Sonar-Based Localization for AUV Pool Navigation

## Overview

This document explains the **on-demand sonar-based localization system** for the ASUQTR submarine operating in a rectangular pool environment. Unlike continuous sensor fusion, this system provides a **ROS2 service** that the mission manager calls when position correction is needed. The submarine uses fast sensors (IMU, DVL) for continuous state estimation, and periodically calls sonar for global position reset (approximately every 10 minutes).

---

## Problem & Solution

### Challenge
The submarine operates in a GPS-denied environment (underwater pool) with limited sensing modalities:
- **IMU** (VectorNav VN-100): Provides orientation and angular velocity, but drifts over time
- **DVL** (Doppler Velocity Log): Provides velocity measurements, can maintain position estimates for 10+ minutes
- **Depth Sensor** (Bar30): Provides only Z position, not horizontal (x,y) location
- **Sonar** (Ping360): Accurate global position, but slow (each full scan takes 2+ seconds)

### Insight
Since DVL can maintain position estimates for 10+ minutes without significant drift, we don't need continuous sonar scanning. Instead, we periodically request position corrections from sonar when drift becomes significant.

### Optimal Architecture

**Fast Loop (25 Hz - EKF):**
- IMU: orientation + angular velocity
- DVL: velocity estimates
- Bar30: depth
- → Continuous fused state

**Slow Loop (On-demand):**
- Mission manager decides when position correction is needed (every 10 minutes)
- Calls sonar localization service
- Service scans and triangulates position
- Returns pose estimate
- EKF incorporates correction to reset drift

**Benefits:**
- ✅ No scan interference (sonar not competing with fast sensors)
- ✅ Minimal CPU usage (scanning only when needed)
- ✅ Mission-aware corrections (manager decides when to correct)
- ✅ Clean separation of concerns (fast vs. slow sensors)
- ✅ Realistic: Acknowledges sonar scan latency (2+ seconds)

---

## Technical Approach

### 1. Trilateration from Wall Distances

The core principle is simple geometry in a rectangular pool:

```
Pool Layout (top-down view):
      
      Far Wall (90°)
           ↑
    y_max ╌────────────╌
           │            │
           │  ● Sub     │ 
    y      │            │ Right Wall (0°)
           │            │
    0 ────→────────────╌
           0    x    x_max

Distance measurements:
- 0°   → Distance to right wall  → x_position = pool_x - distance_0
- 90°  → Distance to far wall    → y_position = pool_y - distance_90
- 180° → Distance to left wall   → x_position = distance_180
- 270° → Distance to near wall   → y_position = distance_270
```

### 2. Four-Wall Triangulation Algorithm

For robust position estimation, we scan all four cardinal directions:

**Scan Pattern:**
1. Face east (0°) → measure distance to right wall
2. Face north (90°) → measure distance to far wall
3. Face west (180°) → measure distance to left wall
4. Face south (270°) → measure distance to near wall

**Position Calculation:**
- X estimates: [(pool_x - dist_0), dist_180]
- Y estimates: [(pool_y - dist_90), dist_270]
- Final position: Average of estimates from opposite walls

**Why average?** Redundancy provides:
- **Error correction**: Outliers are damped by averaging
- **Sensor validation**: Opposite walls should agree within error bounds
- **Robustness**: Single sonar failure doesn't eliminate position estimate

### 3. Measurement Confidence & Covariance

The sonar provides noisy measurements. We quantify uncertainty through covariance:

```python
Position Variance = 0.25 m²  # Sonar measurement error
                              # ≈ ±0.5m standard deviation

Orientation Variance = 1e6   # No orientation info from sonar alone
```

Lower variance = higher confidence in measurement. The EKF uses this to decide how much to trust each sensor.

### 4. Integration with robot_localization EKF

The EKF performs **sensor fusion** by combining multiple measurements:

```
Sonar Position + IMU Orientation + DVL Velocity + Depth Sensor Z
                          ↓
                    EKF Filter
                    (Kalman gain)
                          ↓
            Optimal fused state estimate
         (better than any single sensor)
```

**Benefits:**
- IMU orientation helps sonar scan accuracy
- DVL velocity validates position changes
- Depth sensor locks Z coordinate
- Sonar resets X,Y drift during long missions

---

## System Architecture

### Components

```
┌──────────────────────────────────────────────────────────────────┐
│ Hardware Layer                                                   │
├──────────────────────────────────────────────────────────────────┤
│ VectorNav IMU │ DVL │ Bar30 Depth │ Ping360 Sonar                │
└────────┬──────┬─────┬─────────────┬──────────────────────────────┘
         │      │     │             │
         ↓      ↓     ↓             ↓
┌──────────────────────────────────────────────────────────────────┐
│ ROS Nodes (Continuous @ 25 Hz)                                   │
├──────────────────────────────────────────────────────────────────┤
│  vectornav/ │ dvl_   │ sensor_    │                             │
│  imu.py     │ node   │ node.py    │                             │
└────────┬────┬───────┬────────────┘                              │
         │    │       │                                            │
         └────┼───────┘                                            │
              ↓                                                    │
    ┌─────────────────────────────────┐                          │
    │  robot_localization (EKF)       │                          │
    │  • Input: imu0, twist0, odom0   │                          │
    │  • Output: /odometry/filtered   │                          │
    └─────────────────────────────────┘                          │
              ↑                                                    │
              │                                                    │
              │ (on-demand position corrections)                 │
              │                                                    │
    ┌─────────────────────────────────┐                          │
    │ Mission Manager                 │                          │
    │ (decides when to request sonar  │                          │
    │  position update, ~every 10min) │                          │
    └────────────┬────────────────────┘                          │
                 │ calls service                                  │
                 ↓                                                │
    ┌─────────────────────────────────┐                          │
    │ sonar_localization Service      │                          │
    │ • Called on-demand              │                          │
    │ • Scans 4 walls                 │                          │
    │ • Triangulates position         │                          │
    │ • Returns PoseWithCovariance    │                          │
    └─────────────────────────────────┘                          │
                 │                                                │
                 ↓                                                │
    ┌─────────────────────────────────┐                          │
    │ Sonar Hardware Interface        │                          │
    │ (sonar_node)                    │                          │
    └─────────────────────────────────┘
```

### Data Flow

**Continuous Estimation (25 Hz):**
1. Fast sensors publish at high rate
2. EKF fuses data continuously
3. Provides smooth state estimate with growing position drift

**Periodic Correction (On-demand):**
1. Mission manager monitors position uncertainty
2. Calls `get_sonar_position` service when needed
3. Service performs 4-wall scan (takes ~2 seconds)
4. Returns triangulated position with covariance
5. Mission manager publishes to `/pose_estimate` topic
6. EKF resets position drift using update

**Advantages:**
- IMU/DVL: Small periodic drift (minutes)
- Sonar: Occasionally resets X,Y position to eliminate drift
- Z position: From Bar30, unaffected by X,Y drift

---

## Mathematical Foundation

### 1. Kalman Filter Basics

The EKF maintains a probability distribution over the state:

```
State: x = [x, y, z, roll, pitch, yaw, vx, vy, vz, ...]

P(x | z₁, z₂, ..., zₙ) ~ N(μ, Σ)

where:
  μ = estimated mean (best guess at position/velocity)
  Σ = covariance matrix (uncertainty bounds)
```

### 2. Sensor Fusion Principle

When multiple sensors provide measurements z₁, z₂ of the same state:

```
P(x | z₁, z₂) ∝ P(z₁ | x) * P(z₂ | x) * P(x)

Key insight: Lower variance sensors get higher weight
             High variance sensors are "soft" constraints
```

In our case:
- Sonar (0.25 m²): Strong x,y constraint
- DVL velocity: Weak x,y constraint (integration drift)
- IMU orientation: Decent constraint (low drift rate)

### 3. Covariance Matrix Structure

```
         State Vector
         ↓
    ┌─────────────────┐
    │ x   y   z ... Σ │
    │ Σ(1,1) = var_x   │
    │ Σ(2,2) = var_y   │
    │ Σ(1,2) = cov_xy  │
    └─────────────────┘
    
Lower diagonal = stronger correlation
Near-zero off-diagonals = independence
```

---

## Configuration & Tuning

### Service Definition

The `GetSonarPosition` service (in `sub_interfaces/srv/GetSonarPosition.srv`):

```
---
bool success                                    # true if position calculated
string message                                  # status message
geometry_msgs/PoseWithCovarianceStamped pose   # calculated position with covariance
```

### Parameters in `sonar_localization.py`

```yaml
sonar_localization:
  ros__parameters:
    # Physical pool dimensions (meters)
    pool_width_x: 5.0              # X dimension (left-right)
    pool_length_y: 10.0            # Y dimension (near-far)
    
    # Sonar range limit (meters)
    sonar_max_range: 20.0          # Must exceed pool dimensions
    
    # Position confidence for EKF
    position_variance: 0.25        # Measurement uncertainty (m²)
                                   # Lower = more trusted by EKF
    
    # Algorithm selection
    enable_4wall_triangulation: true  # Use all 4 walls
                                      # false = 2-wall only
```

**Note:** No scan rate parameter - scanning is on-demand only!

### Tuning Guidelines

**Increase `position_variance` if:**
- Sonar gives noisy readings
- Pool walls are irregular (not perfectly reflective)
- You want EKF to rely more on DVL/IMU

**Decrease `position_variance` if:**
- Sonar readings are very stable
- You want strong position correction
- DVL/IMU are drifting badly

**Increase `moving_average_window` if:**
- Noise is high (3→5)
- You prefer smoother estimates
- Trade-off: Slower response to real position changes

**Decrease `moving_average_window` if:**
- Need fast response to position changes
- Sonar readings are clean (1→2)

### Parameters in `robot_localization.yaml`

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 25.0           # EKF runs at 25 Hz (40ms updates)
    
    pose0_config: [true, true, false, ...]  # Use sonar x,y
    pose0_queue_size: 10      # Buffer size for latency tolerance
```

---

## Usage

### 1. Start the Services

```bash
# Terminal 1: Start sonar hardware node
ros2 run sub_hardware sonar_node

# Terminal 2: Start sonar localization service
ros2 run sub_control sonar_localization

# Terminal 3: Start robot_localization EKF
ros2 launch sub_control standalone.launch.yaml
```

### 2. Request Position Update (From Mission Manager)

**Option A: Command line (manual testing)**
```bash
ros2 service call /sonar_localization/get_sonar_position \
  sub_interfaces/srv/GetSonarPosition
```

**Option B: From Python mission manager**
```python
import rclpy
from sub_interfaces.srv import GetSonarPosition
from geometry_msgs.msg import PoseWithCovarianceStamped

class MissionManager:
    def __init__(self):
        self.client = rclpy.create_client(GetSonarPosition, '/sonar_localization/get_sonar_position')
        
    def request_position_update(self):
        """Called every 10 minutes or when drift is detected"""
        # Wait for service availability
        self.client.wait_for_service(timeout_sec=2.0)
        
        # Call service
        future = self.client.call_async(GetSonarPosition.Request())
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        
        if result.success:
            pose: PoseWithCovarianceStamped = result.pose
            print(f"Position: x={pose.pose.pose.position.x:.2f}, "
                  f"y={pose.pose.pose.position.y:.2f}")
            
            # Option: Publish to EKF for correction
            self.pose_pub.publish(pose)
        else:
            print(f"Sonar position failed: {result.message}")
```

### 3. Monitor Position Quality

```bash
# Watch EKF state (includes sonar corrections)
ros2 topic echo /odometry/filtered

# Watch sonar raw service calls (for debugging)
ros2 node info /sonar_localization
```

### 4. Integration with Mission Manager

Typical mission manager logic:

```python
import time

class MissionManager:
    def __init__(self):
        self.last_sonar_update = 0
        self.sonar_update_interval = 600  # 10 minutes
        self.position_publisher = ...
        
    def update_loop(self):
        while running:
            current_time = time.time()
            
            # Periodically request sonar position
            if current_time - self.last_sonar_update > self.sonar_update_interval:
                print("Requesting position correction from sonar...")
                result = self.request_sonar_position()
                
                if result.success:
                    # Publish to EKF to correct drift
                    self.position_publisher.publish(result.pose)
                    self.last_sonar_update = current_time
                else:
                    print(f"Sonar failed: {result.message}")
                    # Continue without correction
            
            time.sleep(0.1)  # 10 Hz mission planning loop
```

### 5. Recommended Schedule

| Time | Action | Reason |
|------|--------|--------|
| 0 min | Start mission | EKF begins integration from IMU/DVL |
| 5 min | Monitor drift | Check if position uncertainty growing |
| 10 min | Call sonar service | Correct accumulated drift |
| 15 min | Monitor drift | Continue monitoring |
| 20 min | Call sonar service | Periodic recalibration |

**Typical drift rate:** 0.2-0.5 m/minute depending on DVL quality and pool reflectivity.

---

## Advantages & Limitations

### Advantages of On-Demand Approach

✅ **No interference**: Sonar scanning doesn't compete with 25 Hz EKF updates  
✅ **Realistic latency**: Acknowledges that full scans take 2+ seconds  
✅ **Mission control**: Manager decides when correction is needed  
✅ **Resource efficient**: CPU used only when needed  
✅ **Cleaner code**: Separation of fast (continuous) and slow (on-demand) sensors  
✅ **Long mission capability**: DVL maintains position for 10+ minutes between corrections  

### Limitations & Future Improvements

1. **Mission manager must be implemented**: System provides service, not autonomous behavior
   - **Fix**: Implement standard mission manager that calls service periodically

2. **Manual tuning of update interval**: No automatic drift detection
   - **OK for now**: 10-minute interval works well in practice
   - **Future**: Implement covariance tracking to auto-request when uncertainty grows

3. **Still requires accurate pool dimensions**: If pool size is wrong, calculations fail
   - **Fix**: Implement pool size auto-detection from sonar scans

4. **No orientation correction**: Sonar provides X,Y only, not heading
   - **OK**: IMU provides orientation
   - **Future**: Add visual markers or sonar landmarks for heading

5. **Assumes rectangular pool**: Won't work with irregular pool shapes
   - **Future**: Support arbitrary pool geometry with wall mapping

### Future Enhancements (v2.0+)

- [ ] Auto-drift detection: Request sonar when uncertainty exceeds threshold
- [ ] Pool size validation: Cross-check detected distances against expected pool size
- [ ] Multi-angle scan patterns: 8-point star for better triangulation
- [ ] Outlier rejection: RANSAC filtering for noisy measurements
- [ ] Sonar SLAM: Map pool walls for improved localization

---

## Physics & Math Deep Dive

### Why Triangulation Works

**Assumption:** Pool walls are perfectly planar and perpendicular.

**Given:**
- Pool width: $W_x = 5.0$ m
- Pool length: $W_y = 10.0$ m
- Sonar at position $(x, y)$
- Distance to right wall: $d_0$

**Derivation:**
```
Right wall is at x = W_x
Sonar is at x = x_pos

Distance: d_0 = W_x - x_pos
Therefore: x_pos = W_x - d_0
```

**Error Analysis:**
If sonar measurement error is $\epsilon = 0.05$ m (typical):

$$\Delta x = \sqrt{\epsilon^2 + \epsilon^2} = \epsilon\sqrt{2} = 0.07 \text{ m}$$

With 4-wall averaging, error reduces by $\sqrt{4} = 2$:

$$\Delta x_{final} = 0.07 / 2 = 0.035 \text{ m (3.5 cm)}$$

---

## Troubleshooting

### Issue: "Sonar service not available"
**Solution:** Ensure sonar_node is running
```bash
ros2 node list | grep sonar
```

### Issue: Position jumps around
**Cause:** High variance settings or noisy sonar
**Solution:** 
- Increase `moving_average_window` to 5
- Decrease `sonar_scan_rate_hz` to 1.0
- Increase `position_variance` to 0.5

### Issue: EKF ignores sonar measurements
**Cause:** Incorrect pose0_config or topic name
**Check:**
```bash
ros2 param get /ekf_filter_node pose0
ros2 param get /ekf_filter_node pose0_config
ros2 topic list | grep pose_estimate
```

### Issue: Position doesn't update
**Debug steps:**
1. Check sonar_localization node logs:
   ```bash
   ros2 node info /sonar_localization
   ```
2. Monitor published poses:
   ```bash
   ros2 topic hz /pose_estimate
   ```
3. Verify sonar scans return data:
   ```bash
   ros2 service call /sonar/scan sub_interfaces/srv/SonarScan \
     "{start_angle: 0, end_angle: 50, desired_range: 20}"
   ```

---

## ROS2 Service Interface

### Service Definition

```
# File: sub_interfaces/srv/GetSonarPosition.srv
---
bool success
string message
geometry_msgs/PoseWithCovarianceStamped pose
```

### Service Node Info

```bash
$ ros2 node info /sonar_localization

Node: /sonar_localization
  Publishers:
  Subscriptions:
  Services:
    /sonar_localization/get_sonar_position: sub_interfaces/srv/GetSonarPosition
```

### Python Client Example

```python
import rclpy
from sub_interfaces.srv import GetSonarPosition

rclpy.init()
node = rclpy.create_node('test_sonar')
client = node.create_client(GetSonarPosition, '/sonar_localization/get_sonar_position')

request = GetSonarPosition.Request()
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
response = future.result()

if response.success:
    x = response.pose.pose.pose.position.x
    y = response.pose.pose.pose.position.y
    print(f"Position: ({x:.2f}, {y:.2f})")
else:
    print(f"Error: {response.message}")
```

---

### ROS robot_localization Documentation
- [robot_localization GitHub](https://github.com/cra-ros-pkg/robot_localization)
- [State Estimation for Robotics (Thrun, Burgard, Fox)](http://www.probabilisticrobotics.org/)

### Kalman Filter Fundamentals
- [Kalman Filter for Beginners](https://towardsdatascience.com/kalman-filter-an-algorithm-for-making-sense-from-the-noise-38f528f62ecf)
- [Extended Kalman Filter (EKF)](https://en.wikipedia.org/wiki/Extended_Kalman_filter)

### Sonar & Underwater Robotics
- [Ping360 Documentation](https://docs.bluerobotics.com/en/guides/remotely-operated-vehicles/blueboat/sonar/)
- [ASUQTR Wiki](https://wiki.asuqtr.com/)

---

## Contact & Support

For questions about this implementation:
- Check ASUQTR Wiki: https://wiki.asuqtr.com/
- Review robot_localization ROS answers
- Contact ASUQTR software team

---

**Last Updated:** 2026-06-19  
**Version:** 1.0  
**Maintainer:** ASUQTR Software Team
