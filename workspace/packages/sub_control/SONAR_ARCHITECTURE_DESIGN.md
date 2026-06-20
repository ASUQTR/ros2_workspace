# Sonar Localization Architecture: Continuous vs. On-Demand

## Quick Summary

**Old Approach (Continuous):**
- Sonar scans every 0.5 seconds
- Problem: Each scan takes 2+ seconds → scans overlap and interfere
- Unrealistic data rates
- Wastes CPU and resources

**New Approach (On-Demand Service):**
- Mission manager calls sonar service when needed (~every 10 minutes)
- Clean separation: Fast sensors (IMU/DVL) run continuously, sonar is called occasionally
- Realistic: Acknowledges sonar scan latency
- Efficient: Only uses sonar when mission requires correction

---

## The Core Insight

**Key Realization:** DVL can maintain position estimates for 10+ minutes without significant drift.

This means we don't need continuous sonar scanning. Instead, we can:

1. Use fast, continuous sensors (IMU @ 25Hz, DVL @ variable Hz) to estimate position
2. Let position drift accumulate for 10 minutes
3. Periodically call sonar service to reset X,Y position drift
4. Loop back to step 1

This is much more realistic and efficient than trying to scan sonar every half-second.

---

## Detailed Comparison

### Continuous Scanning Approach (Original)

```
Time: 0s    0.5s   1.0s   1.5s   2.0s   2.5s   3.0s   3.5s   4.0s
      |──────┼──────┼──────┼──────┼──────┼──────┼──────┼──────┤

Scan 1: [━━━━━━━━━━━━] (2+ seconds)
Scan 2:                    [━━━━━━━━━━━━] (2+ seconds)
Scan 3:                                       [━━━━━━━━━━━━]

Problem: Scans overlap! Sonar can't scan while scan 1 is still processing.
```

**Issues:**
- ❌ Unrealistic timer (0.5s) vs. actual scan time (2+ seconds)
- ❌ Scans would queue and interfere
- ❌ Wastes CPU checking timer constantly
- ❌ Published data at unrealistic rates
- ❌ EKF gets noisy, overlapping measurements

### On-Demand Service Approach (New)

```
Time:  0s     5s    10s    15s    20s   ...   600s   610s   620s
       |──────┼──────┼──────┼──────┼──────┼────────┼──────┼──────┤

IMU/DVL runs: [continuous estimation, position drifts]
              ↓ (drift = 1-2 meters)
Sonar scan request: [━━━━] (2 second scan)
              ↓ (corrected position)
IMU/DVL runs: [continues, starts drifting again]
              
10 minutes later... [repeat]
```

**Benefits:**
- ✅ Realistic: Acknowledges actual scan time
- ✅ Efficient: Sonar only active when needed
- ✅ No interference: Fast loop unaffected by slow sonar
- ✅ Mission control: Manager decides when to correct
- ✅ Clean separation of concerns

---

## Technical Architecture Comparison

### Old: Continuous Fusion

```
Timer (0.5 Hz)
    ↓
[Scan sonar]
    ↓
[Publish /pose_estimate]
    ↓
EKF subscribes [continuous stream of updates]
```

**Problem:** Timer fires faster than sonar can actually scan.

### New: On-Demand Service

```
EKF [25 Hz loop]
  ├─ IMU: orientation + angular velocity
  ├─ DVL: velocity
  └─ Bar30: depth
     ↓ (position drifts slowly)

Mission Manager [1-10 Hz]
  └─ Every 10 minutes:
      └─ Call GetSonarPosition service
         ├─ Scans sonar (2 seconds)
         ├─ Triangulates position
         └─ Returns PoseWithCovariance
            ↓
         Publish to EKF
            ↓
         EKF resets X,Y position, reduces drift
```

**Benefits:**
- ✅ Realistic: Mission manager controls timing
- ✅ Explicit: Service call is blocking, no ambiguity
- ✅ Efficient: Only active when requested
- ✅ Robust: If sonar fails, mission continues

---

## Sensor Fusion Timeline

### 10-Minute Mission Cycle

```
t=0 min:
  Mission starts
  EKF begins with initial position estimate
  IMU: orientation ✓
  DVL: velocity ✓
  Bar30: depth ✓
  → Position uncertainty: low (just started)

t=5 min:
  IMU/DVL have been running continuously
  Position error accumulates: ~0.5-1.0m drift
  Other state (orientation, velocity, depth): still good
  → Position uncertainty: medium (drifting)

t=10 min: ← Sonar Correction Requested
  Mission manager calls: GetSonarPosition()
  
  [Service performs 2-second scan]
  - Scans 4 walls
  - Triangulates position
  - Returns: x=2.15m, y=5.23m (with covariance)
  
  [Mission manager publishes pose_estimate]
  
  [EKF receives correction]
  - Updates: "Actually, true position is 2.15m, 5.23m"
  - Resets X,Y state
  - Continues with IMU/DVL for Z, orientation, velocity
  
  → Position uncertainty: low (just corrected)

t=10-20 min:
  Cycle repeats
  Drift accumulates again
  At t=20 min, sonar corrects again
  
[Continues for entire mission]
```

### Covariance (Uncertainty) Over Time

```
Uncertainty (σ)
     ▲
     │     ╱╲        ╱╲        ╱╲
     │    ╱  ╲      ╱  ╲      ╱  ╲
     │   ╱    ╲____╱    ╲____╱    ╲
     │  ╱                           ╲____
     │ ╱
     │╱
     └─────────────────────────────────────► time
       │     │     │     │     │     │
       0    10    20    30    40    50  (min)
       ↑     ↑     ↑
     start  sonar sonar
            corr  corr
       
Each sonar correction drops uncertainty back to baseline.
Between corrections, uncertainty grows due to drift.
```

---

## Code Example: How It Works

### Service Call (Mission Manager)

```python
# Mission manager periodically calls sonar service
future = self.sonar_client.call_async(GetSonarPosition.Request())
rclpy.spin_until_future_complete(self, future)
response = future.result()

if response.success:
    # Got position from sonar
    x = response.pose.pose.pose.position.x
    y = response.pose.pose.pose.position.y
    
    # Publish to EKF to correct drift
    self.pose_publisher.publish(response.pose)
else:
    # Sonar failed, continue without correction
    print(f"Sonar failed: {response.message}")
```

### Service Implementation (sonar_localization.py)

```python
def sonar_position_callback(self, request, response):
    """Service callback - triggered by client request"""
    
    # Only when requested do we:
    x, y = self._scan_four_walls()  # Scans all 4 walls (2+ seconds)
    
    # Build response
    response.success = True
    response.pose = <triangulated position>
    return response  # Client receives it
```

---

## Why This Makes Sense for Your System

### Your Constraints

1. **Sonar scan time:** 2+ seconds per full scan
2. **EKF update rate:** 25 Hz
3. **DVL drift rate:** ~0.2-0.5 m/minute in pool
4. **Acceptable drift:** Pool is small (5m × 10m), can tolerate 1-2m error

### Solution: 10-Minute Correction Schedule

- **5-10 min drift:** 1-5 meters depending on DVL quality
- **Sonar scan time:** 2 seconds (one-time cost)
- **Benefit:** Full position reset, eliminates drift

**Math:**
```
Drift rate:        0.3 m/min
10-minute drift:   3 meters
DVL accuracy:      ±2m (typical in reflective pool)
Sonar accuracy:    ±0.25m (measured distance to walls)

→ Sonar correction is 8x more accurate!
→ Calling every 10 minutes maintains <1m error
```

---

## Design Patterns Used

### 1. Separation of Concerns
- **Fast loop (25 Hz):** State estimation (IMU, DVL, Bar30)
- **Slow loop (on-demand):** Position correction (sonar)

### 2. Service-Based Architecture
- Client asks for service when needed
- Server provides response
- No tight coupling or shared state

### 3. Mission-Aware Correction
- Mission manager decides when sonar is called
- Can adapt based on:
  - Mission criticality (safety vs. exploration)
  - Position uncertainty threshold
  - Battery/CPU constraints
  - Environmental conditions

### 4. Graceful Degradation
- If sonar fails: Mission continues with IMU/DVL
- If IMU fails: Sonar helps with position, DVL still provides velocity
- No single point of failure

---

## Summary Table

| Aspect | Continuous | On-Demand |
|--------|-----------|-----------|
| **Scan rate** | Every 0.5s (attempted) | When requested (~10 min) |
| **Actual scan latency** | 2+ seconds | 2 seconds |
| **Interference** | Yes (scans overlap) | No (single request) |
| **CPU usage** | High (constant) | Low (intermittent) |
| **Data realism** | Unrealistic (~2/sec) | Realistic (on-demand) |
| **Mission awareness** | None (automatic) | Full (manager decides) |
| **Fail-safe** | Breaks if service unavailable | Continues if service unavailable |
| **Complexity** | Higher (timer + publisher) | Lower (simple service) |

---

## Migration Path

If you ever need faster corrections or autonomous triggering:

**Phase 2: Auto-Drift Detection**
```python
# Monitor covariance instead of time
if ekf_uncertainty > THRESHOLD:
    request_sonar_position()  # Auto-call when needed
```

**Phase 3: Faster Scans**
```python
# If sonar hardware upgraded to scan faster
# Just reduce the 10-minute interval
sonar_update_interval = 300  # 5 minutes instead
```

**Phase 4: Continuous with Buffering**
```python
# If hardware allows, could:
# - Run sonar in background
# - Buffer results
# - Apply when EKF ready
# Still wouldn't interfere with fast loop
```

---

## Conclusion

The **on-demand service approach** is superior for your system because it:

1. **Respects reality:** Acknowledges actual sonar scan time
2. **Efficient:** Only uses sonar when needed
3. **Clean:** Clear separation of fast and slow sensors
4. **Flexible:** Mission manager has control
5. **Robust:** Graceful failure modes
6. **Simple:** Easier to understand and debug

This is a shift from "try to fuse all sensors continuously" to "use best sensor for each job at appropriate rate."
