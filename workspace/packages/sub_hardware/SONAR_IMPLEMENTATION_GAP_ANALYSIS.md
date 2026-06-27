# Sonar Implementation Gap Analysis

**Date:** 2026-06-26  
**Scope:** Compare current `sonar_node.py` and `sonar_localization.py` implementation against requirements in `SONAR_RESEARCH.md` and `SONAR_ARCHITECTURE_DESIGN.md`

---

## Summary

Current implementation provides:
- ✅ Basic sonar hardware interface (Ping360)
- ✅ Angle-based scanning capability
- ✅ 4-wall triangulation logic
- ✅ Position estimation from wall distances
- ✅ EKF-compatible pose covariance output

**Missing critical features:**
- ❌ **Yaw estimation from wall orientation** (described as "probably the most important part" in SONAR_RESEARCH.md)
- ❌ **Wall detection via line fitting** (core algorithm in research)
- ❌ **Robust point cloud processing** (outlier rejection, obstacle handling)
- ❌ **Adaptive covariance** (quality-based confidence)
- ❌ **Partial wall visibility handling** (3-wall fallback for larger pools)

---

## Detailed Gap Analysis

### 1. YAW ESTIMATION — **CRITICAL** ❌

**Research Requirement (SONAR_RESEARCH.md, lines 177-187):**
> "The yaw estimation is probably the most important part of the algorithm."
> 
> "The IMU cannot provide a reliable absolute heading because of magnetic disturbances."
> 
> "Instead, the sonar estimates the orientation of the observed wall. Knowing the expected orientation of the wall inside the pool allows the yaw error to be estimated."

**Algorithm Described:**
1. Scan ±15° around expected wall direction (e.g., 0° for right wall)
2. Obtain multiple sonar measurements forming point cloud
3. Fit a straight line to wall points
4. Compute actual wall orientation from line slope
5. Compare to expected wall orientation (0° for right wall = vertical)
6. Difference = yaw error estimate
7. Return corrected yaw as quaternion

**Current Implementation:**
```python
# sonar_localization.py, line 121
pose.pose.pose.orientation.w = 1.0  # Identity quaternion (no yaw info)

# sonar_localization.py, lines 128-130
covariance[21] = 1e6      # roll (no info)
covariance[28] = 1e6      # pitch (no info)
covariance[35] = 1e6      # yaw (no info)  ← Very high variance = no confidence
```

**Impact:**
- IMU yaw drift is NOT corrected by sonar
- Over 10+ minute missions, IMU yaw can drift significantly (magnetometer disturbances)
- Navigation becomes unreliable without correct heading
- Mission controller cannot trust orientation for path planning

**Implementation Required:**
- Modify `_scan_at_angle()` to return point cloud, not single peak
- Add `_fit_wall_line()` function using least-squares line fitting
- Add `_estimate_wall_orientation()` to compute line slope angle
- Add `_estimate_yaw_correction()` to compare detected vs expected orientation
- Update response covariance with yaw estimate (lower variance if high confidence)

---

### 2. WALL DETECTION VIA LINE FITTING — **CORE ALGORITHM** ❌

**Research Requirement (SONAR_RESEARCH.md, lines 220-237):**
> "A better approach would use every beam acquired inside the sector."
> 
> "Typical workflow:
> 1. Scan a 30° sector.
> 2. Convert all polar measurements to Cartesian coordinates.
> 3. Detect points belonging to the wall.
> 4. Fit a straight line.
> 5. Compute the wall orientation.
> 6. Compare with the expected wall orientation.
> 7. Estimate yaw correction.
> 8. Compute perpendicular distance to the wall."

**Current Implementation (sonar_node.py):**
```python
def estimate_wall_distance(self, data):
    # Single peak detection
    # Find first sample above threshold
    above_threshold = np.where(samples > threshold)[0]
    if len(above_threshold) > 0:
        idx = above_threshold[0]
    # Returns SINGLE distance per angle
    return idx, strength
```

**sonar_localization.py:**
```python
def _scan_at_angle(self, angle_deg: int, beam_width: int = 10):
    # Scans a sector but only extracts MINIMUM distance
    min_distance = float('inf')
    for laser_echo in scan_msg.ranges:
        min_distance = min(min_distance, distance)
    return min_distance  # Single value, not point cloud
```

**Problem:**
- Returns only closest point (minimum distance)
- Loses entire point cloud structure
- Cannot fit a line (no points to fit!)
- Cannot detect wall orientation
- Cannot distinguish wall from random obstacle

**Algorithm Described in Research:**
```
For each scan sector:
  1. Collect all (angle, distance) pairs
  2. Convert to (x, y) Cartesian coordinates
  3. Fit line: y = mx + b
  4. Extract perpendicular distance (y-intercept projection)
  5. Extract line angle: arctan(m) = wall orientation
  6. Compare to expected orientation → yaw error
```

**Implementation Required:**
- Collect full point cloud per sector (not minimum distance)
- Add `_polar_to_cartesian()` for coordinate conversion
- Add `_fit_line_to_points()` using NumPy least squares
- Return (distance, orientation, quality) tuple, not single value

---

### 3. ROBUST POINT CLOUD PROCESSING — **NOISE REJECTION** ❌

**Research Requirement (SONAR_RESEARCH.md, lines 241-254):**
> "Wall Detection: The algorithm should identify the dominant wall inside each scanned sector."
> 
> "Potential improvements include:
> - moving average filtering
> - median filtering
> - least squares line fitting
> - RANSAC (optional)"
> 
> "Small obstacles should ideally be ignored."

**Current Implementation:**
- Single threshold-based peak detection (`estimate_wall_distance()`)
- No filtering applied
- No outlier rejection
- Simple 25th percentile noise floor estimation

**Missing:**
- ❌ Moving average filtering (temporal smoothing)
- ❌ Median filtering (outlier rejection)
- ❌ RANSAC for robust line fitting (handles 20-30% outliers)
- ❌ Distinction between wall points and obstacle points

**Failure Mode Example:**
```
Sonar data in a sector:
  Angle  Distance  Interpretation
  -15°   1.8m      Wall point
  -10°   1.9m      Wall point
  -5°    2.0m      Wall point
  0°     2.1m      Wall point
  +5°    5.0m      ← OBSTACLE (returns it as "wall")
  +10°   2.0m      Wall point
  +15°   1.9m      Wall point

Current: Finds min = 1.8m ✓ (works by accident)
But: Loses obstacle info, cannot fit line with outlier
Better: RANSAC ignores +5° outlier, fits line to other 6 points ✓
```

**Implementation Required:**
- Apply moving average to raw amplitude data
- Use RANSAC or robust line fitting (scipy.stats.linregress with error weighting)
- Classify inliers vs outliers
- Track inlier percentage → feeds adaptive covariance

---

### 4. ADAPTIVE COVARIANCE — **QUALITY CONFIDENCE** ❌

**Research Requirement (SONAR_RESEARCH.md, lines 272-292):**
> "Covariance: The covariance does not need to be statistically perfect... Its purpose is simply to indicate how much confidence the EKF should place in the sonar estimate."
> 
> "However, an adaptive covariance could be computed from simple quality indicators such as:
> - number of detected walls
> - consistency between measurements
> - line fitting residual
> - percentage of inliers
> - overall scan quality"
> 
> "Better scans receive a smaller covariance. Poor scans receive a larger covariance."

**Current Implementation (sonar_localization.py, lines 124-131):**
```python
covariance = [0.0] * 36
covariance[0] = self.position_variance   # x variance = 0.25 (FIXED)
covariance[7] = self.position_variance   # y variance = 0.25 (FIXED)
covariance[14] = 1e6                     # z variance (no info)
covariance[21] = 1e6                     # roll (no info)
covariance[28] = 1e6                     # pitch (no info)
covariance[35] = 1e6                     # yaw (no info)
```

**Problem:**
- Covariance is **fixed parameter**, not quality-based
- No distinction between:
  - Excellent scan (4 walls detected, high SNR, low residuals) → should have low variance
  - Poor scan (1 wall detected, low SNR, high residuals) → should have high variance
- EKF cannot distinguish high-confidence from low-confidence measurements

**Quality Metrics NOT Computed:**
- ❌ Number of detected walls (currently: assume all 4)
- ❌ Consistency between opposite walls (e.g., |x_left - x_right| < threshold)
- ❌ Line fitting residual (RMS distance from points to fitted line)
- ❌ Inlier percentage (how many points fit the line well)
- ❌ Signal-to-noise ratio (amplitude / noise floor)

**Implementation Required:**
- Track quality metrics during wall detection
- Compute composite quality score (0-1)
- Map to covariance: `variance = base_variance / quality_score`
- Example: Perfect scan (quality=1.0) → variance=0.25; Poor scan (quality=0.5) → variance=0.50

---

### 5. PARTIAL WALL VISIBILITY HANDLING — **POOL SIZE DEPENDENT** ⚠️

**Research Requirement (SONAR_ARCHITECTURE_DESIGN.md, lines 93-100):**
> "Unlike the initial design, the pool cannot always be considered as a closed rectangle."
> 
> "The pool is approximately 50 meters long and divided into several sections."
> 
> "The vehicle generally has access to only three surrounding walls."
> 
> "Therefore, localization cannot assume that four walls are always visible. The algorithm must operate using the visible walls only."

**Current Implementation (sonar_localization.py, lines 151-209):**
```python
def _scan_four_walls(self) -> Tuple[Optional[float], Optional[float]]:
    """
    Scan in 4 directions to triangulate from walls.
    """
    for angle in scan_angles:  # [0, 90, 180, 270]
        distance = self._scan_at_angle(angle, beam_width=10)
        measured_distances.append(distance)
    
    # Triangulate position from wall distances
    x_estimates = []
    y_estimates = []
    
    # Right wall (0°): x = pool_x - distance
    if measured_distances[0] is not None:
        x_estimates.append(self.pool_x - measured_distances[0])
    
    # ... similar for other walls
    
    x = np.mean(x_estimates) if x_estimates else None
    y = np.mean(y_estimates) if y_estimates else None
```

**Current Handling:**
- ✅ Handles missing detections (if distance is None, skip)
- ✅ Tries to average available estimates
- ✅ Has fallback `_scan_two_walls()` for X-only estimation

**Missing:**
- ❌ Configuration parameter for pool dimensions (currently assumes 5m × 10m)
- ❌ Detection/validation of which walls are actually visible
- ❌ Diagnostic logging of "wall 0° not detected" vs "detection failed"
- ❌ Handling of 3-wall scenarios (one wall missing)
- ❌ Pool size auto-detection (mentioned as future feature in research)

**Real-World Challenge:**
If AUV is in a 50m pool with sections, only nearby walls are detectable:
```
Pool section layout:
  ┌─────────────────────────────┐
  │ Section 1     │ Section 2   │
  │    5m × 10m   │   5m × 10m  │
  │               │             │
  │  ● Sub        │ ● Sub       │
  │               │             │
  └─────────────────────────────┘

AUV in Section 1:
  Can see: Left, Right, Near walls (3 walls)
  Cannot see: Far wall (blocked by section divider)

Current code: Gets None for far wall, continues
But no warning that configuration (pool_y=10m) may be wrong
```

**Implementation Required:**
- Add wall visibility detection/logging
- Validate position against known pool dimensions
- Handle gracefully when only 3 walls available
- Add configuration for multi-section pools

---

### 6. SCAN CONFIGURATION — **PARAMETERS NOT ALIGNED** ⚠️

**Research Requirement (SONAR_RESEARCH.md, lines 338-349):**
> "Several implementation choices remain to be validated experimentally:
> - Optimal scan width (±10°, ±15°, ±20°...)
> - Beam resolution
> - Best wall detection method
> - Required amount of filtering"
> 
> "The implementation should therefore remain modular so different wall detection and fitting algorithms can easily be tested."

**Current Implementation:**
- Hard-coded `beam_width=10` in `_scan_four_walls()` (line 170)
- Hard-coded `angle_increment = 1.0` degrees in sonar_node.py (line 131)
- No configuration parameters for scan strategy

**Missing:**
- ❌ ROS parameter for beam width (should be 10-20° configurable)
- ❌ Parameter for angular resolution
- ❌ Parameter for filtering strategy selection
- ❌ Parameter for line fitting algorithm (least squares vs RANSAC)

**Recommendation from Research:**
- ±15° around wall is more robust than ±10°
- Higher resolution (0.5° instead of 1°) improves line fitting
- RANSAC preferred over least squares for noisy real-world data

---

## Implementation Status (updated 2026-06-26)

### Files changed

| File | Change type | Notes |
|------|-------------|-------|
| `sub_interfaces/srv/SonarScan.srv` | **uint8 → uint16** for angles & range | Required rebuild — see below |
| `sub_hardware/scripts/sonar_node.py` | Bug fixes + wraparound | See "Pre-existing bugs" section |
| `sub_control/scripts/sonar_localization.py` | **Full algorithm rewrite** | All 5 gaps addressed |
| `sub_control/config/robot_localization.yaml` | Enable yaw slot in `pose0_config` | Gap #1 wiring |
| `sub_hardware/config/params.yaml` | Add `port` and `baudrate` defaults | Prevents startup crash |

### Pre-existing bugs fixed (not in gap analysis)

1. **`request.end_angle` / `request.range`**: sonar_node.py read non-existent fields; corrected to `stop_angle` / `desired_range`.
2. **`scan_msg.intensities` type**: was `float[]`; `MultiEchoLaserScan` requires `LaserEcho[]`.
3. **Missing `declare_parameter`**: `baudrate` and `port` caused `ParameterNotDeclaredException` at startup.
4. **ROS2 service-in-service deadlock**: `rclpy.spin_until_future_complete(self, …)` inside a service callback deadlocks with single-threaded executor. Fixed with `ReentrantCallbackGroup` + `MultiThreadedExecutor`.
5. **Angle wraparound**: East wall scan (345°→15°) requires wrap through 0°; now handled in sonar_node.py angle list and by angle_min + i × increment sequential indexing.
6. **EKF yaw not wired**: `pose0_config` yaw slot was `false`; sonar yaw was estimated but never fused. Now `true`.

### Gap analysis completion

| Gap | Status | Implementation |
|-----|--------|---------------|
| **Yaw estimation** | ✅ Implemented | `_analyze_wall`: SVD normal → `atan2` → `wrap_to_π(θ_exp − θ_det)` |
| **Wall detection via line fitting** | ✅ Implemented | `_polar_to_cartesian` + `_fit_wall_svd` (SVD/PCA) |
| **Outlier rejection** | ✅ Implemented | `_ransac_line_fit` (30 iterations, 0.15 m threshold) |
| **Adaptive covariance** | ✅ Implemented | `_adaptive_variance`: quality = f(inlier_ratio, residual_rms, n_points) |
| **Partial wall visibility** | ✅ Implemented | Per-wall optional; position triangulated from available axes only |
| **Scan configuration** | ✅ Implemented | ROS params: `scan_half_width_deg`, `ransac_inlier_thresh`, etc. |

### Required container rebuild

`SonarScan.srv` changed from `uint8` to `uint16`. Run inside the container:
```bash
colcon build --packages-select sub_interfaces sub_hardware sub_control --symlink-install
source install/setup.bash
```

### What is NOT verified (requires hardware / runtime)

- **Yaw sign convention**: Verified analytically for +10° CCW example, but not tested on physical hardware.
- **ReentrantCallbackGroup deadlock fix**: Correct per ROS2 documentation; not exercised in simulation.
- **Angle wraparound** (East wall, 345°→15°): Logic is correct; not tested end-to-end through the device.
- **RANSAC seed=42**: Deterministic for repeatability; real-world convergence depends on actual sonar noise.

### Tuning guidance for pool deployment

| Parameter | Default | Increase if | Decrease if |
|-----------|---------|-------------|-------------|
| `scan_half_width_deg` | 20° | Yaw drift > 15° regularly | Scan time too slow |
| `ransac_inlier_thresh` | 0.15 m | Pool has many obstacles | Clean reflections |
| `base_position_variance` | 0.25 m² | Noisy returns | Very stable returns |
| `base_yaw_variance` | 0.05 rad² | Noisy yaw estimates | Very stable |
| `ransac_iterations` | 30 | >30 % outlier rate | Speed is critical |

---

**Document Status:** Gap analysis complete — implementation applied  
**Last Updated:** 2026-06-26
