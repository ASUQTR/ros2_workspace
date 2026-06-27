# Sonar 3-Wall Orientation-Free Initialization — Gap Analysis

**Date:** 2026-06-26  
**Scope:** Capability to determine AUV position AND pool-frame orientation using 3 detected walls when initial yaw is unknown

---

## Problem Statement

### Current Implementation Assumption

The sonar localization service (implemented 2026-06-26) assumes:
- **AUV's yaw is known within ±20° of cardinal directions** (East, North, West, South)
- Scans are directed at hardcoded angles (0°, 90°, 180°, 270°) in the AUV frame
- Walls are identified **by scan direction**, not by geometry

```python
# Current: wall identity is hardcoded
for cfg in WALL_CONFIGS:
    result = self._analyze_wall(
        wall_name='East',              # ← assumes you know which wall
        scan_center_deg=0,             # ← assumes yaw≈0
        expected_inward_normal_deg=180,
        ...
    )
```

**Failure case:** If AUV is oriented arbitrarily (e.g., facing Northwest at yaw=120°):
- Scans "0°" (East scan in AUV frame) but hits **North wall** (pool frame)
- Algorithm labels it as "East wall" → **wrong position/yaw**
- No recovery mechanism

### Real-World Scenario

**Pool geometry (per SONAR_ARCHITECTURE_DESIGN.md):**
- 50 meters long, divided into sections
- AUV enters a new section with **no prior position/orientation knowledge**
- Cannot rely on magnetometer (EM disturbances)
- Can see 3 walls before compass yaw stabilizes

**Required capability:**
1. **Discovery phase:** Detect available walls without prior orientation assumption
2. **Geometric reasoning:** Identify walls from their relative positions/angles
3. **Orientation solve:** Compute pool-frame orientation from wall geometry
4. **Position triangulation:** Use corrected orientation to find (x, y)

---

## Mathematical Foundation

### Rectangular Pool Geometry

Assume a rectangular pool in the global (pool) frame:
- **East wall** at x = x_max, normal at 180° (points West, inward)
- **North wall** at y = y_max, normal at 270° (points South, inward)
- **West wall** at x = 0, normal at 0° (points East, inward)
- **South wall** at y = 0, normal at 90° (points North, inward)

**Key property:** Opposite walls have parallel normals (180° apart in angle):
- East normal 180° vs West normal 0° → differ by 180°
- North normal 270° vs South normal 90° → differ by 180°

**Key property:** Adjacent walls have perpendicular normals (90° apart):
- East (180°) vs North (270°) → differ by 90°
- North (270°) vs West (0°) → differ by 90°

### AUV Coordinate System

AUV has its own frame:
- x_AUV = forward (AUV's heading direction)
- y_AUV = left (perpendicular to heading)
- yaw_AUV = rotation angle from pool-frame x-axis to AUV x-axis (positive = CCW)

**Transformation:** A vector measured in AUV frame is rotated by -yaw_AUV to get pool frame:
```
v_pool = R(-yaw_AUV) × v_AUV

where R(θ) = [cos(θ), -sin(θ)]
             [sin(θ),  cos(θ)]
```

### Sonar Detections

For each detected wall, sonar returns (in AUV frame):
- **θ_detect_AUV:** angle to the wall in AUV frame (radians, 0 = AUV forward)
- **d_perp:** perpendicular distance from AUV origin to wall (meters)
- **n̂_detect_AUV:** wall normal direction in AUV frame (unit vector)

**Transformation to pool frame:**
```
n̂_detect_pool = R(-yaw) × n̂_detect_AUV
               = [cos(yaw) × n̂_x_AUV - sin(yaw) × n̂_y_AUV,
                  sin(yaw) × n̂_x_AUV + cos(yaw) × n̂_y_AUV]

θ_detect_pool = atan2(n̂_y_pool, n̂_x_pool)
```

---

## The 3-Wall Initialization Problem

### Given

Three detected walls with (in AUV frame):
- Wall A: angle θ_A, distance d_A, normal angle φ_A
- Wall B: angle θ_B, distance d_B, normal angle φ_B
- Wall C: angle θ_C, distance d_C, normal angle φ_C

### Unknowns

1. **yaw:** AUV's orientation in pool frame (±unknown)
2. **Wall identities:** Which wall is East/North/West/South?
3. **(x, y):** AUV's position in pool frame

### Constraints

1. **Wall perpendicularity:** Adjacent walls in pool frame differ in normal angle by 90°
2. **Wall opposition:** Opposite walls in pool frame differ in normal angle by 180° (or 0°)
3. **Pool geometry:** Position estimates from opposite walls should be consistent
4. **Boundary:** Position must be inside pool bounds [0, x_max] × [0, y_max]

---

## Algorithm: Orientation-Free Discovery

### Phase 1: Full Perimeter Scan

**Current approach:** Scan 4 hardcoded cardinal directions (0°, 90°, 180°, 270°)  
**Required approach:** Scan full 360° (or at least 180° to guarantee finding multiple walls)

**Why 180°:** A rectangular pool is symmetric. Scanning any 180° arc will detect at least 2 walls (and usually 3).

```python
def discover_walls_full_sweep(self) -> List[WallDetection]:
    """Scan 0°–359° at 1° resolution (or coarser, e.g., 5°).
    
    Returns list of detected walls, each with:
      - θ_AUV: detection angle in AUV frame (0–359°)
      - d_perp: perpendicular distance
      - n̂: wall normal in AUV frame [n_x, n_y]
    """
    walls = []
    for angle_deg in range(0, 360, 5):  # 5° resolution → ~72 scans
        points = self._scan_wall_sector(angle_deg)
        if not points:
            continue
        
        # Fit a line (as before)
        wall_normal, centroid, residual = self._fit_wall_svd(...)
        d_perp = abs(np.dot(centroid, wall_normal))
        
        walls.append(WallDetection(
            angle_auw=angle_deg,
            distance=d_perp,
            normal=wall_normal,
            quality=...,
        ))
    
    return walls
```

**Complexity:** Full 360° scan takes ~5–10 minutes (vs. ~2 sec for 4 cardinal directions).  
**Trade-off:** Could reduce resolution or scan only until 3+ walls detected.

### Phase 2: Wall Clustering & Deduplication

Multiple scans 90° apart might hit the **same wall** at different angles (e.g., scanning East hits a wall at 0° and scanning North hits the same wall at 85°).

**Cluster walls by their normal angle (not scan angle):**

```python
def cluster_walls_by_normal(walls: List[WallDetection]) -> List[List[WallDetection]]:
    """Group detections that come from the same wall (similar normal angle).
    
    Two detections are the same wall if their normal angles differ by < 30°
    (accounting for 180° wrapping: |θ₁ − θ₂| < 30° OR |θ₁ − θ₂ − 180°| < 30°).
    """
    clusters = []
    for wall in walls:
        # Try to add to existing cluster
        added = False
        for cluster in clusters:
            mean_normal = np.mean([w.normal for w in cluster], axis=0)
            mean_angle = atan2(mean_normal[1], mean_normal[0])
            
            wall_angle = atan2(wall.normal[1], wall.normal[0])
            angular_diff = min(
                abs(wall_angle - mean_angle),
                abs(wall_angle - mean_angle - pi),  # wrap at 180°
            )
            
            if angular_diff < radians(30):  # within 30°
                cluster.append(wall)
                added = True
                break
        
        if not added:
            clusters.append([wall])
    
    return clusters

def select_best_wall_per_cluster(clusters) -> List[WallDetection]:
    """For each cluster, select the detection with highest quality."""
    return [max(cluster, key=lambda w: w.quality) for cluster in clusters]
```

**Output:** Deduplicated list of 3–4 unique walls.

### Phase 3: Wall Identity Resolution

Given 3–4 deduplicated walls with normal angles, determine which is East/North/West/South.

#### 3.1: Classify as Perpendicular vs Opposite Pairs

```python
def classify_wall_pairs(walls: List[WallDetection]) -> Dict:
    """Classify all pairs of walls as perpendicular or opposite.
    
    For each pair (i, j):
      angle_diff = wrap_to_180(|θ_i − θ_j|)
      
      If angle_diff ≈ 90° → perpendicular (adjacent walls)
      If angle_diff ≈ 0° or 180° → opposite (same axis)
    """
    perpendicular_pairs = []
    opposite_pairs = []
    
    for i, wall_i in enumerate(walls):
        for j, wall_j in enumerate(walls[i+1:], i+1):
            angle_i = atan2(wall_i.normal[1], wall_i.normal[0])
            angle_j = atan2(wall_j.normal[1], wall_j.normal[0])
            
            diff = angle_i - angle_j
            diff = atan2(sin(diff), cos(diff))  # wrap to [-π, π]
            diff_deg = degrees(diff)
            
            if abs(diff_deg - 90) < 20:
                perpendicular_pairs.append((i, j))
            elif abs(diff_deg) < 20 or abs(diff_deg - 180) < 20:
                opposite_pairs.append((i, j))
    
    return {
        'perpendicular': perpendicular_pairs,
        'opposite': opposite_pairs,
    }
```

#### 3.2: Build a Constraint Graph

Each wall must be one of {East, North, West, South}. Constraints:
- East ⊥ North, South (perpendicular)
- East ∥ West (opposite)
- North ⊥ East, West (perpendicular)
- North ∥ South (opposite)

This is a **2-coloring + 4-labeling constraint satisfaction problem** (CSP).

For 3 walls, there are only `4 × 3 × 2 = 24` possible assignments (4 choices for wall 1, 3 for wall 2, 2 for wall 3).

```python
def solve_wall_identities(walls: List[WallDetection], pairs: Dict) -> Optional[List[str]]:
    """Brute-force CSP: find assignment of walls to {East, North, West, South}.
    
    Returns: List of wall identities, e.g., ['East', 'North', 'South']
             or None if no valid assignment exists.
    """
    wall_names = ['East', 'North', 'West', 'South']
    perp_pairs = pairs['perpendicular']
    opp_pairs = pairs['opposite']
    
    # Define perpendicularity and opposition relationships
    perpendicular = {
        'East': {'North', 'South'},
        'North': {'East', 'West'},
        'West': {'North', 'South'},
        'South': {'East', 'West'},
    }
    opposite = {
        'East': {'West'},
        'North': {'South'},
        'West': {'East'},
        'South': {'North'},
    }
    
    n_walls = len(walls)
    
    # Try all possible assignments
    for assignment in itertools.permutations(wall_names, n_walls):
        # Check perpendicularity constraints
        valid = True
        for (i, j) in perp_pairs:
            if assignment[j] not in perpendicular[assignment[i]]:
                valid = False
                break
        
        if not valid:
            continue
        
        # Check opposition constraints
        for (i, j) in opp_pairs:
            if assignment[j] not in opposite[assignment[i]]:
                valid = False
                break
        
        if valid:
            return list(assignment)
    
    return None
```

**For 3 walls:** Usually only 1–2 valid assignments (often exactly 1).  
**For 4 walls:** Usually exactly 1 valid assignment.

### Phase 4: Solve for Yaw

Once walls are identified (e.g., wall 0 = East, wall 1 = North, wall 2 = South), each wall provides a yaw estimate:

```
For East wall:
  n̂_East_pool = [0, -1]  (points inward, toward AUV)  [in pool frame]
  θ_East_pool = atan2(-1, 0) = -90° = 270°
  
  n̂_East_AUV = [n_x, n_y]  (detected in AUV frame)
  θ_East_AUV = atan2(n_y, n_x)
  
  Rotation: n̂_East_pool = R(-yaw) × n̂_East_AUV
           270° = atan2(sin(-yaw) × n_x + cos(-yaw) × n_y,
                       cos(-yaw) × n_x - sin(-yaw) × n_y)
  
  Simplify: sin(270°) = -1, cos(270°) = 0
           -1 = cos(-yaw) × n_y - sin(-yaw) × n_x
           
  Solve for yaw (algebra depends on specific normal values).
```

**General formula:**

```python
def solve_yaw_from_wall(wall: WallDetection, wall_name: str) -> float:
    """Estimate AUV's yaw from a single wall detection.
    
    Parameters
    ----------
    wall : WallDetection with normal n̂_AUV = [n_x, n_y]
    wall_name : 'East', 'North', 'West', or 'South'
    
    Returns
    -------
    yaw : angle in radians, [-π, π]
    """
    # Expected inward normal in pool frame (after alignment)
    expected_normals = {
        'East':  (0, -1),   # points West (inward from East wall)
        'North': (-1, 0),   # points South (inward from North wall)
        'West':  (0, 1),    # points East  (inward from West wall)
        'South': (1, 0),    # points North (inward from South wall)
    }
    
    n_expected = np.array(expected_normals[wall_name])
    n_detected = wall.normal  # in AUV frame
    
    # To transform from AUV to pool frame:
    #   n_pool = R(-yaw) × n_AUV
    # 
    # We have:  n_expected = R(-yaw) × n_detected
    # Rearrange: n_detected = R(+yaw) × n_expected  [same as above, backward direction]
    # So:       atan2(n_detected) = yaw + atan2(n_expected)
    
    theta_expected = atan2(n_expected[1], n_expected[0])
    theta_detected = atan2(n_detected[1], n_detected[0])
    
    yaw = wrap_to_pi(theta_detected - theta_expected)
    
    return yaw

def solve_yaw_from_all_walls(walls: List[WallDetection], identities: List[str]) -> float:
    """Estimate yaw from all walls, averaging by quality."""
    yaw_estimates = []
    weights = []
    
    for wall, name in zip(walls, identities):
        yaw_est = solve_yaw_from_wall(wall, name)
        yaw_estimates.append(yaw_est)
        weights.append(wall.quality)
    
    # Circular mean
    sin_sum = sum(w * sin(y) for y, w in zip(yaw_estimates, weights))
    cos_sum = sum(w * cos(y) for y, w in zip(yaw_estimates, weights))
    
    return atan2(sin_sum, cos_sum)
```

**Result:** Best estimate of yaw (±known precision from wall fitting quality).

### Phase 5: Position Triangulation (with known yaw)

Once yaw is known, triangulate position as before:

```python
def triangulate_position(walls: List[WallDetection], identities: List[str], yaw: float) -> Tuple[float, float]:
    """Compute (x, y) position given wall distances and known yaw."""
    
    x_estimates = []
    y_estimates = []
    
    for wall, name in zip(walls, identities):
        d_perp = wall.distance
        
        if name == 'East':
            x_estimates.append((x_max - d_perp, wall.quality))
        elif name == 'West':
            x_estimates.append((d_perp, wall.quality))
        elif name == 'North':
            y_estimates.append((y_max - d_perp, wall.quality))
        elif name == 'South':
            y_estimates.append((d_perp, wall.quality))
    
    x, x_quality = weighted_mean(x_estimates) if x_estimates else (None, 0)
    y, y_quality = weighted_mean(y_estimates) if y_estimates else (None, 0)
    
    return x, y
```

---

## Implementation Gaps (Current vs Required)

| Component | Current | Gap | Effort | Priority |
|-----------|---------|-----|--------|----------|
| **Full 360° scan** | 4 cardinal directions | Need sweep or detection loop | Low (parameter) | HIGH |
| **Wall deduplication** | N/A (assumes 4 distinct) | Cluster walls by normal angle | Medium (clustering algo) | HIGH |
| **Wall identity resolution** | Hardcoded by scan direction | CSP solver (brute-force 24 assignments) | Medium (constraint logic) | HIGH |
| **Yaw from walls** | Assumes each wall is known | Solve from detected normal + expected | Low (algebra) | HIGH |
| **Yaw averaging** | Per-wall estimates | Circular mean | Low (trig) | LOW |
| **Robustness** | Returns None on any failure | Graceful fallback modes | Medium (error handling) | MEDIUM |

---

## Proposed Implementation Phases

### PHASE 1 (Critical): Full Perimeter Discovery

**Goal:** Detect all visible walls without orientation assumption

**Tasks:**
- [ ] Add parameter `discovery_scan_resolution_deg` (default: 5°)
- [ ] Implement `discover_walls_full_sweep()` scanning 0°–360° at resolution
- [ ] Implement `cluster_walls_by_normal()` to deduplicate
- [ ] Return best 3–4 unique walls

**Complexity:** ~200 lines of code  
**Runtime:** 5–10 minutes for full 360° (could be reduced with early-exit when N≥3 walls found)

### PHASE 2 (Critical): Constraint-Based Wall Identification

**Goal:** Map 3 detected walls to East/North/West/South using geometry

**Tasks:**
- [ ] Implement `classify_wall_pairs()` (perpendicular vs opposite)
- [ ] Implement `solve_wall_identities()` CSP solver
- [ ] Add validation: check that inferred position is inside pool bounds
- [ ] Handle ambiguous cases (multiple valid assignments) via quality heuristics

**Complexity:** ~300 lines of code  
**Validation:** Test with synthetic walls (normal angles at 0°, 90°, 180°, 270°)

### PHASE 3 (High): Yaw Solving

**Goal:** Compute yaw from detected wall normals + known identities

**Tasks:**
- [ ] Implement `solve_yaw_from_wall()` for each wall
- [ ] Implement circular mean aggregation
- [ ] Validate yaw wrapping at ±180°

**Complexity:** ~100 lines of code  
**Test:** Verify with rotated wall detections (0°, 45°, 90° yaw examples)

### PHASE 4 (Medium): Robustness & Fallbacks

**Goal:** Handle edge cases gracefully

**Tasks:**
- [ ] If CSP fails (no valid assignment), return error with diagnostics
- [ ] If only 2 walls detected, warn but attempt 2-wall triangulation (x only, y=pool_y/2)
- [ ] If only 1 wall detected, return error (insufficient constraints)
- [ ] Validate final position; reject if outside pool bounds

**Complexity:** ~150 lines of code

---

## Testing Strategy

### Unit Tests

```python
def test_wall_deduplication():
    """Two scans of same wall at 0° and 85° should cluster into one wall."""
    walls = [
        WallDetection(angle=0, normal=[0, -1], distance=2.1, quality=0.9),
        WallDetection(angle=85, normal=[0.087, -0.996], distance=2.0, quality=0.85),
    ]
    clusters = cluster_walls_by_normal(walls)
    assert len(clusters) == 1, "Should cluster into 1 wall"

def test_perpendicularity_detection():
    """Walls at normals 0° and 90° should be perpendicular."""
    walls = [
        WallDetection(normal=[1, 0], ...),   # 0° normal
        WallDetection(normal=[0, 1], ...),   # 90° normal
        WallDetection(normal=[-1, 0], ...),  # 180° normal
    ]
    pairs = classify_wall_pairs(walls)
    assert (0, 1) in pairs['perpendicular'], "Walls 0 & 1 should be perpendicular"
    assert (0, 2) in pairs['opposite'], "Walls 0 & 2 should be opposite"

def test_identity_resolution_3walls():
    """3 walls at correct angles should uniquely identify East, North, South."""
    walls = [
        WallDetection(normal=[0, -1], ...),   # 270° angle → East wall
        WallDetection(normal=[-1, 0], ...),   # 180° angle → North wall
        WallDetection(normal=[1, 0], ...),    # 0° angle   → South wall
    ]
    pairs = classify_wall_pairs(walls)
    identities = solve_wall_identities(walls, pairs)
    assert identities == ['East', 'North', 'South']

def test_yaw_estimation_0deg():
    """If AUV yaw=0° and detects wall normals as expected, yaw estimate should be 0°."""
    wall = WallDetection(normal=[0, -1], ...)  # East wall in AUV frame
    yaw = solve_yaw_from_wall(wall, 'East')
    assert abs(yaw) < radians(5), f"Expected yaw≈0°, got {degrees(yaw)}°"

def test_yaw_estimation_45deg():
    """If AUV yaw=45° CCW, East wall normal rotates by -45°."""
    # East wall normal in pool frame: [0, -1] (270°)
    # After -45° rotation: [sin(45°), -cos(45°)] ≈ [0.707, -0.707] (225° in AUV frame)
    wall = WallDetection(normal=[0.707, -0.707], ...)
    yaw = solve_yaw_from_wall(wall, 'East')
    assert abs(yaw - radians(45)) < radians(5), f"Expected yaw≈45°, got {degrees(yaw)}°"
```

### Integration Tests

```python
def test_full_discovery_cycle_3walls():
    """Simulate sonar detections of 3 walls at arbitrary angles, verify full recovery."""
    # Simulate: AUV at (2, 3) in pool, yaw = 30° CCW
    # Detects: East wall at 0°−5°, North wall at 90°±10°, South wall at 280°
    
    # After discovery:
    walls = simulate_sonar_detections(auw_pos=(2, 3), yaw=30°, visible_walls=[East, North, South])
    
    # Run discovery pipeline
    identities = run_discovery_pipeline(walls)
    yaw_est = solve_yaw_from_all_walls(walls, identities)
    x_est, y_est = triangulate_position(walls, identities, yaw_est)
    
    # Verify recovery
    assert identities == [East, North, South], "Wall identity mismatch"
    assert abs(yaw_est - 30°) < 5°, "Yaw estimation error > 5°"
    assert abs(x_est - 2) < 0.3 and abs(y_est - 3) < 0.3, "Position error > 0.3 m"
```

---

## Known Challenges

### Challenge 1: Orientation Ambiguity (180°)

If you see only 2 walls on the same axis (e.g., East and West), their normals point in opposite directions (0° and 180°). You cannot distinguish which is which without additional information.

**Example:** Detected normals [0°, 180°]  
**Interpretations:**
1. Walls at 0° (East) and 180° (West) — AUV at x ≈ pool_x/2
2. Walls at 180° (East) and 0° (West) — same thing, just labeled differently

**Solution:** The ambiguity doesn't matter for position; both interpretations give the same (x, y). Only yaw estimates differ by 180°. Use the **perpendicular wall** (if visible) to break the 180° ambiguity.

### Challenge 2: Yaw Precision

Yaw estimate accuracy depends on **wall fitting quality** (residual RMS). A noisy fit gives poor yaw estimate, even if the perpendicular distance is accurate.

**Example:**
```
Perfect wall fit: residual = 0.02 m → yaw error ≈ ±3°
Noisy wall fit:   residual = 0.15 m → yaw error ≈ ±15°
```

**Mitigation:** Only fuse yaw corrections when quality is high (residual < 0.1 m).

### Challenge 3: 3 Collinear Walls

**Unlikely but possible:** If AUV is deep in a 50m pool, might detect 3 walls all on one side (e.g., East, part of North, part of South), appearing collinear.

**Detection:** CSP solver will find **no valid assignment** (3 walls cannot satisfy perpendicularity/opposition constraints if collinear).

**Action:** Return error, prompt for manual orientation input or wait for 4th wall to appear.

---

## Integration with Current Implementation

The orientation-free discovery is an **optional initialization phase** that runs once at startup:

```python
class SonarLocalizationNode(Node):
    
    def __init__(self):
        super().__init__('sonar_localization')
        
        self.declare_parameter('initialization_mode', 'known_yaw')  # or 'discovery'
        self.init_mode = self.get_parameter('initialization_mode').value
        
        self.yaw_initialized = False
        self.expected_yaw = None
    
    def sonar_position_callback(self, request, response):
        """Main service callback."""
        
        # Phase 0: Initialization (once, if in discovery mode)
        if not self.yaw_initialized and self.init_mode == 'discovery':
            success, yaw = self._run_discovery_phase()
            if not success:
                response.success = False
                response.message = 'Discovery phase failed'
                return response
            self.expected_yaw = yaw
            self.yaw_initialized = True
        
        # Phase 1–5: Standard scan/fit/triangulate (unchanged)
        x, y, yaw_rad, pos_var, yaw_var = self._scan_all_walls()
        
        # ... rest of callback
```

---

## Deployment Decision Tree

```
AUV startup:
  ├─ "Do you have an initial yaw estimate (within ±20°)?"
  │  ├─ YES → initialization_mode = 'known_yaw'
  │  │        (Use current implementation; fast 2–4 second initialization)
  │  └─ NO  → initialization_mode = 'discovery'
  │           (Run full 360° scan; 5–10 minute initialization)
  │
  └─ After initialization:
     └─ Call get_sonar_position service every 10 minutes (as per architecture)
```

---

## Effort Estimate

| Phase | Lines of Code | Dev Time | Testing | Total |
|-------|--------------|----------|---------|-------|
| 1: Discovery sweep | 150 | 2 hours | 1 hour | **3 hours** |
| 2: Wall identity CSP | 300 | 3 hours | 2 hours | **5 hours** |
| 3: Yaw solving | 100 | 1.5 hours | 1 hour | **2.5 hours** |
| 4: Robustness/fallbacks | 150 | 2 hours | 1.5 hours | **3.5 hours** |
| **Total** | **700** | **8.5 hours** | **5.5 hours** | **14 hours** |

---

## References & Related Work

### Geometric Foundations
- Hartley & Zisserman, *Multiple View Geometry* — Ch. 2 (plane projective geometry)
- Thrun et al., *Probabilistic Robotics* — Ch. 6 (SLAM with known correspondence)

### CSP Solvers
- Constraint programming can be solved with SAT solvers (z3, picosat) or backtracking
- For 4-wall labels, brute-force (24 permutations) is acceptable; 2–3 ms per solve

### Rectangular Room Localization
- Dekeyser et al. (2006): "Recovery of the true orthogonal projection of a plane region imaged under perspective distortion"
- Criminisi et al. (2000): "Single view metrology"  
(Note: these use visual perspective; sonar is simpler — no perspective distortion)

---

**Document Status:** Gap analysis for orientation-free initialization  
**Last Updated:** 2026-06-26  
**Prerequisite:** Complete SONAR_IMPLEMENTATION_GAP_ANALYSIS.md first  
**Next Step:** Decide if deployment scenario requires this capability (see "Deployment Decision Tree")
