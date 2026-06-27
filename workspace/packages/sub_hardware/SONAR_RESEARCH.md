# Sonar Localization - Design Summary

## Project Goal

The objective is to implement a **sonar-based localization system** for an autonomous underwater vehicle (AUV) using the BlueRobotics Ping360 within a ROS2 architecture.

The sonar is **not** intended to perform continuous localization or SLAM. Instead, it is used as a slow but absolute positioning sensor to periodically correct the drift accumulated by the IMU and DVL.

The Mission Manager will request a sonar localization approximately every few minutes (around 10 minutes) depending on mission requirements.

The final pose estimate is injected into `robot_localization` in order to correct the EKF state.

---

# Existing Sensors

## IMU

The IMU provides:

* angular velocity
* roll
* pitch
* yaw estimate

The gyroscope provides an excellent short-term estimate of orientation.

However, the absolute yaw relies on the magnetometer, which is heavily disturbed by the electromagnetic environment inside the AUV.

Consequently:

* short-term orientation changes are accurate;
* long-term yaw drifts significantly.

The sonar must therefore also estimate the yaw correction.

---

## DVL

The DVL provides excellent local velocity estimation.

Its position estimate remains usable for several minutes but inevitably accumulates drift.

The sonar periodically resets this drift.

---

## Ping360

The Ping360 is the only sensor capable of providing an absolute measurement relative to the pool.

Its disadvantages are:

* slow scanning speed
* several seconds required for a complete scan
* noisy measurements
* possible reflections and obstacles

Its advantages are:

* absolute geometric measurements
* independent from IMU drift
* independent from DVL drift

---

# Environment

The environment is highly constrained.

Assumptions:

* indoor swimming pool
* known dimensions
* walls are planar
* walls are perpendicular
* no SLAM required

Important:

The pool reference frame defines "North", "South", "East" and "West".

These names **do not refer to Earth's magnetic north**.

They only represent the fixed reference frame attached to the pool.

---

# Pool Geometry

Unlike the initial design, the pool cannot always be considered as a closed rectangle.

The pool is approximately 50 meters long and divided into several sections.

The vehicle generally has access to only three surrounding walls.

Therefore, localization cannot assume that four walls are always visible.

The algorithm must operate using the visible walls only.

---

# General Architecture

Continuous loop:

* IMU
* DVL
* Depth sensor

↓

robot_localization EKF

↓

continuous state estimate

Slow loop:

Mission Manager

↓

calls sonar localization service

↓

Ping360 performs scans

↓

Position + yaw estimation

↓

PoseWithCovarianceStamped

↓

robot_localization correction

The sonar localization is implemented as a ROS2 service because scanning requires several seconds.

---

# Main Idea

Instead of reading a single sonar beam toward a wall, the sonar scans an angular sector.

Example:

Expected East wall

-15°

↓

0°

↓

+15°

This provides a small point cloud instead of a single distance measurement.

This offers several advantages:

* obstacle rejection
* wall reconstruction
* yaw estimation
* more robust distance estimation

---

# Yaw Correction

The yaw estimation is probably the most important part of the algorithm.

The IMU cannot provide a reliable absolute heading because of magnetic disturbances.

Instead, the sonar estimates the orientation of the observed wall.

Knowing the expected orientation of the wall inside the pool allows the yaw error to be estimated.

The corrected yaw is then fused inside the EKF.

---

# Proposed Geometric Method

A simple trigonometric approach is preferred over heavy computer vision or SLAM algorithms.

For each wall:

* perform a scan over approximately ±15°
* obtain several sonar measurements

The initial intuition was based on three representative vectors:

* left edge
* center
* right edge

If the sonar is perfectly perpendicular to the wall:

* the center measurement is the shortest
* the two side measurements form symmetric right triangles

If symmetry is lost:

* the wall is rotated with respect to the sonar
* the yaw error can be estimated geometrically

This approach only requires elementary trigonometry.

---

# Possible Improvement

Although the three-vector method is simple, it is sensitive to noisy measurements.

A better approach would use every beam acquired inside the sector.

Typical workflow:

1. Scan a 30° sector.
2. Convert all polar measurements to Cartesian coordinates.
3. Detect points belonging to the wall.
4. Fit a straight line.
5. Compute the wall orientation.
6. Compare with the expected wall orientation.
7. Estimate yaw correction.
8. Compute perpendicular distance to the wall.

This remains computationally inexpensive while being significantly more robust.

---

# Wall Detection

The algorithm should identify the dominant wall inside each scanned sector.

Potential improvements include:

* moving average filtering
* median filtering
* least squares line fitting
* RANSAC (optional)

The objective is not to reconstruct the environment but simply to identify the dominant planar surface.

Small obstacles should ideally be ignored.

---

# Position Estimation

Once wall orientations are known:

* compute the perpendicular distance to each visible wall;
* use the known pool dimensions;
* estimate the AUV position.

When opposite walls are available, both measurements can be averaged.

If only three walls are visible, the position should still remain observable.

---

# Covariance

The covariance does not need to be statistically perfect.

Its purpose is simply to indicate how much confidence the EKF should place in the sonar estimate.

A fixed covariance would already be acceptable.

However, an adaptive covariance could be computed from simple quality indicators such as:

* number of detected walls;
* consistency between measurements;
* line fitting residual;
* percentage of inliers;
* overall scan quality.

Better scans receive a smaller covariance.

Poor scans receive a larger covariance.

The exact statistical value is less important than maintaining a consistent confidence ranking.

---

# Design Philosophy

The project intentionally avoids implementing a full underwater SLAM solution.

Instead, it exploits strong prior knowledge:

* known pool geometry;
* known wall orientations;
* predictable environment.

This greatly simplifies the localization problem.

The sonar acts as an occasional absolute reference used to recalibrate the EKF rather than as a continuous localization sensor.

The overall philosophy is therefore:

* fast sensors provide continuous navigation;
* sonar periodically removes accumulated drift;
* exploit geometry instead of building a map.

---

# Potential Mathematical Tools

The implementation should remain lightweight.

Useful mathematical tools include:

* polar to Cartesian coordinate conversion;
* elementary trigonometry;
* vector projections;
* dot products;
* perpendicular distance from a point to a line;
* least squares line fitting;
* optional RANSAC for outlier rejection;
* moving average or median filtering.

Heavy computer vision or SLAM algorithms are intentionally avoided unless experimentation later proves them necessary.

---

# Open Questions

Several implementation choices remain to be validated experimentally:

* Optimal scan width (±10°, ±15°, ±20°...)
* Beam resolution
* Best wall detection method
* Required amount of filtering
* Adaptive covariance strategy
* Handling of partial wall visibility
* Robust obstacle rejection

The implementation should therefore remain modular so different wall detection and fitting algorithms can easily be tested.
