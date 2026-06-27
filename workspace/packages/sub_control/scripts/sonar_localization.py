#!/usr/bin/env python3
"""
Sonar-Based Localization Service for AUV in Pool Environment

Architecture
------------
Fast loop  (25 Hz, EKF): IMU + DVL + Bar30 → continuous fused state (growing drift)
Slow loop  (on-demand):  Mission manager calls this service every ~10 min to correct drift

The service:
  1. Scans a ±scan_half_width_deg sector toward each cardinal wall.
  2. Converts sonar beams to a Cartesian point cloud in the AUV frame.
  3. Rejects outliers (obstacles, multi-path echoes) using RANSAC.
  4. Fits a line to the wall inliers using SVD (Principal Component Analysis).
  5. Derives the perpendicular distance to the wall and the wall's orientation.
  6. Triangulates (x, y) from opposite-wall distance pairs.
  7. Estimates yaw error from the mismatch between the detected and expected wall normals.
  8. Returns a PoseWithCovarianceStamped with adaptive covariance for the EKF.

Coordinate conventions
-----------------------
- AUV frame: x = forward (East when aligned with pool), y = left (North when aligned)
  (ROS REP-103: right-hand system, z = up)
- Pool frame: x increases toward East wall, y increases toward North wall
- Yaw positive = counter-clockwise (CCW) rotation from pool frame to AUV frame

Mathematical operations are documented inline.
"""

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

from sub_interfaces.srv import GetSonarPosition, SonarScan


# ---------------------------------------------------------------------------
# Wall configuration table
# ---------------------------------------------------------------------------
# Each entry defines one cardinal wall:
#   scan:   center angle of the scan sector in degrees (AUV frame, yaw=0 assumed)
#   normal: expected inward normal direction in degrees when AUV yaw=0
#           "inward" = pointing from the wall TOWARD the AUV = (scan + 180) % 360
#   axis:   which pool coordinate this wall constrains ('x' or 'y')
#   sign:   +1 → position = pool_dim − d_perp  (East / North walls)
#           −1 → position = d_perp              (West / South walls)
WALL_CONFIGS: List[Dict] = [
    {'name': 'East',  'scan': 0,   'normal': 180, 'axis': 'x', 'sign': +1},
    {'name': 'North', 'scan': 90,  'normal': 270, 'axis': 'y', 'sign': +1},
    {'name': 'West',  'scan': 180, 'normal': 0,   'axis': 'x', 'sign': -1},
    {'name': 'South', 'scan': 270, 'normal': 90,  'axis': 'y', 'sign': -1},
]


@dataclass
class WallResult:
    """Result of analysing a single wall sector."""
    name: str
    axis: str               # 'x' or 'y'
    sign: int               # +1 or -1 (see WALL_CONFIGS)
    perp_distance: float    # perpendicular distance from AUV origin to wall (m)
    yaw_error_rad: float    # yaw error estimated from this wall (rad)
    quality: float          # combined quality score in [0, 1]
    n_inliers: int          # inlier count after RANSAC
    residual_rms: float     # RMS point-to-line distance for inliers (m)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class SonarLocalizationNode(Node):

    def __init__(self):
        super().__init__('sonar_localization')

        # ---- Pool geometry ----
        self.declare_parameter('pool_width_x', 5.0)    # East–West extent (m)
        self.declare_parameter('pool_length_y', 10.0)  # North–South extent (m)
        self.pool_x = self.get_parameter('pool_width_x').value
        self.pool_y = self.get_parameter('pool_length_y').value

        # ---- Sonar hardware ----
        self.declare_parameter('sonar_max_range', 20.0)
        self.sonar_max_range = self.get_parameter('sonar_max_range').value

        # ---- Scan sector ----
        # Half-width of the angular sector scanned around each wall direction.
        # Must exceed the maximum expected yaw error so the wall stays inside
        # the sector.  Default: 20° (handles yaw drift up to ~20° before the
        # wall exits the field of view).
        self.declare_parameter('scan_half_width_deg', 20)
        self.scan_half_width_deg = int(self.get_parameter('scan_half_width_deg').value)

        # Minimum number of valid beams required before attempting a line fit.
        self.declare_parameter('min_points_per_wall', 5)
        self.min_points_per_wall = int(self.get_parameter('min_points_per_wall').value)

        # ---- RANSAC ----
        # Number of iterations.  With up to 30 % outliers and 2-point minimal
        # sample, the probability of at least one all-inlier sample is:
        #   1 − (1 − 0.7²)^30 ≈ 0.99
        self.declare_parameter('ransac_iterations', 30)
        self.declare_parameter('ransac_inlier_thresh', 0.15)  # meters
        self.ransac_iterations = int(self.get_parameter('ransac_iterations').value)
        self.ransac_inlier_thresh = float(self.get_parameter('ransac_inlier_thresh').value)

        # ---- Covariance ----
        self.declare_parameter('base_position_variance', 0.25)  # m²
        self.declare_parameter('base_yaw_variance', 0.05)       # rad²
        self.base_position_variance = float(
            self.get_parameter('base_position_variance').value
        )
        self.base_yaw_variance = float(self.get_parameter('base_yaw_variance').value)

        # Deterministic RNG for RANSAC (fixed seed → reproducible results in tests).
        self._rng = np.random.default_rng(seed=42)

        # ---- ROS interfaces ----
        # ReentrantCallbackGroup is required so that the localization service
        # callback (which internally calls the sonar scan service) does not
        # deadlock.  With a standard MutuallyExclusiveCallbackGroup the outer
        # spin() cannot process the sonar response while waiting inside the
        # service callback.  ReentrantCallbackGroup + MultiThreadedExecutor in
        # main() allows both callbacks to run concurrently.
        self.cb_group = ReentrantCallbackGroup()

        self.sonar_client = self.create_client(
            SonarScan, '/sonar/scan', callback_group=self.cb_group
        )

        self.localization_service = self.create_service(
            GetSonarPosition,
            'get_sonar_position',
            self.sonar_position_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            f'Sonar localization ready  '
            f'pool={self.pool_x:.1f}×{self.pool_y:.1f} m  '
            f'sector=±{self.scan_half_width_deg}°'
        )

    # -----------------------------------------------------------------------
    # Public service callback
    # -----------------------------------------------------------------------

    def sonar_position_callback(self, request, response):
        """Triggered by mission manager.  Runs the full scan→fit→triangulate pipeline."""
        self.get_logger().info('Sonar position request received — scanning walls…')

        if not self.sonar_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Sonar scan service not available')
            response.success = False
            response.message = 'Sonar service unavailable'
            return response

        try:
            x, y, yaw_rad, pos_variance, yaw_variance = self._scan_all_walls()
        except Exception as exc:
            self.get_logger().error(f'Scan pipeline error: {exc}')
            response.success = False
            response.message = f'Pipeline error: {exc}'
            return response

        if x is None or y is None:
            response.success = False
            response.message = 'Insufficient wall detections to triangulate'
            return response

        pose = self._build_pose(x, y, yaw_rad, pos_variance, yaw_variance)
        response.success = True
        response.message = (
            f'x={x:.3f} m  y={y:.3f} m  '
            f'yaw={math.degrees(yaw_rad):.1f}°  '
            f'pos_var={pos_variance:.3f}'
        )
        response.pose = pose

        self.get_logger().info(response.message)
        return response

    # -----------------------------------------------------------------------
    # Wall scanning pipeline
    # -----------------------------------------------------------------------

    def _scan_all_walls(
        self,
    ) -> Tuple[Optional[float], Optional[float], float, float, float]:
        """Scan all four walls and return (x, y, yaw_rad, pos_variance, yaw_variance).

        Returns (None, None, 0, …) when fewer than one wall per axis is available.
        """
        wall_results: List[WallResult] = []

        for cfg in WALL_CONFIGS:
            result = self._analyze_wall(
                wall_name=cfg['name'],
                scan_center_deg=cfg['scan'],
                expected_inward_normal_deg=cfg['normal'],
                axis=cfg['axis'],
                sign=cfg['sign'],
            )
            if result is not None:
                wall_results.append(result)

        if not wall_results:
            return None, None, 0.0, 1e6, 1e6

        # ---- Position triangulation ----
        # For each detected wall:
        #   East wall (sign=+1, axis='x'):  x = pool_x − d_perp
        #   West wall (sign=−1, axis='x'):  x = d_perp
        #   North wall (sign=+1, axis='y'): y = pool_y − d_perp
        #   South wall (sign=−1, axis='y'): y = d_perp
        #
        # Average estimates from opposite walls for robustness.
        x_estimates: List[Tuple[float, float]] = []  # (position, quality)
        y_estimates: List[Tuple[float, float]] = []

        for wr in wall_results:
            pool_dim = self.pool_x if wr.axis == 'x' else self.pool_y
            if wr.sign == +1:
                pos = pool_dim - wr.perp_distance
            else:
                pos = wr.perp_distance

            if wr.axis == 'x':
                x_estimates.append((pos, wr.quality))
            else:
                y_estimates.append((pos, wr.quality))

        if not x_estimates or not y_estimates:
            self.get_logger().warn(
                f'Only walls on one axis detected '
                f'(x={len(x_estimates)}, y={len(y_estimates)})'
            )
            return None, None, 0.0, 1e6, 1e6

        # Quality-weighted mean position per axis.
        x, quality_x = self._weighted_mean(x_estimates)
        y, quality_y = self._weighted_mean(y_estimates)

        # ---- Yaw estimation ----
        # Weighted average of yaw estimates from all walls.
        # Each wall provides one yaw_error_rad observation.
        yaw_obs = [(wr.yaw_error_rad, wr.quality) for wr in wall_results]
        yaw_rad, quality_yaw = self._weighted_circular_mean(yaw_obs)

        # ---- Adaptive covariance ----
        overall_quality = (quality_x + quality_y) / 2.0
        pos_variance = self._adaptive_variance(self.base_position_variance, overall_quality)
        yaw_variance = self._adaptive_variance(self.base_yaw_variance, quality_yaw)

        # Sanity check: positions must be inside pool bounds.
        if not (0.0 <= x <= self.pool_x) or not (0.0 <= y <= self.pool_y):
            self.get_logger().warn(
                f'Position ({x:.2f}, {y:.2f}) outside pool bounds '
                f'[0,{self.pool_x}]×[0,{self.pool_y}] — discarding'
            )
            return None, None, 0.0, 1e6, 1e6

        return x, y, yaw_rad, pos_variance, yaw_variance

    def _analyze_wall(
        self,
        wall_name: str,
        scan_center_deg: int,
        expected_inward_normal_deg: float,
        axis: str,
        sign: int,
    ) -> Optional[WallResult]:
        """Full pipeline for a single wall: scan → Cartesian → RANSAC → SVD → result.

        Parameters
        ----------
        scan_center_deg
            Direction toward the wall in degrees (AUV frame, yaw=0 assumed).
        expected_inward_normal_deg
            Angle of the wall's inward normal when AUV yaw=0; equals
            (scan_center_deg + 180) % 360.
        """
        # ---- Step 1: collect point cloud ----
        polar_points = self._scan_wall_sector(scan_center_deg)
        if len(polar_points) < self.min_points_per_wall:
            self.get_logger().warn(
                f'{wall_name} wall: only {len(polar_points)} points '
                f'(need ≥ {self.min_points_per_wall})'
            )
            return None

        # ---- Step 2: polar → Cartesian ----
        # x_i = d_i · cos(α_i),   y_i = d_i · sin(α_i)
        # where α_i is the beam angle in the AUV frame (radians).
        cartesian = self._polar_to_cartesian(polar_points)

        # ---- Step 3: RANSAC outlier rejection ----
        inlier_mask = self._ransac_line_fit(cartesian)
        inlier_pts = cartesian[inlier_mask]
        n_inliers = int(inlier_mask.sum())
        inlier_ratio = n_inliers / len(cartesian)

        if n_inliers < 2:
            self.get_logger().warn(f'{wall_name} wall: too few inliers after RANSAC')
            return None

        # ---- Step 4: SVD line fit ----
        wall_normal, centroid, residual_rms = self._fit_wall_svd(inlier_pts)

        # ---- Step 5a: perpendicular distance ----
        #
        # The AUV is at the origin of its own frame.  The wall passes through
        # the centroid c of the fitted points.  The wall normal n̂ (unit vector)
        # is perpendicular to the wall.
        #
        # We choose the sign of n̂ so it points TOWARD the AUV (inward normal):
        #   If c · n̂ > 0 the normal points AWAY from the origin → flip it.
        if np.dot(centroid, wall_normal) > 0:
            wall_normal = -wall_normal

        # Perpendicular distance from origin to the wall:
        #   d_perp = |c · n̂|
        # Because n̂ points toward the origin and c is "ahead" of the origin:
        #   c · n̂ < 0  →  d_perp = −(c · n̂) = |c · n̂|
        d_perp = float(abs(np.dot(centroid, wall_normal)))

        # ---- Step 5b: yaw error from wall normal ----
        #
        # If the AUV has a yaw error of φ (positive = CCW rotation from pool
        # frame), the detected inward wall normal is rotated by −φ relative to
        # the expected direction.
        #
        # Detected normal angle in AUV frame:
        #   θ_det = atan2(n̂_y, n̂_x)
        #
        # Expected inward normal angle in AUV frame when yaw=0:
        #   θ_exp = expected_inward_normal_deg × (π / 180)
        #
        # Yaw error:
        #   φ = wrap_to_π(θ_exp − θ_det)
        #
        # Derivation: let the AUV be rotated by φ CCW.  The East-wall inward
        # normal in pool frame is 180°.  In AUV frame it appears at 180° − φ.
        # Therefore: φ = 180° − θ_det = θ_exp − θ_det  ✓
        theta_det = math.atan2(float(wall_normal[1]), float(wall_normal[0]))
        theta_exp = math.radians(expected_inward_normal_deg)
        yaw_error = math.atan2(
            math.sin(theta_exp - theta_det),
            math.cos(theta_exp - theta_det),
        )

        # ---- Step 6: quality score ----
        #
        # A composite score in [0, 1] combining:
        #   • inlier_ratio   (fraction of points not rejected by RANSAC)
        #   • residual quality (how tightly points cluster on the fitted line)
        #   • point count    (more data = more reliable fit)
        #
        # Expected residual for a good sonar wall return: ~0.05–0.10 m.
        # At 0.20 m the fit is considered unreliable (quality → 0).
        residual_quality = max(0.0, 1.0 - residual_rms / 0.20)
        point_quality = min(1.0, n_inliers / 20.0)  # saturates at 20 inliers
        quality = 0.5 * inlier_ratio + 0.3 * residual_quality + 0.2 * point_quality

        self.get_logger().debug(
            f'{wall_name} wall: d_perp={d_perp:.3f} m  '
            f'yaw_err={math.degrees(yaw_error):.1f}°  '
            f'quality={quality:.2f}  '
            f'inliers={n_inliers}/{len(cartesian)}  '
            f'residual={residual_rms:.3f} m'
        )

        return WallResult(
            name=wall_name,
            axis=axis,
            sign=sign,
            perp_distance=d_perp,
            yaw_error_rad=yaw_error,
            quality=quality,
            n_inliers=n_inliers,
            residual_rms=residual_rms,
        )

    # -----------------------------------------------------------------------
    # Scan sector
    # -----------------------------------------------------------------------

    def _scan_wall_sector(self, center_deg: int) -> List[Tuple[float, float]]:
        """Call the sonar hardware service and return valid (angle_rad, distance) pairs.

        The sector spans [center_deg − half_width, center_deg + half_width].
        Angles wrap around 0° naturally: the returned angle_rad values are
        sequential and may exceed 2π; trig functions handle this correctly.

        Returns a list of (angle_rad, distance_m) for beams with a valid echo.
        """
        hw = self.scan_half_width_deg
        start_deg = (center_deg - hw) % 360
        stop_deg = (center_deg + hw) % 360

        req = SonarScan.Request(
            start_angle=start_deg,
            stop_angle=stop_deg,
            desired_range=int(self.sonar_max_range),
        )
        future = self.sonar_client.call_async(req)

        # Busy-wait is safe here because this method runs in a worker thread
        # spawned by the MultiThreadedExecutor.  The executor's other threads
        # can continue processing the sonar response callback while we wait.
        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        if not future.done() or future.result() is None:
            self.get_logger().error(f'Scan request at center={center_deg}° timed out')
            return []

        scan_msg = future.result().data
        points: List[Tuple[float, float]] = []

        for i, laser_echo in enumerate(scan_msg.ranges):
            if not laser_echo.ranges:
                continue
            distance = laser_echo.ranges[0]
            if not (0.1 < distance < self.sonar_max_range):
                continue

            # Reconstruct sequential angle from scan metadata.
            # angle_min is radians(start_deg); incrementing by angle_increment
            # steps through the sector monotonically (wrapping is handled by
            # the trig functions used downstream).
            angle_rad = scan_msg.angle_min + i * scan_msg.angle_increment
            points.append((angle_rad, distance))

        return points

    # -----------------------------------------------------------------------
    # Geometric helpers
    # -----------------------------------------------------------------------

    @staticmethod
    def _polar_to_cartesian(
        polar_points: List[Tuple[float, float]]
    ) -> np.ndarray:
        """Convert polar (α, d) pairs to Cartesian (x, y) in AUV frame.

        For each beam at angle α (radians) measuring distance d (meters):
            x_i = d_i · cos(α_i)   [along AUV forward axis]
            y_i = d_i · sin(α_i)   [along AUV left axis]

        Returns array of shape (N, 2).
        """
        if not polar_points:
            return np.empty((0, 2))
        angles = np.array([p[0] for p in polar_points])
        distances = np.array([p[1] for p in polar_points])
        return np.column_stack([distances * np.cos(angles),
                                distances * np.sin(angles)])

    def _ransac_line_fit(self, points: np.ndarray) -> np.ndarray:
        """Robust line fit using RANSAC; returns boolean inlier mask.

        Algorithm
        ---------
        Repeat ransac_iterations times:
          1. Sample 2 points at random.
          2. Compute the unit normal n̂ to the line through those points:
               t = p2 − p1                     (direction along line)
               n̂ = [−t_y, t_x] / ‖t‖          (perpendicular direction)
          3. Signed distance from each point q to the line:
               dist(q) = |(q − p1) · n̂|
          4. Count inliers: points with dist < inlier_thresh.
        Keep the candidate set with the most inliers.

        If the best set covers fewer than max(2, N//2) points the fallback
        is to use ALL points (low-quality fit; the residual quality metric
        will reduce the covariance weight for this wall).

        Returns
        -------
        inlier_mask : ndarray of bool, shape (N,)
        """
        N = len(points)
        if N < 2:
            return np.ones(N, dtype=bool)

        best_mask = np.ones(N, dtype=bool)
        best_count = 0

        for _ in range(self.ransac_iterations):
            idx = self._rng.choice(N, size=2, replace=False)
            p1, p2 = points[idx[0]], points[idx[1]]

            t = p2 - p1
            t_norm = np.linalg.norm(t)
            if t_norm < 1e-6:
                continue  # degenerate: same point sampled twice

            # Unit normal perpendicular to the line:
            #   n̂ = [−t_y, t_x] / ‖t‖
            n = np.array([-t[1], t[0]]) / t_norm

            # Perpendicular distance from each point to the line:
            #   dist(q) = |(q − p1) · n̂|
            residuals = np.abs((points - p1) @ n)
            mask = residuals < self.ransac_inlier_thresh
            count = int(mask.sum())

            if count > best_count:
                best_count = count
                best_mask = mask

        min_required = max(2, N // 2)
        if best_count < min_required:
            self.get_logger().debug(
                f'RANSAC: weak fit ({best_count}/{N} inliers), using all points'
            )
            return np.ones(N, dtype=bool)

        return best_mask

    @staticmethod
    def _fit_wall_svd(
        points: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Fit a line to 2-D points using Singular Value Decomposition (SVD).

        This is equivalent to finding the first principal component of the
        centred point set (PCA).

        Mathematical derivation
        -----------------------
        Let A be the (N × 2) matrix of centred points:
            A = points − mean(points)

        SVD factorisation:  A = U S Vᵀ
          • Vᵀ[0] = first right singular vector  = direction ALONG the wall
          • Vᵀ[1] = second right singular vector = direction ⊥ to wall (normal)
          • S[0]  = spread of points along the wall direction
          • S[1]  = spread of points perpendicular to the wall

        Residual RMS (mean point-to-line distance):
            rms = S[1] / √N

        Proof:  S[1]² = Σᵢ (Aᵢ · Vᵀ[1])²  (sum of squared perpendicular
        projections onto the normal).  Dividing by N and taking the square
        root gives the RMS.

        Perpendicular distance from AUV origin to wall:
            d_perp = |centroid · Vᵀ[1]|
        (centroid is any point on the wall; its projection onto the unit
        normal equals the signed distance from the origin to the wall plane.)

        Parameters
        ----------
        points : (N, 2) array of inlier Cartesian points

        Returns
        -------
        wall_normal : (2,) unit vector ⊥ to wall (sign not yet determined)
        centroid    : (2,) mean of the input points (a point on the wall)
        residual_rms: float, RMS distance of points from fitted line (m)
        """
        centroid = points.mean(axis=0)
        A = points - centroid  # centre the points

        # SVD of the (N, 2) centred matrix.
        # full_matrices=False: U is (N,2), S is (2,), Vt is (2,2).
        _, S, Vt = np.linalg.svd(A, full_matrices=False)

        wall_normal = Vt[1]          # second right singular vector = wall normal
        residual_rms = float(S[1] / np.sqrt(len(points)))

        return wall_normal, centroid, residual_rms

    # -----------------------------------------------------------------------
    # Aggregation helpers
    # -----------------------------------------------------------------------

    @staticmethod
    def _weighted_mean(estimates: List[Tuple[float, float]]) -> Tuple[float, float]:
        """Quality-weighted mean.  Returns (weighted_mean, mean_quality)."""
        values = np.array([e[0] for e in estimates])
        weights = np.array([e[1] for e in estimates])
        total_w = weights.sum()
        if total_w < 1e-9:
            return float(values.mean()), 0.0
        return float((values * weights).sum() / total_w), float(weights.mean())

    @staticmethod
    def _weighted_circular_mean(
        obs: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """Quality-weighted circular mean of angles (handles ±π wrap-around).

        For angles in [−π, π], the circular mean is computed via unit-vector
        averaging:
            mean_angle = atan2(Σ wᵢ sin(φᵢ),  Σ wᵢ cos(φᵢ))
        """
        sin_sum = sum(w * math.sin(a) for a, w in obs)
        cos_sum = sum(w * math.cos(a) for a, w in obs)
        mean_q = sum(w for _, w in obs) / len(obs)
        return math.atan2(sin_sum, cos_sum), mean_q

    @staticmethod
    def _adaptive_variance(base_variance: float, quality: float) -> float:
        """Scale base variance inversely with quality.

        A perfect scan (quality = 1.0) returns base_variance unchanged.
        A poor scan (quality → 0) can inflate variance at most 10×.

        Formula:  variance = base_variance / clamp(quality, 0.1, 1.0)
        """
        clamped = max(0.1, min(1.0, quality))
        return base_variance / clamped

    # -----------------------------------------------------------------------
    # Response builder
    # -----------------------------------------------------------------------

    def _build_pose(
        self,
        x: float,
        y: float,
        yaw_rad: float,
        pos_variance: float,
        yaw_variance: float,
    ) -> PoseWithCovarianceStamped:
        """Build a PoseWithCovarianceStamped for the EKF.

        Covariance matrix (6×6 flat, row-major):
            Diagonal: [var_x, var_y, var_z, var_roll, var_pitch, var_yaw]
            Off-diagonal: 0  (no cross-correlations estimated by sonar)

        The EKF uses the covariance to weight this measurement against other
        sensors.  Larger variance = less weight.  We set yaw variance from
        the scan quality; z/roll/pitch are set to 1e6 (effectively ignored).

        Quaternion from yaw (rotation about z-axis only):
            q_w = cos(yaw / 2),  q_z = sin(yaw / 2),  q_x = q_y = 0
        """
        pose = PoseWithCovarianceStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        # Position
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0

        # Orientation: yaw-only quaternion
        pose.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)

        # Covariance  (6×6 flat, indices 0–35)
        #   [0] = var_x      [7] = var_y      [14] = var_z
        #   [21]= var_roll   [28]= var_pitch   [35] = var_yaw
        cov = [0.0] * 36
        cov[0] = pos_variance    # x
        cov[7] = pos_variance    # y
        cov[14] = 1e6            # z   (depth from Bar30, not sonar)
        cov[21] = 1e6            # roll  (from IMU)
        cov[28] = 1e6            # pitch (from IMU)
        cov[35] = yaw_variance   # yaw (now estimated from wall normal)
        pose.pose.covariance = cov

        return pose


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SonarLocalizationNode()

    # MultiThreadedExecutor is required to allow the localization service
    # callback to call the sonar scan service without deadlocking:
    #   outer spin() → service callback → call_async → needs spin() to complete
    # With MTE + ReentrantCallbackGroup both callbacks can run concurrently.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sonar localization service')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
