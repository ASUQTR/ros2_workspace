#!/usr/bin/env python3

import scipy.linalg
import numpy as np
import math

# ==============================================================================
# THRUSTER ALLOCATION MATRIX (T)
# ==============================================================================
THRUST_ALLOC_MAT = np.array([[-1, 1, 0, 0, 0, 1],
                             [-1, -1, 0, 0, 0, -1],
                             [0, 0, -1, -1, -1, 0],
                             [0, 0, -1, 1, -1, 0],
                             [0, 0, -1, -1, 1, 0],
                             [0, 0, -1, 1, 1, 0],
                             [1, 1, 0, 0, 0, -1],
                             [1, -1, 0, 0, 0, 1]], dtype=np.int8)

# ==============================================================================
# CONTROL INPUT MATRIX (B Matrix)
# ==============================================================================
Bm = np.zeros((12, 8), dtype=np.float32)

# --- Kinetics (Linear Accelerations: u_dot, v_dot, w_dot) ---
# These coefficients include the 45 degree horizontal thruster geometry and an
# added-mass fit to the Unity vehicle. The sign pattern intentionally mirrors
# THRUST_ALLOC_MAT.T for [surge, sway, heave, roll, pitch, yaw].
# Surge (u_dot) affected by horizontal thrusters
Bm[6][0] = -0.021117481435499889398735391488808
Bm[6][1] = -0.021117481435499889398735391488808
Bm[6][6] = 0.021117481435499889398735391488808
Bm[6][7] = 0.021117481435499889398735391488808

# Sway (v_dot)
Bm[7][0] = 0.01902284924040905501271856144737
Bm[7][1] = -0.01902284924040905501271856144737
Bm[7][6] = 0.01902284924040905501271856144737
Bm[7][7] = -0.01902284924040905501271856144737

# Heave (w_dot) affected by vertical thrusters
Bm[8][2] = -0.026906434569178295633265292004767
Bm[8][3] = -0.026906434569178295633265292004767
Bm[8][4] = -0.026906434569178295633265292004767
Bm[8][5] = -0.026906434569178295633265292004767

# --- Kinetics (Angular Accelerations: p_dot, q_dot, r_dot) ---
# Roll (p_dot)
Bm[9][2] = -0.47145328719723183391003460207612
Bm[9][3] = 0.47145328719723183391003460207612
Bm[9][4] = -0.47145328719723183391003460207612
Bm[9][5] = 0.47145328719723183391003460207612

# Pitch (q_dot)
Bm[10][2] = -0.16128655774647620680552224586649
Bm[10][3] = -0.16128655774647620680552224586649
Bm[10][4] = 0.16128655774647620680552224586649
Bm[10][5] = 0.16128655774647620680552224586649

# Yaw (r_dot)
Bm[11][0] = 0.39463536905835478514559256667363
Bm[11][1] = -0.39463536905835478514559256667363
Bm[11][6] = -0.39463536905835478514559256667363
Bm[11][7] = 0.39463536905835478514559256667363

# ==============================================================================
# UNITY-ALIGNED HYDRODYNAMIC FIT
# ==============================================================================
# Coefficients are positive magnitudes. The dissipative sign (-1) is applied
# directly in A per marine convention: ν_dot = M⁻¹(τ - D(ν)ν - ...).
# Quadratic terms are SDRE-linearized using the current state magnitude: d(v) = linear + quadratic * abs(v).
#
# Marine axes used by the LQR: surge u, sway v, heave w, roll p, pitch q, yaw r.
# Unity mapping in the Hydrodynamics component:
#   local X = sway, local Y = heave, local Z = surge
#   angular X = pitch, angular Y = yaw, angular Z = roll
HYDRO_LINEAR = {
    "surge": 23.9201,
    "sway": 43.6523,
    "heave": 52.9362,
    "roll": 1.0752,
    "pitch": 1.4659,
    "yaw": 1.3090,
}

HYDRO_QUADRATIC = {
    "surge": 26.7035,
    "sway": 80.1106,
    "heave": 117.8097,
    "roll": 3.1250,
    "pitch": 5.0470,
    "yaw": 2.9207,
}

# Effective masses/inertias aligned with the B matrix above. Translational values
# include the Unity added-mass fit currently used in the simulator.
EFFECTIVE_MASS = {
    "surge": 0.7071067811865476 / 0.021117481435499889,
    "sway": 0.7071067811865476 / 0.019022849240409055,
    "heave": 1.0 / 0.026906434569178296,
    "roll": 0.2725 / 0.47145328719723183,
    "pitch": (1.4659 + 5.0470) / 9.789778396523997,
    "yaw": (1.3090 + 2.9207) / 4.613927230130708,
}


# ==============================================================================
# LQR SOLVER CLASS
# ==============================================================================
class SubLQRSolver:
    """
    Executes a State-Dependent Riccati Equation (SDRE) controller strategy.
    
    Instead of solving the LQR once offline, this class continuously re-solves 
    the Riccati equation at runtime to dynamically adjust its tuning gains as 
    hydrodynamic drag and Coriolis forces change during transit.

    ==========================================================================
    LQR OUTPUT UNITS: PROOF OF NEWTONS
    ==========================================================================
    The output of `compute_thrust_force` is interpreted as physical NEWTONS per
    thruster. The state-space equation is $\dot{x} = Ax + Bu$, and the B matrix
    converts each per-thruster force command into the resulting linear/angular
    acceleration. These coefficients already include the thruster allocation
    geometry, Unity mass/inertia, and added-mass fit, so they are not simply
    `1 / mass`.

    ==========================================================================
    LQR LIMITATIONS & SOFTWARE CLIPPING
    ==========================================================================
    LQR is a purely linear controller where the output force ($u$) is calculated as:
    $u = -Kx$ (where $K$ is the gain and $x$ is the error).
    Because it is linear, it mathematically assumes you have infinitely powerful 
    thrusters. 
    * YOU CANNOT LIMIT THE MAX OUTPUT USING THE B MATRIX: Changing B destroys 
      the fundamental physics model (e.g., telling the math the Sub weighs 10,000 kg).
    * YOU CANNOT STRICTLY LIMIT THE OUTPUT USING Q AND R: Because $u$ scales 
      linearly with the error $x$, a massive waypoint jump (e.g., 20m) will ALWAYS 
      produce a massive requested force (e.g., 1000N+).
    
    SOLUTION: You must enforce your physical actuator limits (e.g., 50N) in software. 
    Wrap the output of this solver in `np.clip(thrust_efforts, -50.0, 50.0)` 
    inside your ROS 2 node before sending the commands to the thruster ESCs.

    ==========================================================================
    TUNING GUIDE (Q AND R MATRICES)
    ==========================================================================
    Quick Math Rule of Thumb: The calculated optimal gain matrix ($K$) is roughly 
    proportional to the square root of Q over R: 
    $$K \propto \sqrt{\frac{Q}{R}}$$
    Therefore, the requested thrust in Newtons is roughly: 
    $$u \approx \sqrt{\frac{Q}{R}} \times \text{error}$$

    * R MATRIX (Thruster Cost): Penalizes using thruster energy. 
      Higher R = "Save battery, thrust is expensive." This lowers overall output force.
    * Q MATRIX (State Error Cost): Penalizes being away from the target.
      Higher Q = "Get to the target immediately." This increases overall output force.
        - Position Q (Indices 0-2): Acts like a Spring. Pulls the Sub to the target.
        - Angle Q (Indices 3-5): Acts like a Torsional Spring. Snaps heading.
        - Velocity Q (Indices 6-11): Acts like a Damper (Shock Absorber). 
          High velocity Q forces the Sub to brake smoothly to avoid overshoot.

    --- CONFIGURATION SUGGESTIONS ---
    Suggested R: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    Suggested Q: [0.5, 0.5, 0.5, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    
    Expected Behavior of these suggestions:
    1.  A small 1-meter position error requests roughly 15-20N of force. 
    2.  A 3-meter position error requests roughly 60N of force (which your software 
        node will clip to 50N). The Sub drives at 100% max hardware capacity 
        until it gets within ~2.5m, then the LQR smoothly throttles down.
    3.  The high Velocity Q (1.0) causes the Sub to actively reverse thrusters 
        to kill its momentum right before reaching the target coordinate, ensuring 
        zero overshoot.

    ==========================================================================
    USING TARGET VELOCITIES (STATION KEEPING VS. TRAJECTORY TRACKING)
    ==========================================================================
    The LQR controller's behavior fundamentally changes based on the target 
    velocities (u, v, w, p, q, r) provided in the target state array:

    * TARGET VELOCITY = 0 (Station Keeping / Point-to-Point):
      - Goal: "Get to the target X, Y, Z coordinate and come to a dead stop."
      - Mechanism: As the Sub accelerates toward the target, its speed increases. 
        The LQR sees a growing error between current speed and the target speed (0 m/s). 
        The Velocity Q costs act as dampers (brakes), fighting the forward motion 
        as the Sub nears the target to bring it to a perfect halt.

    * TARGET VELOCITY > 0 (Trajectory Tracking / Cruise Control):
      - Goal: "Pass exactly through the target X, Y, Z coordinate while moving 
        at the specified speed (e.g., 1.0 m/s)."
      - Mechanism: If the Sub is moving at the target speed, the velocity error is 0. 
        The damper completely stops fighting the forward movement. The Sub will 
        smoothly glide through the waypoint without hitting the brakes.
      - Use Cases: Smooth multi-waypoint following (eliminating jerky start/stop 
        behavior), continuous seabed mapping (constant sensor sweeping), or pure 
        forward cruise control.
      - WARNING: If you command a non-zero target velocity, your ROS 2 node MUST 
        continuously update the target position (like a moving carrot on a stick). 
        If you leave the target coordinate static, the Sub will cross the point, 
        realize it overshot, but realize it still needs to maintain 1.0 m/s. It 
        will turn around and perform infinite high-speed figure-eights around 
        the static waypoint.
    """
    def __init__(self):
        # Pre-allocate matrices to prevent heavy garbage collection overhead
        # during high-frequency (e.g., 200Hz) ROS odometry callbacks.
        self.system_dynamics_matrix = np.zeros((12, 12), dtype=np.float32)
        # Last gain matrix successfully computed by the ARE solver.
        # Used as a fallback if the solver fails at a given operating point.
        self._last_valid_k_gain = None
        # Number of consecutive control cycles where the cached gain was used
        # instead of a freshly solved one. Exposed for the ROS node to log.
        self.frozen_gain_count = 0
        # Maximum consecutive cycles allowed on a frozen gain before zeroing thrust.
        # At 25 Hz, 25 cycles = 1 second — enough to ride a transient, short enough
        # to avoid running on a gain computed far from the current operating point.
        self.max_frozen_cycles = 25

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """
        Converts a ROS 2 Quaternion into standard Euler angles (Roll, Pitch, Yaw).
        For details on the transfer logic, see:
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw
    
    def update_system_dynamics_matrix_A(self, Am: np.ndarray, state: np.ndarray) -> np.ndarray:
        """
        Computes the State-Dependent System Dynamics Matrix (A Matrix).
        
        Args:
            Am (np.ndarray): 12x12 pre-allocated array representing physical dynamics.
            state (np.ndarray): 12-element 1D array representing current Sub NED/FRD states.

        Returns:
            np.ndarray: Evaluated 12x12 state-dependent dynamics matrix.
        """
        Am.fill(0.0)

        # Extracting state's data. 
        # X and Y are locked to 0 because of "Translational Invariance" — hydrodynamic 
        # drag and inertia behave exactly the same regardless of global GPS coordinates.
        x = 0
        y = 0
        z = state[2]
        roll_ = state[3]
        pitch_ = state[4]
        yaw_ = state[5]
        
        # Mapping live velocities to ensure true SDRE transit capability.
        u = state[6]
        v = state[7]
        w = state[8]
        p = state[9]
        q = state[10]
        r = state[11]

        # Precomputing of sin, cos and tan to avoid redundant CPU math
        sin_roll = math.sin(roll_)
        sin_yaw = math.sin(yaw_)
        sin_pitch = math.sin(pitch_)
        cos_roll = math.cos(roll_)
        cos_yaw = math.cos(yaw_)
        cos_pitch = math.cos(pitch_)
        tan_pitch = math.tan(pitch_)

        # Unity-aligned hydrostatic fit. The scene uses mass=23.9 kg,
        # displaced_volume=0.024 m^3, water_density=1000 kg/m^3, and a CB-CG
        # z-up offset near 0.05 m. Restoring signs below make roll/pitch stable.
        net_buoyancy_accel = 0.026395212312363908
        surge_buoyancy_coupling = 0.029301625584477216
        roll_restoring_accel = 20.366782006920415
        pitch_restoring_accel = 17.694924117348722

        surge_damping = self._sdre_damping("surge", u)
        sway_damping = self._sdre_damping("sway", v)
        heave_damping = self._sdre_damping("heave", w)
        roll_damping = self._sdre_damping("roll", p)
        pitch_damping = self._sdre_damping("pitch", q)
        yaw_damping = self._sdre_damping("yaw", r)

        # --- KINEMATIC COUPLING (Rows 0-5) ---
        # These rows represent standard rotation matrices defining how body-frame 
        # velocities (u,v,w,p,q,r) translate into world-frame velocities (x_dot, 
        # y_dot, z_dot, roll_dot, pitch_dot, yaw_dot).

        # Row 0: x_dot (World North Velocity)
        Am[0][0] = 0.0
        Am[0][1] = 0.0
        Am[0][2] = 0.0
        Am[0][3] = v * (sin_roll * sin_yaw + cos_roll * cos_yaw * sin_pitch) + w * (cos_roll * sin_yaw - 1.0 * cos_yaw * sin_pitch * sin_roll)
        Am[0][4] = w * cos_pitch * cos_roll * cos_yaw - 1.0 * u * cos_yaw * sin_pitch + v * cos_pitch * cos_yaw * sin_roll
        Am[0][5] = w * (cos_yaw * sin_roll - 1.0 * cos_roll * sin_pitch * sin_yaw) - 1.0 * v * (cos_roll * cos_yaw + sin_pitch * sin_roll * sin_yaw) - 1.0 * u * cos_pitch * sin_yaw
        Am[0][6] = cos_pitch * cos_yaw
        Am[0][7] = cos_yaw * sin_pitch * sin_roll - 1.0 * cos_roll * sin_yaw
        Am[0][8] = sin_roll * sin_yaw + cos_roll * cos_yaw * sin_pitch
        Am[0][9:12] = 0.0

        # Row 1: y_dot (World East Velocity)
        Am[1][0:3] = 0.0
        Am[1][3] = - 1.0 * w * (cos_roll * cos_yaw + sin_pitch * sin_roll * sin_yaw) - 1.0 * v * (cos_yaw * sin_roll - 1.0 * cos_roll * sin_pitch * sin_yaw)
        Am[1][4] = w * cos_pitch * cos_roll * sin_yaw - 1.0 * u * sin_pitch * sin_yaw + v * cos_pitch * sin_roll * sin_yaw
        Am[1][5] = w * (sin_roll * sin_yaw + cos_roll * cos_yaw * sin_pitch) - 1.0 * v * (cos_roll * sin_yaw - 1.0 * cos_yaw * sin_pitch * sin_roll) + u * cos_pitch * cos_yaw
        Am[1][6] = cos_pitch * sin_yaw
        Am[1][7] = cos_roll * cos_yaw + sin_pitch * sin_roll * sin_yaw
        Am[1][8] = cos_roll * sin_pitch * sin_yaw - 1.0 * cos_yaw * sin_roll
        Am[1][9:12] = 0.0

        # Row 2: z_dot (World Down Velocity)
        Am[2][0:3] = 0.0
        Am[2][3] = v * cos_pitch * cos_roll - 1.0 * w * cos_pitch * sin_roll
        Am[2][4] = - 1.0 * u * cos_pitch - 1.0 * w * cos_roll * sin_pitch - 1.0 * v * sin_pitch * sin_roll
        Am[2][5] = 0.0
        Am[2][6] = -1.0 * sin_pitch
        Am[2][7] = cos_pitch * sin_roll
        Am[2][8] = cos_pitch * cos_roll
        Am[2][9:12] = 0.0

        # Row 3: roll_dot (World Roll Rate)
        Am[3][0:3] = 0.0
        Am[3][3] = q * cos_roll * tan_pitch - 1.0 * r * tan_pitch * sin_roll
        Am[3][4] = r * cos_roll * (tan_pitch ** 2 + 1.0) + q * sin_roll * (tan_pitch ** 2 + 1.0)
        Am[3][5:9] = 0.0
        Am[3][9] = 1.0
        Am[3][10] = tan_pitch * sin_roll
        Am[3][11] = cos_roll * tan_pitch

        # Row 4: pitch_dot (World Pitch Rate)
        Am[4][0:3] = 0.0
        Am[4][3] = - 1.0 * r * cos_roll - 1.0 * q * sin_roll
        Am[4][4:10] = 0.0
        Am[4][10] = cos_roll
        Am[4][11] = -1.0 * sin_roll

        # Row 5: yaw_dot (World Yaw Rate)
        Am[5][0:3] = 0.0
        Am[5][3] = (q * cos_roll) / cos_pitch - (1.0 * r * sin_roll) / cos_pitch
        Am[5][4] = (r * cos_roll * sin_pitch) / cos_pitch ** 2 + (q * sin_pitch * sin_roll) / cos_pitch ** 2
        Am[5][5:10] = 0.0
        Am[5][10] = sin_roll / cos_pitch
        Am[5][11] = cos_roll / cos_pitch

        # --- KINETIC & HYDRODYNAMIC COUPLING (Rows 6-11) ---
        # These rows calculate accelerations based on hydrodynamic drag, Coriolis 
        # forces (cross-coupling between axes), and hydrostatic restoring forces 
        # (gravity/buoyancy vectors shifting as the Sub pitches and rolls).

        # Row 6: u_dot (Surge Acceleration)
        Am[6][0:4] = 0.0
        Am[6][4] = surge_buoyancy_coupling * cos_pitch
        Am[6][5] = 0.0
        # Damping opposes motion: sign is always negative per marine convention
        # ν_dot = M⁻¹(τ - C(ν)ν - D(ν)ν - g(η)), D(ν) > 0 → contribution is -D(ν)ν.
        Am[6][6] = -surge_damping
        Am[6][7] = 1.1101113807200520490365790644713 * r
        Am[6][8] = -1.1101113807200520490365790644713 * q
        Am[6][9] = 0.0
        Am[6][10] = -1.1101113807200520490365790644713 * w
        Am[6][11] = 1.1101113807200520490365790644713 * v

        # Row 7: v_dot (Sway Acceleration)
        Am[7][0:3] = 0.0
        Am[7][3] = -net_buoyancy_accel * cos_pitch * cos_roll
        Am[7][4] = net_buoyancy_accel * sin_pitch * sin_roll
        Am[7][5] = 0.0
        Am[7][6] = -0.90081051087988085109735042490748 * r
        Am[7][7] = -sway_damping
        Am[7][8] = p
        Am[7][9] = w
        Am[7][10] = 0.0
        Am[7][11] = -0.90081051087988085109735042490748 * u

        # Row 8: w_dot (Heave Acceleration)
        Am[8][0:3] = 0.0
        Am[8][3] = net_buoyancy_accel * cos_pitch * sin_roll
        Am[8][4] = net_buoyancy_accel * cos_roll * sin_pitch
        Am[8][5] = 0.0
        Am[8][6] = 0.90081051087988085109735042490748 * q
        Am[8][7] = -1.0 * p
        Am[8][8] = -heave_damping
        Am[8][9] = -1.0 * v
        Am[8][10] = 0.90081051087988085109735042490748 * u
        Am[8][11] = 0.0

        # Row 9: p_dot (Roll Angular Acceleration)
        Am[9][0:3] = 0.0
        Am[9][3] = -roll_restoring_accel * cos_pitch * cos_roll
        Am[9][4] = roll_restoring_accel * sin_pitch * sin_roll
        Am[9][5:9] = 0.0
        Am[9][9] = -roll_damping
        Am[9][10] = -0.43503277217070903803294482236905 * r
        Am[9][11] = -0.43503277217070903803294482236905 * q

        # Row 10: q_dot (Pitch Angular Acceleration)
        Am[10][0:4] = 0.0
        Am[10][4] = -pitch_restoring_accel * cos_pitch
        Am[10][5] = 0.0
        Am[10][6] = 5.5412526536013647028260744793525 * w
        Am[10][7] = 0.0
        Am[10][8] = 5.5412526536013647028260744793525 * u
        Am[10][9] = 0.50914915170048566293331160158391 * r
        Am[10][10] = -pitch_damping
        Am[10][11] = 0.50914915170048566293331160158391 * p

        # Row 11: r_dot (Yaw Angular Acceleration)
        Am[11][0:9] = 0.0
        Am[11][6] = -4.0213389143211517422715271307161 * v
        Am[11][7] = -4.0213389143211517422715271307161 * u
        Am[11][8] = 0.0
        Am[11][9] = -0.095203664338187202006697551579755 * q
        Am[11][10] = -0.095203664338187202006697551579755 * p
        Am[11][11] = -yaw_damping

        return Am

    def _sdre_damping(self, axis: str, velocity: float) -> float:
        """Return state-dependent damping acceleration gain for one DOF."""
        return (
            HYDRO_LINEAR[axis]
            + HYDRO_QUADRATIC[axis] * abs(float(velocity))
        ) / EFFECTIVE_MASS[axis]

    def compute_thrust_force(self, state: np.ndarray, state_error: np.ndarray, 
                             q_matrix: np.ndarray, r_matrix: np.ndarray, 
                             inv_r_matrix: np.ndarray) -> np.ndarray:
        """
        The core LQR mathematical engine.
        
        Args:
            state (np.ndarray): 12-element 1D array (Current NED/FRD states).
            state_error (np.ndarray): 12-element 1D array (Difference between target and current state).
            q_matrix (np.ndarray): 12x12 diagonal 2D array (Cost penalty for state errors).
            r_matrix (np.ndarray): 8x8 diagonal 2D array (Cost penalty for using thruster energy).
            inv_r_matrix (np.ndarray): Pre-computed 8x8 inverse of R matrix to save CPU cycles.
            
        Returns:
            np.ndarray: An 8-element 1D array representing the required raw Newton 
            effort for each physical thruster. YOU MUST CLIP THIS OUTPUT EXTERNALLY.
        """
        # 1. Update the A matrix with the current physics based on live velocities
        a_matrix = self.update_system_dynamics_matrix_A(self.system_dynamics_matrix, state)

        # Guard 1: Non-finite entries in A (e.g. cos_pitch -> 0 at pitch = ±90°
        # causes row 5 entries to blow up). The ARE solver produces undefined
        # results with inf/NaN inputs without necessarily raising an exception.
        if not np.all(np.isfinite(a_matrix)):
            return self._apply_frozen_gain(state_error)

        try:
            # 2. Solve the Continuous Algebraic Riccati Equation (ARE)
            # This is computationally heavy but solvable in real-time on a Jetson Xavier.
            # It finds the 'X' matrix that minimizes the infinite-horizon cost function.
            x_matrix = scipy.linalg.solve_continuous_are(a_matrix, Bm, q_matrix, r_matrix)

            # 3. Compute the Optimal Gain Matrix (K)
            # K = R^-1 * B^T * X
            k_gain = np.dot(inv_r_matrix, np.dot(Bm.T, x_matrix))

            # Guard 2: ARE returned without raising but produced non-finite K.
            # Happens when the Hamiltonian has near-imaginary eigenvalues and the
            # QZ decomposition is poorly conditioned.
            if not np.all(np.isfinite(k_gain)):
                return self._apply_frozen_gain(state_error)

        except scipy.linalg.LinAlgError:
            # Raised when the stable subspace of the Hamiltonian cannot be isolated
            # (system not stabilizable / not detectable at this operating point).
            return self._apply_frozen_gain(state_error)

        # Successful solve: cache the gain and reset the freeze counter.
        self._last_valid_k_gain = k_gain
        self.frozen_gain_count = 0

        # 4. Compute optimal control effort (u = -Kx)
        # Where 'x' is the state error. We negate it to drive the error to zero.
        return -np.dot(k_gain, state_error)

    def _apply_frozen_gain(self, state_error: np.ndarray) -> np.ndarray:
        """
        Apply the last known-good gain when the ARE solver fails.

        Returns zero thrust if no valid gain has ever been computed, or if the
        frozen gain has been in use for more than max_frozen_cycles consecutive
        cycles (operating point has drifted too far from where K was computed).
        """
        self.frozen_gain_count += 1
        if self._last_valid_k_gain is None or self.frozen_gain_count > self.max_frozen_cycles:
            return np.zeros(8, dtype=np.float64)
        return -np.dot(self._last_valid_k_gain, state_error)
