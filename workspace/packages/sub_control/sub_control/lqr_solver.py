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
# Surge (u_dot) affected by horizontal thrusters
  Bm[0][0:8] = 0.0
  Bm[1][0:8] = 0.0
  Bm[2][0:8] = 0.0
  Bm[3][0:8] = 0.0
  Bm[4][0:8] = 0.0
  Bm[5][0:8] = 0.0
  Bm[6][0] = -0.021117481435499889398735391488808
  Bm[6][1] = -0.021117481435499889398735391488808
  Bm[6][2:6] = 0.0
  Bm[6][6] = 0.021117481435499889398735391488808
  Bm[6][7] = 0.021117481435499889398735391488808
  Bm[7][0] = 0.01902284924040905501271856144737
  Bm[7][1] = -0.01902284924040905501271856144737
  Bm[7][2:6] = 0.0
  Bm[7][6] = 0.01902284924040905501271856144737
  Bm[7][7] = -0.01902284924040905501271856144737
  Bm[8][0:2] = 0.0
  Bm[8][2] = -0.026906434569178295633265292004767
  Bm[8][3] = -0.026906434569178295633265292004767
  Bm[8][4] = -0.026906434569178295633265292004767
  Bm[8][5] = -0.026906434569178295633265292004767
  Bm[8][6:8] = 0.0
  Bm[9][0:2] = 0.0
  Bm[9][2] = -0.47145328719723183391003460207612
  Bm[9][3] = 0.47145328719723183391003460207612
  Bm[9][4] = -0.47145328719723183391003460207612
  Bm[9][5] = 0.47145328719723183391003460207612
  Bm[9][6:8] = 0.0
  Bm[10][0:2] = 0.0
  Bm[10][2] = -0.16128655774647620680552224586649
  Bm[10][3] = -0.16128655774647620680552224586649
  Bm[10][4] = 0.16128655774647620680552224586649
  Bm[10][5] = 0.16128655774647620680552224586649
  Bm[10][6:8] = 0.0
  Bm[11][0] = 0.39463536905835478514559256667363
  Bm[11][1] = -0.39463536905835478514559256667363
  Bm[11][2:6] = 0.0
  Bm[11][6] = -0.39463536905835478514559256667363
  Bm[11][7] = 0.39463536905835478514559256667363


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
    The output of `compute_thrust_force` is strictly in physical NEWTONS.
    Proof: The state-space equation is $\dot{x} = Ax + Bu$. The B matrix converts 
    control input ($u$) into physical acceleration ($\dot{x}$). By Newton's second 
    law, $a = \frac{F}{m}$.
    If we look at the B matrix for Surge (row 6), the coefficient is ~0.0372. 
    If we mathematically apply an input of $u = 1$, then acceleration $a = 0.0372 \text{ m/s}^2$. 
    Therefore, the mass of the Sub modeled here is $m = \frac{1}{0.0372} \approx 26.88 \text{ kg}$.
    Because the mass is already baked directly into the B matrix as $\frac{1}{m}$, 
    an input of $u = 1.0$ mathematically equates to exactly 1.0 Newton of force.

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
        # +1.0 keeps current damping sign, -1.0 flips all damping diagonal terms.
        self.damping_sign = 1.0

    def set_damping_sign(self, sign: float):
        """
        Set the global sign applied to diagonal damping terms in A.

        Args:
            sign (float): Must be +1.0 or -1.0.
        """
        if sign not in (-1.0, 1.0):
            raise ValueError('damping_sign must be either -1.0 or 1.0')
        self.damping_sign = float(sign)

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

        # Sub params
        vehicule_radius = 0.26

        # Hydrostatic check: If the submarine breaches the surface, buoyancy decreases.
        # This alters the restoring forces in the Z, Roll, and Pitch dynamics.
        # radius = max(vehicule_radius - abs(z), 0) if z < 0.8 else vehicule_radius
        radius = vehicule_radius

        # --- KINEMATIC COUPLING (Rows 0-5) ---
        # These rows represent standard rotation matrices defining how body-frame 
        # velocities (u,v,w,p,q,r) translate into world-frame velocities (x_dot, 
        # y_dot, z_dot, roll_dot, pitch_dot, yaw_dot).

        # Row 0: x_dot (World North Velocity)
        Am[0][0:3] = 0.0
        Am[0][3] = v*(sin_roll*sin_yaw + cos_roll*cos_yaw*sin_pitch) + w*(cos_roll*sin_yaw - 1.0*cos_yaw*sin_pitch*sin_roll)
        Am[0][4] = w*cos_pitch*cos_roll*cos_yaw - 1.0*u*cos_yaw*sin_pitch + v*cos_pitch*cos_yaw*sin_roll
        Am[0][5] = w*(cos_yaw*sin_roll - 1.0*cos_roll*sin_pitch*sin_yaw) - 1.0*v*(cos_roll*cos_yaw + sin_pitch*sin_roll*sin_yaw) - 1.0*u*cos_pitch*sin_yaw
        Am[0][6] = cos_pitch*cos_yaw
        Am[0][7] = cos_yaw*sin_pitch*sin_roll - 1.0*cos_roll*sin_yaw
        Am[0][8] = sin_roll*sin_yaw + cos_roll*cos_yaw*sin_pitch
        Am[0][9:12] = 0.0
        Am[1][0:3] = 0.0
        Am[1][3] = - 1.0*w*(cos_roll*cos_yaw + sin_pitch*sin_roll*sin_yaw) - 1.0*v*(cos_yaw*sin_roll - 1.0*cos_roll*sin_pitch*sin_yaw)
        Am[1][4] = w*cos_pitch*cos_roll*sin_yaw - 1.0*u*sin_pitch*sin_yaw + v*cos_pitch*sin_roll*sin_yaw
        Am[1][5] = w*(sin_roll*sin_yaw + cos_roll*cos_yaw*sin_pitch) - 1.0*v*(cos_roll*sin_yaw - 1.0*cos_yaw*sin_pitch*sin_roll) + u*cos_pitch*cos_yaw
        Am[1][6] = cos_pitch*sin_yaw
        Am[1][7] = cos_roll*cos_yaw + sin_pitch*sin_roll*sin_yaw
        Am[1][8] = cos_roll*sin_pitch*sin_yaw - 1.0*cos_yaw*sin_roll
        Am[1][9:12] = 0.0
        Am[2][0:3] = 0.0
        Am[2][3] = v*cos_pitch*cos_roll - 1.0*w*cos_pitch*sin_roll
        Am[2][4] = - 1.0*u*cos_pitch - 1.0*w*cos_roll*sin_pitch - 1.0*v*sin_pitch*sin_roll
        Am[2][5] = 0.0
        Am[2][6] = -1.0*sin_pitch
        Am[2][7] = cos_pitch*sin_roll
        Am[2][8] = cos_pitch*cos_roll
        Am[2][9:12] = 0.0
        Am[3][0:3] = 0.0
        Am[3][3] = q*cos_roll*tan_pitch - 1.0*r*tan_pitch*sin_roll
        Am[3][4] = r*cos_roll*(tan_pitch**2 + 1.0) + q*sin_roll*(tan_pitch**2 + 1.0)
        Am[3][5:9] = 0.0
        Am[3][9] = 1.0
        Am[3][10] = tan_pitch*sin_roll
        Am[3][11] = cos_roll*tan_pitch
        Am[4][0:3] = 0.0
        Am[4][3] = - 1.0*r*cos_roll - 1.0*q*sin_roll
        Am[4][4:10] = 0.0
        Am[4][10] = cos_roll
        Am[4][11] = -1.0*sin_roll
        Am[5][0:3] = 0.0
        Am[5][3] = (q*cos_roll)/cos_pitch - (1.0*r*sin_roll)/cos_pitch
        Am[5][4] = (r*cos_roll*sin_pitch)/cos_pitch**2 + (q*sin_pitch*sin_roll)/cos_pitch**2
        Am[5][5:10] = 0.0
        Am[5][10] = sin_roll/cos_pitch
        Am[5][11] = cos_roll/cos_pitch
        Am[6][0:4] = 0.0
        Am[6][4] = 0.61533413727402152970770551635211*cos_pitch
        Am[6][5] = 0.0
        Am[6][6] = 0.66339358229484093853735932809963
        Am[6][7] = 1.1101113807200520490365790644713*r
        Am[6][8] = -1.1101113807200520490365790644713*q
        Am[6][9] = 0.0
        Am[6][10] = -1.1101113807200520490365790644713*w
        Am[6][11] = 1.1101113807200520490365790644713*v
        Am[7][0:3] = 0.0
        Am[7][3] = -0.5542994585596420683408982805902*cos_pitch*cos_roll
        Am[7][4] = 0.5542994585596420683408982805902*sin_pitch*sin_roll
        Am[7][5] = 0.0
        Am[7][6] = -0.90081051087988085109735042490748*r
        Am[7][7] = 0.7501513957886908822554363410929
        Am[7][8] = p
        Am[7][9] = w
        Am[7][10] = 0.0
        Am[7][11] = -0.90081051087988085109735042490748*u
        Am[8][0:3] = 0.0
        Am[8][3] = 0.5542994585596420683408982805902*cos_pitch*sin_roll
        Am[8][4] = 0.5542994585596420683408982805902*cos_roll*sin_pitch
        Am[8][5] = 0.0
        Am[8][6] = 0.90081051087988085109735042490748*q
        Am[8][7] = -1.0*p
        Am[8][8] = 1.1338371527451733779857994050809
        Am[8][9] = -1.0*v
        Am[8][10] = 0.90081051087988085109735042490748*u
        Am[8][11] = 0.0
        Am[9][0:3] = 0.0
        Am[9][3] = 22.064013840830449826989619377163*cos_pitch*cos_roll
        Am[9][4] = -22.064013840830449826989619377163*sin_pitch*sin_roll
        Am[9][5:9] = 0.0
        Am[9][9] = 2.802768166089965397923875432526
        Am[9][10] = -0.43503277217070903803294482236905*r
        Am[9][11] = -0.43503277217070903803294482236905*q
        Am[10][0:4] = 0.0
        Am[10][4] = 19.169501127127782529271437106573*cos_pitch
        Am[10][5] = 0.0
        Am[10][6] = 5.5412526536013647028260744793525*w
        Am[10][7] = 0.0
        Am[10][8] = 5.5412526536013647028260744793525*u
        Am[10][9] = 0.50914915170048566293331160158391*r
        Am[10][10] = 2.435081300552576468079646209727
        Am[10][11] = 0.50914915170048566293331160158391*p
        Am[11][0:6] = 0.0
        Am[11][6] = -4.0213389143211517422715271307161*v
        Am[11][7] = -4.0213389143211517422715271307161*u
        Am[11][8] = 0.0
        Am[11][9] = -0.095203664338187202006697551579755*q
        Am[11][10] = -0.095203664338187202006697551579755*p
        Am[11][11] = 1.7671612910636088319231574039434


        return Am

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
        
        # 2. Solve the Continuous Algebraic Riccati Equation (ARE)
        # This is computationally heavy but solvable in real-time on a Jetson Xavier.
        # It finds the 'X' matrix that minimizes the infinite-horizon cost function.
        x_matrix = scipy.linalg.solve_continuous_are(a_matrix, Bm, q_matrix, r_matrix)
        
        # 3. Compute the Optimal Gain Matrix (K)
        # K = R^-1 * B^T * X
        k_gain = np.dot(inv_r_matrix, np.dot(Bm.T, x_matrix))
        
        # 4. Compute optimal control effort (u = -Kx)
        # Where 'x' is the state error. We negate it to drive the error to zero.
        return -np.dot(k_gain, state_error)