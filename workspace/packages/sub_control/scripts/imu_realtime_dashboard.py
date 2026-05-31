#!/usr/bin/env python3

import math
import threading

import matplotlib
matplotlib.use("TkAgg")

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


# ---------------------------------------------------------------------------
# Rotation matrices
# ---------------------------------------------------------------------------

def rx(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])

def ry(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, 0, sa], [0, 1, 0], [-sa, 0, ca]])

def rz(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]])

def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------------
# Frame conversions  (AUV convention: NED+FRD internal, ENU+FLU for ROS)
# ---------------------------------------------------------------------------

# Position vectors:  [y, x, -z]  (symmetric in both directions)
_R_NED_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)

def ned_to_enu(v):
    """Position/velocity vector: NED → ENU.  [x,y,z]_ENU = [y_NED, x_NED, -z_NED]"""
    return _R_NED_ENU @ v

def enu_to_ned(v):
    """Position/velocity vector: ENU → NED.  [x,y,z]_NED = [y_ENU, x_ENU, -z_ENU]"""
    return _R_NED_ENU @ v  # symmetric

# Body vectors:  [x, -y, -z]  (symmetric in both directions)
_R_FRD_FLU = np.diag([1.0, -1.0, -1.0])

def frd_to_flu(v):
    """Body vector: FRD → FLU.  [x,y,z]_FLU = [x_FRD, -y_FRD, -z_FRD]"""
    return _R_FRD_FLU @ v

def flu_to_frd(v):
    """Body vector: FLU → FRD.  [x,y,z]_FRD = [x_FLU, -y_FLU, -z_FLU]"""
    return _R_FRD_FLU @ v  # symmetric

def euler_enu_to_ned(roll, pitch, yaw):
    """ENU/FLU Euler angles → NED/FRD.
    roll unchanged (same physical Forward axis), pitch negated (Right = -Left),
    yaw = π/2 − yaw_ENU  (ENU zero = East, NED zero = North)."""
    yaw_ned = (math.pi / 2.0 - yaw + math.pi) % (2 * math.pi) - math.pi
    return roll, -pitch, yaw_ned

# NED→ENU is the same involution (applying twice returns to start)
euler_ned_to_enu = euler_enu_to_ned

def quaternion_to_euler(x, y, z, w):
    """Standard ZYX decomposition — result is in the frame of the input quaternion."""
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Mode table  {id: (label, frame, description)}
# frame "ned_frd" → modes 1 & 4   |   "enu_flu" → modes 2 & 3
# 3D model ALWAYS in NED/FRD; only the text panel changes convention.
# ---------------------------------------------------------------------------

MODES = {
    1: ("NED raw",  "ned_frd", "VN-100 direct"),
    2: ("ENU raw",  "enu_flu", "→ EKF input"),
    3: ("EKF ENU",  "enu_flu", "filtered"),
    4: ("NED LQR",  "ned_frd", "→ LQR"),
}

FRAME_CONV = {
    "ned_frd": "NED / FRD",
    "enu_flu": "ENU / FLU",
}

# Display transform: NED math coords → matplotlib plot coords
#   North → +X_plot  |  East → −Y_plot  |  Down → −Z_plot
# Negative Y and Z so that "Down" appears downward and "North" faces forward.
DISP_NED = np.diag([1.0, -1.0, -1.0])


# ---------------------------------------------------------------------------
# Submarine geometry
# ---------------------------------------------------------------------------

def _quad2tri(fq):
    ft = []
    for q in fq:
        ft += [[q[0], q[1], q[2]], [q[0], q[2], q[3]]]
    return np.array(ft, dtype=int)

def _cylinder(length, radius, n, cx):
    th = np.linspace(0, 2*np.pi, n, endpoint=False)
    x1, x2 = cx - length/2, cx + length/2
    r1 = np.c_[np.full(n, x1), radius*np.cos(th), radius*np.sin(th)]
    r2 = np.c_[np.full(n, x2), radius*np.cos(th), radius*np.sin(th)]
    v  = np.vstack([r1, r2, [[x1, 0, 0]], [[x2, 0, 0]]])
    side = [[i, (i+1)%n, n+(i+1)%n, n+i] for i in range(n)]
    cap1 = [[2*n,   (i+1)%n, i]       for i in range(n)]
    cap2 = [[2*n+1,  n+i,  n+(i+1)%n] for i in range(n)]
    f = np.vstack([_quad2tri(np.array(side, int)),
                   np.array(cap1, int), np.array(cap2, int)])
    return v, f

def _box(lx, ly, lz, cx):
    hx, hy, hz = lx/2, ly/2, lz/2
    v = np.array([[-hx,-hy,-hz],[hx,-hy,-hz],[hx,hy,-hz],[-hx,hy,-hz],
                  [-hx,-hy, hz],[hx,-hy, hz],[hx,hy, hz],[-hx,hy, hz]]) + cx
    f = np.array([[0,1,2],[0,2,3],[4,7,6],[4,6,5],
                  [0,4,5],[0,5,1],[1,5,6],[1,6,2],
                  [2,6,7],[2,7,3],[3,7,4],[3,4,0]], int)
    return v, f

def build_submarine():
    n = 28
    v_body, f_body = _cylinder(2.2, 0.55, n, 0.0)
    v_nose, f_nose = _cylinder(0.2, 0.55, n, 1.19)
    v_plate, f_plate = _box(3.0, 3.0, 0.08, np.array([0, 0, 0]))
    v_sl, f_sl = _cylinder(2.2, 0.3, n, 0.0)
    v_sr, f_sr = _cylinder(2.2, 0.3, n, 0.0)
    v_sl[:, 1] += 0.82;  v_sl[:, 2] += 0.58
    v_sr[:, 1] -= 0.82;  v_sr[:, 2] += 0.58
    return [
        (v_body,  f_body,  (0.45, 0.45, 0.50), 0.45),
        (v_nose,  f_nose,  (0.90, 0.30, 0.30), 0.85),
        (v_plate, f_plate, (0.72, 0.72, 0.78), 0.30),
        (v_sl,    f_sl,    (0.55, 0.55, 0.60), 0.35),
        (v_sr,    f_sr,    (0.55, 0.55, 0.60), 0.35),
    ]


# ---------------------------------------------------------------------------
# Dashboard node
# ---------------------------------------------------------------------------

class ImuRealtimeDashboard(Node):

    def __init__(self):
        super().__init__("imu_realtime_dashboard")

        self.declare_parameter("imu_topic",  "/vectornav/imu")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("active_mode", 1)

        self.imu_topic   = self.get_parameter("imu_topic").value
        self.odom_topic  = self.get_parameter("odom_topic").value
        self.active_mode = int(self.get_parameter("active_mode").value)
        if self.active_mode not in MODES:
            self.active_mode = 1

        self.lock = threading.Lock()
        self.imu_rpy_enu = (0.0, 0.0, 0.0)
        self.ekf_rpy_enu = (0.0, 0.0, 0.0)
        self.imu_count   = 0
        self.ekf_count   = 0
        self.label_mode  = "body"

        self.manual_mode  = False
        self.manual_roll  = 0.0   # NED/FRD radians
        self.manual_pitch = 0.0
        self.manual_yaw   = 0.0
        self.manual_step  = math.radians(5.0)

        self.geom = build_submarine()

        L = 1.0
        self.L_world = 2.0 * L
        self.L_body  = 1.4 * L
        self.lim     = 2.8

        self._build_figure()

        self.create_subscription(Imu, self.imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        self.get_logger().info(f"Dashboard prêt  —  IMU: {self.imu_topic}  EKF: {self.odom_topic}")

    # ------------------------------------------------------------------
    # Figure construction
    # ------------------------------------------------------------------

    def _build_figure(self):
        self.fig = plt.figure("ASUQTR IMU / EKF Dashboard", figsize=(16, 10))
        self.ax  = self.fig.add_axes([0.25, 0.03, 0.73, 0.94], projection="3d")
        self._setup_axes()
        self._create_world_axes()
        self._create_body_axes()
        self.mesh_artists = []

        # Top-left: live angles + mode info
        self.info_text = self.fig.text(
            0.01, 0.97, "", fontsize=11, family="monospace",
            verticalalignment="top",
            bbox=dict(facecolor="white", edgecolor="0.6", boxstyle="round,pad=0.5")
        )
        # Bottom-left: key guide (always visible)
        self.help_text = self.fig.text(
            0.01, 0.46, "", fontsize=10, family="monospace",
            verticalalignment="top",
            bbox=dict(facecolor="#f5f5f5", edgecolor="0.6", boxstyle="round,pad=0.5")
        )

        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    def _setup_axes(self):
        ax = self.ax
        ax.set_xlim(-self.lim, self.lim)
        ax.set_ylim(-self.lim, self.lim)
        ax.set_zlim(-self.lim, self.lim)
        ax.set_box_aspect([1, 1, 1])
        ax.view_init(elev=25, azim=135)
        ax.grid(True)
        ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
        ax.plot([0], [0], [0], 'ko', markersize=4)

    def _create_world_axes(self):
        L = self.L_world
        # NED world frame (modes 1 & 4): Red=N, Green=E, Blue=D
        # All positions via DISP_NED (North=+X_plot, East=−Y_plot, Down=−Z_plot)
        ned_tips = [
            (DISP_NED @ np.array([L, 0, 0]), (0.85, 0.15, 0.15), "N"),
            (DISP_NED @ np.array([0, L, 0]), (0.15, 0.60, 0.15), "E"),
            (DISP_NED @ np.array([0, 0, L]), (0.15, 0.35, 0.85), "D"),
        ]
        # ENU world frame (modes 2 & 3): Red=E(X_ENU), Green=N(Y_ENU), Blue=U(Z_ENU)
        # East = Y_NED → DISP_NED@[0,1,0] = [0,−1,0]
        # North = X_NED → DISP_NED@[1,0,0] = [1,0,0]
        # Up = −Z_NED → DISP_NED@[0,0,−1] = [0,0,+1]  (arrow points upward)
        enu_tips = [
            (DISP_NED @ np.array([0, L, 0]), (0.85, 0.15, 0.15), "E"),
            (DISP_NED @ np.array([L, 0, 0]), (0.15, 0.60, 0.15), "N"),
            (DISP_NED @ np.array([0, 0,-L]), (0.15, 0.35, 0.85), "U"),
        ]
        self.ned_world_lines, self.ned_world_labels = [], []
        self.enu_world_lines, self.enu_world_labels = [], []
        for tips, lines, labels, init_vis in [
            (ned_tips, self.ned_world_lines, self.ned_world_labels, True),
            (enu_tips, self.enu_world_lines, self.enu_world_labels, False),
        ]:
            for pts, color, name in tips:
                ln, = self.ax.plot([0, pts[0]], [0, pts[1]], [0, pts[2]],
                                   color=color, lw=2.5, visible=init_vis)
                tx = self.ax.text(pts[0]*1.2, pts[1]*1.2, pts[2]*1.2, name,
                                  color=color, fontsize=12, fontweight="bold",
                                  ha="center", va="center", visible=False)
                lines.append(ln)
                labels.append(tx)

    def _create_body_axes(self):
        L = self.L_body
        self.bx, = self.ax.plot([0, L], [0, 0], [0, 0], color="red",    lw=4)
        self.by, = self.ax.plot([0, 0], [0, L], [0, 0], color="green",  lw=4)
        self.bz, = self.ax.plot([0, 0], [0, 0], [0, L], color="orange", lw=4)
        self.body_label_artists = []   # recreated each frame for correct horizontal text

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    def _on_close(self, _event):
        rclpy.shutdown()

    def _on_key(self, event):
        if event.key in ("1", "2", "3", "4"):
            self.active_mode = int(event.key)
            label, frame, desc = MODES[self.active_mode]
            self.get_logger().info(f"Mode [{self.active_mode}] {label} — {frame.upper()} — {desc}")
        elif event.key == "b":
            self.label_mode = "world" if self.label_mode == "body" else "body"
        elif event.key == "m":
            self.manual_mode = not self.manual_mode
            self.get_logger().info(f"Manual mode {'ON' if self.manual_mode else 'OFF'}")
        elif self.manual_mode:
            s = self.manual_step
            if   event.key == "up":    self.manual_pitch -= s   # nose up = negative pitch NED
            elif event.key == "down":  self.manual_pitch += s
            elif event.key == "left":  self.manual_yaw   -= s
            elif event.key == "right": self.manual_yaw   += s
            elif event.key == "[":     self.manual_roll  -= s
            elif event.key == "]":     self.manual_roll  += s
            elif event.key == "r":
                self.manual_roll = self.manual_pitch = self.manual_yaw = 0.0
        elif event.key == "escape":
            plt.close(self.fig)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        if q.x == q.y == q.z == 0.0 and q.w == 0.0:
            return
        rpy = quaternion_to_euler(q.x, q.y, q.z, q.w)
        with self.lock:
            self.imu_rpy_enu = rpy
            self.imu_count  += 1

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        if q.x == q.y == q.z == 0.0 and q.w == 0.0:
            return
        rpy = quaternion_to_euler(q.x, q.y, q.z, q.w)
        with self.lock:
            self.ekf_rpy_enu = rpy
            self.ekf_count  += 1

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def update_plot(self, _frame=None):
        with self.lock:
            imu_enu   = self.imu_rpy_enu
            ekf_enu   = self.ekf_rpy_enu
            imu_count = self.imu_count
            ekf_count = self.ekf_count
            mode      = self.active_mode

        label, frame, desc = MODES[mode]

        if self.manual_mode:
            # Manual mode: use keyboard-controlled NED/FRD angles, ignore IMU/EKF
            roll_3d, pitch_3d, yaw_3d = self.manual_roll, self.manual_pitch, self.manual_yaw
            if frame == "ned_frd":
                roll, pitch, yaw = roll_3d, pitch_3d, yaw_3d
            else:
                roll, pitch, yaw = euler_ned_to_enu(roll_3d, pitch_3d, yaw_3d)
        else:
            # Live mode: angles from IMU or EKF depending on mode
            imu_ned = euler_enu_to_ned(*imu_enu)
            ekf_ned = euler_enu_to_ned(*ekf_enu)
            roll_3d, pitch_3d, yaw_3d = imu_ned if mode in (1, 2) else ekf_ned
            if frame == "ned_frd":
                roll, pitch, yaw = imu_ned if mode == 1 else ekf_ned
            else:
                roll, pitch, yaw = imu_enu if mode == 2 else ekf_enu

        # Body rotation matrix (always NED/FRD — internal control convention)
        R = rz(yaw_3d) @ ry(pitch_3d) @ rx(roll_3d)

        is_ned = (frame == "ned_frd")
        show_labels = (self.label_mode == "world")

        # World reference axes: show NED set (modes 1,4) or ENU set (modes 2,3).
        # B key toggles the text labels (N/E/D or E/N/U) on whichever set is active.
        for ln, lbl in zip(self.ned_world_lines, self.ned_world_labels):
            ln.set_visible(is_ned)
            lbl.set_visible(is_ned and show_labels)
        for ln, lbl in zip(self.enu_world_lines, self.enu_world_labels):
            ln.set_visible(not is_ned)
            lbl.set_visible((not is_ned) and show_labels)

        # Body axes depend on frame convention:
        #   NED/FRD → Forward/Right/Down (Fwd/Rgt/Dwn)
        #   ENU/FLU → Forward/Left/Up    (Fwd/Lft/Up)   Y and Z are negated vs FRD
        if is_ned:
            xb = DISP_NED @ (R[:, 0] * self.L_body)    # Forward (FRD X)
            yb = DISP_NED @ (R[:, 1] * self.L_body)    # Right   (FRD Y)
            zb = DISP_NED @ (R[:, 2] * self.L_body)    # Down    (FRD Z)
            body_lbls = [("Fwd", "red"), ("Rgt", "green"), ("Dwn", "orange")]
        else:
            xb = DISP_NED @ (R[:, 0] * self.L_body)    # Forward (FLU X = FRD X)
            yb = DISP_NED @ (-R[:, 1] * self.L_body)   # Left    (FLU Y = −FRD Y)
            zb = DISP_NED @ (-R[:, 2] * self.L_body)   # Up      (FLU Z = −FRD Z)
            body_lbls = [("Fwd", "red"), ("Lft", "green"), ("Up", "orange")]

        self.bx.set_data_3d([0, xb[0]], [0, xb[1]], [0, xb[2]])
        self.by.set_data_3d([0, yb[0]], [0, yb[1]], [0, yb[2]])
        self.bz.set_data_3d([0, zb[0]], [0, zb[1]], [0, zb[2]])

        d = math.degrees

        # Body vector labels — recreated each frame so matplotlib keeps text horizontal.
        for a in self.body_label_artists:
            a.remove()
        self.body_label_artists = []
        tip = 1.6
        for pt, (lbl, col) in zip([xb, yb, zb], body_lbls):
            t = self.ax.text(pt[0]*tip, pt[1]*tip, pt[2]*tip, lbl,
                             color=col, fontsize=11, fontweight="bold",
                             ha="center", va="center")
            self.body_label_artists.append(t)

        self.ax.set_xlabel("")
        self.ax.set_ylabel("")
        self.ax.set_zlabel("")
        self.ax.set_title(f"[{mode}] {label}  {FRAME_CONV[frame]}  —  {desc}", pad=8)

        # Rebuild submarine mesh (FRD geometry rotated into NED display space)
        for a in self.mesh_artists:
            a.remove()
        self.mesh_artists = []
        for v, f, color, alpha in self.geom:
            vr = (R @ v.T).T                 # rotate FRD vertices into NED world
            vd = (DISP_NED @ vr.T).T         # NED → matplotlib plot coords
            coll = Poly3DCollection([vd[fi] for fi in f],
                                    facecolors=[color], edgecolors="none", alpha=alpha)
            self.ax.add_collection3d(coll)
            self.mesh_artists.append(coll)

        # Info panel — big angles
        conv = "NED" if is_ned else "ENU"
        lmode = f"{conv} ON" if self.label_mode == "world" else f"{conv} OFF"
        manual_tag = "  *** MANUAL ***\n" if self.manual_mode else ""
        info = (
            f"{manual_tag}"
            f"[{mode}] {label}  {FRAME_CONV[frame]}\n"
            f"    {desc}\n"
            f"\n"
            f"  φ  roll   {wrap180(d(roll)):>+8.2f}°\n"
            f"  θ  pitch  {wrap180(d(pitch)):>+8.2f}°\n"
            f"  ψ  yaw    {wrap180(d(yaw)):>+8.2f}°\n"
            f"\n"
            f"  IMU  {imu_count:>6} msgs\n"
            f"  EKF  {ekf_count:>6} msgs"
        )
        m_state = "ON " if self.manual_mode else "OFF"
        help_str = (
            "  Mode\n"
            "  ─────────────────────────────\n"
            "  1  NED raw    (VN-100)\n"
            "  2  ENU raw    (-> EKF input)\n"
            "  3  EKF ENU    (filtered)\n"
            "  4  NED LQR    (-> LQR)\n"
            "  ─────────────────────────────\n"
            f"  B  world labels : [{lmode}]\n"
            f"  M  manual mode  : [{m_state}]\n"
            "  ESC  quit\n"
            "\n"
            "  Manual controls\n"
            "  ─────────────────────────────\n"
            "  Up / Down    pitch\n"
            "  Left / Right yaw\n"
            "  [  /  ]      roll\n"
            "  R            reset to 0°\n"
            "  (active only when M = ON)"
        )

        self.info_text.set_text(info)
        self.help_text.set_text(help_str)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ImuRealtimeDashboard()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.ani = animation.FuncAnimation(
        node.fig, node.update_plot,
        interval=50, blit=False, cache_frame_data=False
    )
    plt.show()

    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
