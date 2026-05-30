#!/usr/bin/env python3

import math
import threading

import matplotlib
matplotlib.use("TkAgg")

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def rx(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([
        [1, 0, 0],
        [0, ca, -sa],
        [0, sa, ca]
    ])


def ry(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([
        [ca, 0, sa],
        [0, 1, 0],
        [-sa, 0, ca]
    ])


def rz(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([
        [ca, -sa, 0],
        [sa,  ca, 0],
        [0,   0,  1]
    ])


def wrap_to_180(x):
    return (x + 180.0) % 360.0 - 180.0


def rotate_vertices(v, r):
    return (r @ v.T).T


def apply_display_transform(v, d):
    return (d @ v.T).T


def quad2tri(fq):
    ft = []
    for q in fq:
        ft.append([q[0], q[1], q[2]])
        ft.append([q[0], q[2], q[3]])
    return np.array(ft, dtype=int)


def make_cylinder_x(length, radius, n, cx):
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)

    x1 = cx - length / 2
    x2 = cx + length / 2

    ring1 = np.column_stack([
        np.full(n, x1),
        radius * np.cos(th),
        radius * np.sin(th)
    ])

    ring2 = np.column_stack([
        np.full(n, x2),
        radius * np.cos(th),
        radius * np.sin(th)
    ])

    c1 = np.array([[x1, 0, 0]])
    c2 = np.array([[x2, 0, 0]])

    v = np.vstack([ring1, ring2, c1, c2])

    f_side = []
    for i in range(n):
        j = (i + 1) % n
        f_side.append([i, j, n + j, n + i])

    idx_c1 = 2 * n
    idx_c2 = 2 * n + 1

    f_cap1 = []
    f_cap2 = []
    for i in range(n):
        j = (i + 1) % n
        f_cap1.append([idx_c1, j, i])
        f_cap2.append([idx_c2, n + i, n + j])

    f = np.vstack([
        quad2tri(np.array(f_side, dtype=int)),
        np.array(f_cap1, dtype=int),
        np.array(f_cap2, dtype=int)
    ])

    return v, f


def make_box_vertices(lx, ly, lz, c):
    hx, hy, hz = lx / 2, ly / 2, lz / 2
    v = np.array([
        [-hx, -hy, -hz],
        [ hx, -hy, -hz],
        [ hx,  hy, -hz],
        [-hx,  hy, -hz],
        [-hx, -hy,  hz],
        [ hx, -hy,  hz],
        [ hx,  hy,  hz],
        [-hx,  hy,  hz]
    ])
    return v + np.array(c)


def make_box_faces():
    return np.array([
        [0, 1, 2], [0, 2, 3],
        [4, 7, 6], [4, 6, 5],
        [0, 4, 5], [0, 5, 1],
        [1, 5, 6], [1, 6, 2],
        [2, 6, 7], [2, 7, 3],
        [3, 7, 4], [3, 4, 0]
    ], dtype=int)


def build_simple_submarine(sub):
    v_body, f_body = make_cylinder_x(sub["body_len"], sub["body_rad"], sub["n_circ"], 0.0)

    nose_center_x = sub["body_len"] / 2 + sub["nose_len"] / 2 - 0.02
    v_nose, f_nose = make_cylinder_x(sub["nose_len"], sub["nose_rad"], sub["n_circ"], nose_center_x)

    v_plate = make_box_vertices(
        sub["plate_len"], sub["plate_wid"], sub["plate_thk"], [sub["plate_x"], 0, 0]
    )
    f_plate = make_box_faces()

    side_cx = 0.0
    v_side_l, f_side_l = make_cylinder_x(sub["side_len"], sub["side_rad"], sub["n_circ"], side_cx)
    v_side_r, f_side_r = make_cylinder_x(sub["side_len"], sub["side_rad"], sub["n_circ"], side_cx)

    v_side_l[:, 1] += sub["side_y_off"]
    v_side_r[:, 1] -= sub["side_y_off"]
    v_side_l[:, 2] += sub["side_z_off"]
    v_side_r[:, 2] += sub["side_z_off"]

    return {
        "V_body": v_body,   "F_body": f_body,
        "V_nose": v_nose,   "F_nose": f_nose,
        "V_plate": v_plate, "F_plate": f_plate,
        "V_sideL": v_side_l, "F_sideL": f_side_l,
        "V_sideR": v_side_r, "F_sideR": f_side_r
    }


def faces_to_polys(vertices, faces):
    return [vertices[f] for f in faces]


def quaternion_to_euler_xyz(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


MODES = {
    1: "NED raw  (VN-100 in)",
    2: "ENU      (-> EKF in)",
    3: "ENU EKF  (filtered) ",
    4: "NED LQR  (-> LQR in)",
}


def enu_to_ned_rpy(roll_enu, pitch_enu, yaw_enu):
    """Convert ENU/FLU roll/pitch/yaw to NED/FRD — same logic as control_node.py."""
    roll_ned = roll_enu
    pitch_ned = -pitch_enu
    yaw_ned = math.pi / 2.0 - yaw_enu
    yaw_ned = (yaw_ned + math.pi) % (2 * math.pi) - math.pi
    return roll_ned, pitch_ned, yaw_ned


class ImuRealtimeDashboard(Node):
    def __init__(self):
        super().__init__("imu_realtime_dashboard")

        self.declare_parameter("imu_topic", "/vectornav/imu")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("active_mode", 1)  # 1-4

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.active_mode = int(self.get_parameter("active_mode").value)

        if self.active_mode not in MODES:
            self.active_mode = 1

        self.lock = threading.Lock()

        # ENU RPY from vectornav/imu
        self.imu_roll_enu = 0.0
        self.imu_pitch_enu = 0.0
        self.imu_yaw_enu = 0.0

        # ENU RPY from odometry/filtered
        self.ekf_roll_enu = 0.0
        self.ekf_pitch_enu = 0.0
        self.ekf_yaw_enu = 0.0

        self.imu_msg_count = 0
        self.ekf_msg_count = 0

        self.L = 1.0
        self.L_ned = 2 * self.L
        self.L_body = 1.4 * self.L
        self.view_lim = 2.8

        self.submarine = {
            "body_len": 2.2,
            "body_rad": 0.55,
            "nose_len": 0.2,
            "nose_rad": 0.55,
            "plate_len": 3.0,
            "plate_wid": 3.0,
            "plate_thk": 0.08,
            "plate_x": 0.0,
            "side_len": 2.2,
            "side_rad": 0.3,
            "side_y_off": 0.82,
            "side_z_off": 0.58,
            "n_circ": 28
        }

        # Affichage type NED visuel
        self.ddisp = np.diag([1, -1, -1])
        self.geom = build_simple_submarine(self.submarine)

        self.fig = plt.figure("ASUQTR IMU / EKF Dashboard", figsize=(12, 7))
        self.ax = self.fig.add_axes([0.35, 0.08, 0.62, 0.82], projection="3d")

        self.setup_axes()
        self.create_static_elements()
        self.create_dynamic_elements()

        self.fig.canvas.mpl_connect("close_event", self.on_close)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f"Dashboard pret.")
        self.get_logger().info(f"IMU topic  : {self.imu_topic}")
        self.get_logger().info(f"Odom topic : {self.odom_topic}")
        self.get_logger().info(f"Mode actif : [{self.active_mode}] {MODES[self.active_mode]}")

    def setup_axes(self):
        ax = self.ax
        lim = self.view_lim

        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-lim, lim)
        ax.set_box_aspect([1, 1, 1])

        ax.view_init(elev=25, azim=135)

        ax.set_xlabel("X_N = North")
        ax.set_ylabel("Y_N = East")
        ax.set_zlabel("Z_N = Down")
        ax.set_title("Orientation temps reel")

        ax.grid(True)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False

        ax.scatter([0], [0], [0], color="k", s=30)

    def create_static_elements(self):
        pnx = self.ddisp @ np.array([self.L_ned, 0, 0])
        pny = self.ddisp @ np.array([0, self.L_ned, 0])
        pnz = self.ddisp @ np.array([0, 0, self.L_ned])

        self.hN_x, = self.ax.plot([0, pnx[0]], [0, pnx[1]], [0, pnx[2]], color=(0.85, 0.15, 0.15), lw=3)
        self.hN_y, = self.ax.plot([0, pny[0]], [0, pny[1]], [0, pny[2]], color=(0.15, 0.60, 0.15), lw=3)
        self.hN_z, = self.ax.plot([0, pnz[0]], [0, pnz[1]], [0, pnz[2]], color=(0.15, 0.35, 0.85), lw=3)

        self.tN_x = self.ax.text(*(pnx * 1.06), "N / x_N", color=(0.85, 0.15, 0.15), fontsize=11, fontweight="bold")
        self.tN_y = self.ax.text(*(pny * 1.06), "E / y_N", color=(0.15, 0.60, 0.15), fontsize=11, fontweight="bold")
        self.tN_z = self.ax.text(*(pnz * 1.06), "D / z_N", color=(0.15, 0.35, 0.85), fontsize=11, fontweight="bold")

        self.info_text = self.fig.text(
            0.02, 0.22, "", fontsize=10, family="monospace",
            bbox=dict(facecolor="white", edgecolor="0.7")
        )

        self.help_text = self.fig.text(
            0.02, 0.06, "", fontsize=10, family="monospace",
            bbox=dict(facecolor="white", edgecolor="0.7")
        )

    def create_dynamic_elements(self):
        self.hB_x, = self.ax.plot([0, self.L_body], [0, 0], [0, 0], color="red", lw=4)
        self.hB_y, = self.ax.plot([0, 0], [0, self.L_body], [0, 0], color="green", lw=4)
        self.hB_z, = self.ax.plot([0, 0], [0, 0], [0, self.L_body], color="orange", lw=4)

        self.tB_x = self.ax.text(self.L_body * 1.06, 0, 0, "x_b forward", color="red", fontsize=11, fontweight="bold")
        self.tB_y = self.ax.text(0, self.L_body * 1.06, 0, "y_b right", color="green", fontsize=11, fontweight="bold")
        self.tB_z = self.ax.text(0, 0, self.L_body * 1.06, "z_b down", color="orange", fontsize=11, fontweight="bold")

        # Create mesh collections once — updated in-place each frame via set_verts()
        mesh_config = [
            ("V_body",  "F_body",  (0.45, 0.45, 0.50), 0.45),
            ("V_nose",  "F_nose",  (0.90, 0.30, 0.30), 0.85),
            ("V_plate", "F_plate", (0.72, 0.72, 0.78), 0.30),
            ("V_sideL", "F_sideL", (0.55, 0.55, 0.60), 0.35),
            ("V_sideR", "F_sideR", (0.55, 0.55, 0.60), 0.35),
        ]
        self.mesh_collections = []
        for key_v, key_f, color, alpha in mesh_config:
            polys = faces_to_polys(self.geom[key_v], self.geom[key_f])
            coll = Poly3DCollection(polys, facecolors=[color], edgecolors="none", alpha=alpha)
            self.ax.add_collection3d(coll)
            self.mesh_collections.append((coll, key_v, key_f))

    def on_close(self, event):
        self.get_logger().info("Fermeture de la fenetre.")
        rclpy.shutdown()

    def on_key(self, event):
        if event.key in ("1", "2", "3", "4"):
            self.active_mode = int(event.key)
            self.get_logger().info(f"Mode -> [{self.active_mode}] {MODES[self.active_mode]}")
        elif event.key == "escape":
            plt.close(self.fig)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return
        roll, pitch, yaw = quaternion_to_euler_xyz(q.x, q.y, q.z, q.w)
        with self.lock:
            self.imu_roll_enu = roll
            self.imu_pitch_enu = pitch
            self.imu_yaw_enu = yaw
            self.imu_msg_count += 1

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return
        roll, pitch, yaw = quaternion_to_euler_xyz(q.x, q.y, q.z, q.w)
        with self.lock:
            self.ekf_roll_enu = roll
            self.ekf_pitch_enu = pitch
            self.ekf_yaw_enu = yaw
            self.ekf_msg_count += 1

    def set_axis_line(self, line, p1):
        line.set_data_3d([0, p1[0]], [0, p1[1]], [0, p1[2]])

    def update_plot(self, frame=None):
        with self.lock:
            imu_r = self.imu_roll_enu
            imu_p = self.imu_pitch_enu
            imu_y = self.imu_yaw_enu
            ekf_r = self.ekf_roll_enu
            ekf_p = self.ekf_pitch_enu
            ekf_y = self.ekf_yaw_enu
            imu_count = self.imu_msg_count
            ekf_count = self.ekf_msg_count
            active_mode = self.active_mode

        # Compute all 4 stages
        ned1_r, ned1_p, ned1_y = enu_to_ned_rpy(imu_r, imu_p, imu_y)   # NED raw  (IMU)
        enu2_r, enu2_p, enu2_y = imu_r, imu_p, imu_y                    # ENU IMU  (-> EKF)
        enu3_r, enu3_p, enu3_y = ekf_r, ekf_p, ekf_y                    # ENU EKF  (filtered)
        ned4_r, ned4_p, ned4_y = enu_to_ned_rpy(ekf_r, ekf_p, ekf_y)   # NED LQR  (-> LQR)

        # Select angles for 3D model — always in NED for correct display
        if active_mode == 1:
            roll_3d, pitch_3d, yaw_3d = ned1_r, ned1_p, ned1_y
        elif active_mode == 2:
            roll_3d, pitch_3d, yaw_3d = enu_to_ned_rpy(enu2_r, enu2_p, enu2_y)
        elif active_mode == 3:
            roll_3d, pitch_3d, yaw_3d = enu_to_ned_rpy(enu3_r, enu3_p, enu3_y)
        else:
            roll_3d, pitch_3d, yaw_3d = ned4_r, ned4_p, ned4_y

        r = rz(yaw_3d) @ ry(pitch_3d) @ rx(roll_3d)

        xb = self.ddisp @ (r[:, 0] * self.L_body)
        yb = self.ddisp @ (r[:, 1] * self.L_body)
        zb = self.ddisp @ (r[:, 2] * self.L_body)

        self.set_axis_line(self.hB_x, xb)
        self.set_axis_line(self.hB_y, yb)
        self.set_axis_line(self.hB_z, zb)

        px = xb * 1.06
        py = yb * 1.06
        pz = zb * 1.06

        self.tB_x.set_position((px[0], px[1]))
        self.tB_x.set_3d_properties(px[2], zdir='z')
        self.tB_y.set_position((py[0], py[1]))
        self.tB_y.set_3d_properties(py[2], zdir='z')
        self.tB_z.set_position((pz[0], pz[1]))
        self.tB_z.set_3d_properties(pz[2], zdir='z')

        for coll, key_v, key_f in self.mesh_collections:
            v_rot = rotate_vertices(self.geom[key_v], r)
            v_plot = apply_display_transform(v_rot, self.ddisp)
            coll.set_verts(faces_to_polys(v_plot, self.geom[key_f]))

        def fmt(r, p, y):
            return (f"  R:{wrap_to_180(math.degrees(r)):+7.2f}"
                    f"  P:{wrap_to_180(math.degrees(p)):+7.2f}"
                    f"  Y:{wrap_to_180(math.degrees(y)):+7.2f} deg")

        active_label = f"[{active_mode}] {MODES[active_mode].strip()}"
        info_str = (
            f"IMU: {imu_count} msgs   EKF: {ekf_count} msgs\n"
            f"modele 3D : {active_label}\n\n"
            f"[1] NED raw  (VN-100)\n{fmt(ned1_r, ned1_p, ned1_y)}\n\n"
            f"[2] ENU      (-> EKF)\n{fmt(enu2_r, enu2_p, enu2_y)}\n\n"
            f"[3] ENU EKF  (filtered)\n{fmt(enu3_r, enu3_p, enu3_y)}\n\n"
            f"[4] NED LQR  (-> LQR)\n{fmt(ned4_r, ned4_p, ned4_y)}"
        )

        help_str = (
            "Clavier\n"
            "1 : NED raw  (VN-100)\n"
            "2 : ENU -> EKF\n"
            "3 : EKF ENU\n"
            "4 : NED -> LQR\n"
            "ESC : quitter"
        )

        self.info_text.set_text(info_str)
        self.help_text.set_text(help_str)

        self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)
    node = ImuRealtimeDashboard()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Canvas timer fires update_plot() in the Tk main thread — safe for matplotlib
    timer = node.fig.canvas.new_timer(interval=50)
    timer.add_callback(node.update_plot)
    timer.start()

    plt.show()  # blocks and runs the Tk event loop

    if rclpy.ok():
        rclpy.shutdown()

    spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
