#!/usr/bin/env python3
"""
Lidar Obstacle Tracker (ROS 2)
─────────────────────────────
• Toma un PointCloud2 2-D (LiDAR, 2-D scan densificado) y devuelve
  un PoseArray "/obstacle_navigation_nodes" con la posición, radio y
  velocidad filtrada de cada obstáculo.

• Diferencias con la versión simple:
  ───────────────────────────────────
  1. Clustering DBSCAN más estricto (min_samples = 5, eps paramétrico).
  2. Seguimiento cada obstáculo = filtro de Kalman CV (pos, vel).
  3. Asociación frame-a-frame: Hungarian (scipy.optimize.linear_sum_assignment).
  4. Publicación a 15 Hz con IDs estables y salida depurada.

Dependencias extra: numpy, scipy, filterpy, scikit-learn.
"""

import math, time, struct
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node      import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseArray
from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment
from filterpy.kalman import KalmanFilter

# ───────── PARAMETERS ─────────────────────────────────────────────
EPS              = 0.35     # cluster distance [m]
MIN_PTS          = 5        # min points per cluster
MAX_ASSOC_DIST   = 0.7      # max centroid distance for matching [m]
PUBLISH_HZ       = 15.0
Q_VAR_POS        = 0.05     # process noise pos [m²]
Q_VAR_VEL        = 0.20     # process noise vel [(m/s)²]
R_VAR            = 0.02     # meas noise [m²]
RADIUS_PAD       = 0.10     # inflate radius [m]

# ───────── UTILITIES ──────────────────────────────────────────────

def pc2_to_xy(msg: PointCloud2) -> np.ndarray:
    step = msg.point_step
    off_x = msg.fields[0].offset
    off_y = msg.fields[1].offset
    buf   = msg.data
    pts   = np.empty((msg.width * msg.height, 2), np.float32)
    for i in range(msg.width * msg.height):
        b = i * step
        pts[i, 0] = struct.unpack_from('f', buf, b + off_x)[0]
        pts[i, 1] = struct.unpack_from('f', buf, b + off_y)[0]
    return pts

@dataclass
class Track:
    kf: KalmanFilter
    radius: float
    last_seen: float

    def predict(self):
        self.kf.predict()

    def update(self, z):
        self.kf.update(z)

    @property
    def pos(self):
        return self.kf.x[:2].copy()

    @property
    def vel(self):
        return self.kf.x[2:].copy()

# ───────── NODE ───────────────────────────────────────────────────
class LidarObstacleTracker(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_tracker')
        self.create_subscription(PointCloud2, '/scan_cloud_filtered', self.cb_cloud, 10)
        self.pub = self.create_publisher(PoseArray, '/obstacle_navigation_nodes_lidar', 10)

        self.dt          = 1.0 / PUBLISH_HZ
        self.tracks: dict[int, Track] = {}
        self.next_id     = 1
        self.last_pub    = self.get_clock().now().nanoseconds / 1e9

    # ───── per scan callback ─────
    def cb_cloud(self, msg: PointCloud2):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_pub < 1.0 / PUBLISH_HZ:
            return
        self.last_pub = now

        pts = pc2_to_xy(msg)
        if pts.size == 0:
            return

        labels = DBSCAN(eps=EPS, min_samples=MIN_PTS).fit_predict(pts)
        uniq   = set(labels) - {-1}
        meas   = []                            # [id_tmp, (x,y,r)]
        for lbl in uniq:
            cl  = pts[labels == lbl]
            c   = cl.mean(axis=0)
            r   = np.ptp(cl, axis=0).max() / 2 + RADIUS_PAD  # actualizado para NumPy 2.0
            meas.append((None, np.array([c[0], c[1], r], np.float32)))

        # ───── predict existing tracks ─────
        for tr in self.tracks.values():
            tr.predict()

        # ───── association cost matrix (centroid distance) ─────
        T  = len(self.tracks)
        M  = len(meas)
        if T and M:
            cost = np.full((T, M), np.inf)
            track_ids = list(self.tracks.keys())
            for ti, tid in enumerate(track_ids):
                tp = self.tracks[tid].pos
                for mi, (_, z) in enumerate(meas):
                    cost[ti, mi] = np.linalg.norm(tp - z[:2])
            ti, mi = linear_sum_assignment(cost)
            for t_idx, m_idx in zip(ti, mi):
                if cost[t_idx, m_idx] < MAX_ASSOC_DIST:
                    tid       = track_ids[t_idx]
                    meas[m_idx] = (tid, meas[m_idx][1])

        # ───── prune old tracks ─────
        for tid in list(self.tracks.keys()):
            self.tracks[tid].last_seen += 1
            if self.tracks[tid].last_seen > 30:   # >2 s desaparecido
                del self.tracks[tid]

        # ───── create / update tracks ─────
        for tid_tmp, z in meas:
            x, y, r = z
            if tid_tmp is None:
                # nuevo track
                kf = KalmanFilter(dim_x=4, dim_z=2)
                kf.F = np.eye(4); kf.F[0,2] = kf.F[1,3] = self.dt
                kf.H = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
                kf.R *= R_VAR
                kf.Q = np.diag([Q_VAR_POS, Q_VAR_POS, Q_VAR_VEL, Q_VAR_VEL])
                kf.P *= 1.0
                kf.x = np.array([x, y, 0, 0], np.float32)
                track = Track(kf=kf, radius=r, last_seen=0)
                tid = self.next_id; self.next_id += 1
                self.tracks[tid] = track
            else:
                # track existente
                tid = tid_tmp
                track = self.tracks[tid]
                track.radius = r
                track.last_seen = 0
            track.update(z[:2])

        # ───── publicar PoseArray ─────
        msg_out = PoseArray()
        msg_out.header.frame_id = msg.header.frame_id
        msg_out.header.stamp    = msg.header.stamp
        for tid, tr in self.tracks.items():
            pose = Pose()
            pose.position.x = float(tr.pos[0])
            pose.position.y = float(tr.pos[1])
            pose.position.z = float(tid)            # ID codificado opcional
            pose.orientation.w = float(tr.radius)
            pose.orientation.x = float(tr.vel[0])
            pose.orientation.y = float(tr.vel[1])
            msg_out.poses.append(pose)
        self.pub.publish(msg_out)

# ───────── MAIN ─────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
