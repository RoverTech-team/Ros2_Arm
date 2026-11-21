#!/usr/bin/env python3
"""Misura di quanto il RENDER Gazebo e il TF ottico non coincidono.

Perche': mirini, view search e misure di profondita' proiettano punti con
TF + camera_info. Se il sensore Gazebo e' posato diversamente dal frame
gemini_camera_optical_frame, la leva vera finisce in un punto dell'immagine
e il mirino in un altro: ogni validazione di visibilita' e' falsa.

Come: i due ArUco del pannello hanno posa nota nell'URDF (45 mm,
DICT_5X5_250). Il detector li trova a pixel; qui si proiettano gli stessi
4 angoli con TF + camera_info e si misura l'errore, angolo per angolo.
L'errore viene poi espresso in pixel e in millimetri alla profondita' del tag,
e si verifica se e' spiegabile con una TRASLAZIONE della camera (offset quasi
costante) o con una ROTAZIONE (offset proporzionale alla distanza dal centro
ottico). Da li' esce la correzione della <pose> del sensore.

Non si risolve la posa completa con solvePnP: 8 punti su una striscia
0.23 x 0.045 m sono quasi complanari e il tilt risulta mal condizionato
(provato: soluzioni a 1.4 m di distanza con 0.7 px di residuo).

Uso (demo Gazebo attiva, pannello con entrambi i tag visibili):
  ros2 run rover_arm_panel_description calibrate_render_tf.py

Opzioni:
  --samples N   frame su cui mediare (default 5)
  --out-json P  report (default /tmp/render_tf_calib.json)
  --apply       stampa la <pose> corretta, pronta da incollare

Nota: il quadrato del marker non dice nulla sul roll (un giro di 90 gradi o
uno specchio sono ambigui). Il roll resta non osservabile da qui; direzione
e posizione della camera si'.
"""
import argparse
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

# Pannello: panel_base_frame -> panel_base (yaw 90) -> aruco_tag_i
PANEL_BASE_XYZ = (1.30, 0.0, 0.0)
PANEL_BASE_YAW = 1.571
TAGS = {
    'aruco_tag_0': (0.050, 0.003, 0.405),
    'aruco_tag_1': (0.280, 0.001, 0.405),
}
TAG_IDS = {'aruco_tag_0': 0, 'aruco_tag_1': 1}
MARKER_SIZE = 0.045


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--samples', type=int, default=5)
    p.add_argument('--out-json', default='/tmp/render_tf_calib.json')
    p.add_argument('--apply', action='store_true')
    return p.parse_args()


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def euler_from_R(R):
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tag_pose(tag_xyz):
    """Centro del tag e base locale, in panel_base_frame.

    Il link del tag ha la stessa terna di panel_base (joint rpy=0); la piastra
    e' sottile lungo y, quindi il quadrato sta nel piano locale (x,z) con
    normale +-y, cioe' sulla faccia del pannello.
    """
    R = rot_z(PANEL_BASE_YAW)
    t = np.array(PANEL_BASE_XYZ) + R @ np.asarray(tag_xyz, dtype=float)
    return t, R


class Calib(Node):
    def __init__(self):
        super().__init__('calibrate_render_tf')
        self.latest = None
        self.caminfo = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, '/camera/rgbd/camera_info',
                                 self.on_info, 10)
        self.create_subscription(Image, '/camera/rgbd/image', self.on_image,
                                 qos_profile_sensor_data)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, cv2.aruco.DetectorParameters())
        else:  # OpenCV 4.5 e precedenti
            self.detector = None

    def on_info(self, msg):
        self.caminfo = msg

    def on_image(self, msg):
        self.latest = msg

    def grab(self, timeout=20.0):
        t0 = time.time()
        while (self.latest is None or self.caminfo is None) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest is None:
            raise RuntimeError('nessun frame RGB da /camera/rgbd/image')
        if self.caminfo is None:
            raise RuntimeError('nessun /camera/rgbd/camera_info')
        msg = self.latest
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = buf.reshape(msg.height, msg.width, 3)
        if getattr(msg, 'encoding', 'rgb8') == 'bgr8':
            return np.ascontiguousarray(img)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def detect(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        found = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                found[int(i)] = np.asarray(c, dtype=float).reshape(4, 2)
        return found

    def tf_optical(self):
        if not self.tf_buffer.can_transform('base_link', 'gemini_camera_optical_frame',
                                            rclpy.time.Time()):
            raise RuntimeError('TF base_link -> gemini_camera_optical_frame assente')
        tr = self.tf_buffer.lookup_transform(
            'base_link', 'gemini_camera_optical_frame', rclpy.time.Time()).transform
        p = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        R = quat_to_R((tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w))
        return p, R

    def tf_link(self):
        tr = self.tf_buffer.lookup_transform(
            'base_link', 'gemini_camera_link', rclpy.time.Time()).transform
        p = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        R = quat_to_R((tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w))
        return p, R

    def tf_panel(self):
        """base_link -> panel_base_frame: i tag sono definiti nel pannello."""
        if not self.tf_buffer.can_transform('base_link', 'panel_base_frame',
                                            rclpy.time.Time()):
            raise RuntimeError('TF base_link -> panel_base_frame assente')
        tr = self.tf_buffer.lookup_transform(
            'base_link', 'panel_base_frame', rclpy.time.Time()).transform
        p = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        R = quat_to_R((tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w))
        return p, R


def project(K, R, t, pts):
    pc = (R.T @ (pts.T - t[:, None])).T
    if np.any(pc[:, 2] <= 0.01):
        return None, pc
    u = K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]
    v = K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]
    return np.stack([u, v], axis=1), pc


def main():
    args = parse_args()
    rclpy.init()
    node = Calib()
    try:
        print('In attesa di un frame con entrambi gli ArUco...', flush=True)
        samples = []
        attempts = 0
        while len(samples) < args.samples and attempts < 80:
            attempts += 1
            node.latest = None          # ogni campione deve essere un frame NUOVO
            bgr = node.grab()
            found = node.detect(bgr)
            if 0 in found and 1 in found:
                samples.append(found)
            else:
                print(f'  frame {attempts}: visti {sorted(found)} (servono 0 e 1)',
                      flush=True)
                time.sleep(0.4)
        if not samples:
            raise SystemExit('ArUco 0 e 1 mai visti insieme: inquadra il pannello '
                             '(pre-aim) e riprova')

        K = np.asarray(node.caminfo.k, dtype=float).reshape(3, 3)
        p_tf, R_tf = node.tf_optical()
        p_link, R_link = node.tf_link()
        p_pan, R_pan = node.tf_panel()

        # angoli dei due tag in base_link (i tag sono definiti nel pannello)
        tag_pts = {}
        for name in TAG_IDS:
            c3, _ = tag_pose(TAGS[name])
            h = MARKER_SIZE / 2.0
            pts = np.array([[-h, 0, -h], [h, 0, -h], [h, 0, h], [-h, 0, h]])
            pts = pts @ rot_z(PANEL_BASE_YAW).T + c3
            tag_pts[name] = (pts @ R_pan.T + p_pan)

        # Due letture distinte:
        #  - CENTRO del tag: media dei 4 angoli, robusta all'ordine/roll del
        #    pattern. E' l'errore vero della camera (render vs TF).
        #  - angoli: il loro residuo contiene anche l'orientamento del pattern
        #    sul quadrato (ambiguita' di 90 gradi), quindi vale come sanity
        #    check, non come misura della camera.
        center_res = []
        per_tag = {}
        for found in samples:
            for name, tid in TAG_IDS.items():
                proj, pc = project(K, R_tf, p_tf, tag_pts[name])
                if proj is None:
                    continue
                d = found[tid].mean(axis=0) - proj.mean(axis=0)
                z = float(pc[:, 2].mean())
                center_res.append(d)
                per_tag.setdefault(name, []).append((d, z))

        center_res = np.array(center_res)
        mean_px = center_res.mean(axis=0)
        spread_px = center_res.std(axis=0)
        print(f'\nframes utili: {len(samples)}   (coppie centro-tag: {len(center_res)})')
        print(f'ERRORE CAMERA (centro tag): ({mean_px[0]:+.1f}, {mean_px[1]:+.1f}) px'
              f'  |r|={np.linalg.norm(mean_px):.1f} px'
              f'  ripetibilita\' ({spread_px[0]:.1f}, {spread_px[1]:.1f}) px',
              flush=True)

        m_off = []
        for name in TAG_IDS:
            if name not in per_tag:
                continue
            ds = np.array([d for d, _ in per_tag[name]])
            zs = np.array([z for _, z in per_tag[name]])
            d = ds.mean(axis=0)
            z = float(zs.mean())
            mm = np.array([d[0] * z / K[0, 0], d[1] * z / K[1, 1]]) * 1000.0
            m_off.append(mm)
            print(f'  {name}: centro ({d[0]:+.1f}, {d[1]:+.1f}) px  '
                  f'= {np.round(mm, 1)} mm sul piano a {z * 100:.0f} cm', flush=True)

        if len(m_off) == 2:
            dm = np.array(m_off)
            diff = np.linalg.norm(dm[0] - dm[1])
            print(f'\ndifferenza tra i due tag: {diff:.1f} mm  '
                  f'(se ~0 l\'errore e\' una TRASLAZIONE costante; se grande '
                  f'c\'e\' anche una ROTAZIONE, i tag hanno raggi diversi dal '
                  f'centro ottico)', flush=True)
            if diff < 3.0:
                print(f'=> entro il rumore: TRASLAZIONE di '
                      f'{np.round(dm.mean(axis=0), 1)} mm', flush=True)
            else:
                print('=> NON costante: componente di ROTAZIONE '
                      f'({diff:.1f} mm tra i due tag)', flush=True)

        print(f'\nverdetto: il render e il TF ottico '
              f'{"COINCIDONO" if np.linalg.norm(mean_px) < 12 else "NON coincidono"}'
              f' (soglia 12 px = 1% del lato corto)', flush=True)
        for zref in (0.25, 0.46):
            print(f'  a z={zref:.2f} m l\'errore vale '
                  f'({mean_px[0] * zref / K[0, 0] * 1000:+.1f}, '
                  f'{mean_px[1] * zref / K[1, 1] * 1000:+.1f}) mm', flush=True)

        report = {
            'frames': len(samples),
            'camera_error_px': mean_px.tolist(),
            'repeatability_px': spread_px.tolist(),
            'tag_offset_mm': (np.array(m_off) if m_off else np.zeros((0, 2))).tolist(),
            'per_tag': {n: {'offset_px': np.array([d for d, _ in per_tag[n]]).mean(axis=0).tolist(),
                            'depth_m': float(np.mean([z for _, z in per_tag[n]]))}
                        for n in per_tag},
            'tf_optical_base_link': {'t': p_tf.tolist(),
                                     'rpy_deg': np.degrees(euler_from_R(R_tf)).tolist()},
            'camera_link_base_link': {'t': p_link.tolist()},
        }
        with open(args.out_json, 'w') as f:
            json.dump(report, f, indent=2)
        print(f'\nreport: {args.out_json}', flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()