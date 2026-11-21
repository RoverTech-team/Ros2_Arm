#!/usr/bin/env python3
"""Cattura frame RGB REALI dalla camera del mondo Gazebo.

Legge /camera/rgbd/image (ros_gz_bridge) e disegna il mirino della leva
proiettata con TF + camera_info, cosi' il frame reale si confronta con la
previsione del demo.

Due modalita':
  - aperture del gripper a una posa fissa (--openings, --goto-arrival);
  - una vista per OGNI leva (--levers), con la IK del demo: serve a validare
    il montaggio camera con immagini vere, una per leva.

Prerequisiti:
  - demo/Simulazione Gazebo attiva (mondo panel_interaction) con il pannello
    e il robot spawnati;
  - il demo C++ NON deve muovere il braccio mentre si cattura (chiudere
    panel_interaction_demo).

Uso tipico:
  ros2 run rover_arm_panel_description capture_gripper_openings.py \
      --goto-arrival -1.661 -0.571 -1.188 1.168 -1.205 \
      --openings 0 0.005 0.010 0.015

  ros2 run rover_arm_panel_description capture_gripper_openings.py \
      --levers "0.677 -0.114 0.175" "0.677 -0.056 0.176" \
      --lever-names main_switch lever1 --out-dir /tmp \
      --gripper-open 0.025 --seed-joints -1.400 -0.500 0.120 0.0 0.0

Con --gripper-open la vista leva usa la STESSA apertura della validazione a
posa fissa (frame grip_open_XXXmm.png) e la stampa sull'immagine, cosi' i due
controlli di visibilita' sono confrontabili.

Salva lever_<nome>.png (o grip_open_XXXmm.png) e stampa un riepilogo.
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data

JOINTS = ('j1_joint', 'j2_joint', 'j3_joint', 'ee_base_joint_x', 'ee_base_joint_y')


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--openings', nargs='+', type=float,
                   default=[0.0, 0.005, 0.010, 0.015],
                   help='Aperture slider_joint in metri')
    p.add_argument('--goto-arrival', nargs=5, type=float, metavar='Q',
                   help='Giunti di arrivo (8 cm) da raggiungere prima di catturare')
    p.add_argument('--out-dir', default='/tmp')
    p.add_argument('--settle', type=float, default=1.0,
                   help='Attesa dopo ogni movimento (s)')
    p.add_argument('--move-seconds', type=float, default=10.0,
                   help='Durata della traiettoria verso ogni posa (s)')
    p.add_argument('--lever-center', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                   help='Centro leva in base_link: disegna il mirino verde')
    p.add_argument('--levers', nargs='+', metavar='"X Y Z"',
                   help='Cattura una vista per OGNI leva (IK con la stessa logica '
                        'del demo) e salva lever_<nome>.png')
    p.add_argument('--lever-names', nargs='+', metavar='NAME',
                   help='Nomi dei file per --levers (stesso ordine)')
    p.add_argument('--lever-frames', nargs='+', metavar='FRAME',
                   help='Frame TF dei pomelli (es. main_switch_knob): le posizioni '
                        'sono lette live, il rover si sposta mentre il braccio lavora')
    p.add_argument('--standoff', type=float, default=0.08,
                   help='Distanza di pre-approach usata dalla IK (m, default 0.08)')
    p.add_argument('--gripper-open', type=float,
                   help='Apertura slider_joint (m) impostata prima delle catture. '
                        'Senza questo flag le dita restano come le ha lasciate il '
                        'demo: la vista leva dipende da uno stato che nessuno '
                        'controlla, e due run non sono confrontabili')
    p.add_argument('--settle-tol', type=float, default=0.02,
                   help='Errore giunti massimo (rad) accettato prima di scattare '
                        '(default 0.02): sopra questa soglia il frame mostra una '
                        'posa diversa da quella proiettata')
    p.add_argument('--label', help='Etichetta stampata sull\'immagine (es. il '
                        'nome della leva quando si usa --lever-center)')
    p.add_argument('--aim-camera', action='store_true',
                   help='Dopo la IK cerca (j1 e polso) la posa in cui la leva '
                        'cade vicino al centro ottico: la punta si scosta di '
                        'quanto serve alla camera (posa di OSSERVAZIONE)')
    p.add_argument('--seed-joints', nargs=5, type=float, metavar='Q',
                   help='Seed IK (posa di pre-aim): rende la scelta del ramo '
                        'deterministica anche se il demo non ha ancora pre-aimato')
    p.add_argument('--wrist-offset', type=float, default=0.0,
                   help='Rotazione aggiunta a ee_base_joint_x (rad) dopo la IK. '
                        'Misurato a 8 cm: sposta quasi solo v (~1000 px/rad), '
                        'NON chiude il gap orizzontale')
    p.add_argument('--roll-offset', type=float, default=0.0,
                   help='Rotazione aggiunta a ee_base_joint_y (rad). Misurato a '
                        '8 cm: +0.25/+0.50 rad portano v a 197/357 px con u quasi '
                        'fermo (-160/-133): NON sostituisce il view search')
    return p.parse_args()


def parse_levers(values):
    levers = []
    for raw in values:
        parts = [float(v) for v in raw.replace(',', ' ').split()]
        if len(parts) != 3:
            raise SystemExit(f'leva non valida: {raw!r} (attesi 3 numeri)')
        levers.append(np.asarray(parts))
    return levers


def lever_from_tf(node, frame, timeout=10.0):
    """Posizione del pomello in base_link, via odom (i frame del pannello
    sono pubblicati su odom e il rover si sposta: va letta sul momento)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            t = node.tf_buffer.lookup_transform('base_link', 'odom',
                                                rclpy.time.Time()).transform
            p = node.tf_buffer.lookup_transform('odom', frame,
                                                rclpy.time.Time()).transform.translation
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
            continue
        q = t.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        T = np.array((t.translation.x, t.translation.y, t.translation.z))
        return R @ np.array((p.x, p.y, p.z)) + T
    raise RuntimeError(f'TF non disponibile per {frame}')


def compute_arrivals(levers, standoff, frames=None, seed=None, aim_camera=False):
    """IK di pre-approach con il modulo del demo (richiede MoveIt attivo).
    Con frames, le posizioni sono lette live da TF e restituite insieme.
    Con aim_camera, dopo la IK si cerca (j1 e polso) la posa in cui la
    PROIEZIONE della leva cade vicino al centro ottico: la camera sta 10 cm
    fuori asse e a 8 cm la IK nuda lascia la leva fuori dal FOV."""
    import importlib.util
    import os
    from ament_index_python.packages import get_package_share_directory

    path = os.path.join(get_package_share_directory('rover_arm_panel_description'),
                        'launch', 'main_switch_pre_approach_AUTO.py')
    spec = importlib.util.spec_from_file_location('ms_auto', path)
    ms = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ms)

    node = ms.Test()
    try:
        for srv in (node.scene, node.fk, node.valid, node.plan):
            if not srv.wait_for_service(timeout_sec=60.0):
                raise RuntimeError('Servizio MoveIt assente: ' + srv.srv_name)
        original = node.get_state()
        q_start = (np.asarray(seed, dtype=float) if seed is not None
                   else ms.q_from_state(original))
        if frames:
            levers = [lever_from_tf(node, f) for f in frames]
        arrivals = []
        for center in levers:
            target = center - np.array((standoff, 0.0, 0.0))
            goal = node.find_joint_goal(q_start, target, original)
            if aim_camera:
                # piu' candidate ordinate: il controller puo' non riuscire a
                # TENERE la prima (contatto/saturazione), quindi si prova la
                # successiva invece di fotografare una posa che non esiste.
                arrivals.append(aim_candidates(node, ms, goal,
                                               np.asarray(center, dtype=float)))
            else:
                arrivals.append([goal])
        return arrivals, levers
    finally:
        node.destroy_node()


def aim_candidates(node, ms, goal, center, max_n=8):
    """Posa in cui la leva cade vicino al centro ottico (j1 + polso).

    La IK mette la PUNTA sul target, ma la camera e' 10 cm fuori asse: a 8 cm
    la leva resta 20-140 px fuori dal bordo. Si scandiscono j1 (orizzontale) e
    polso (verticale) e si tengono solo pose che MoveIt dichiara valide
    (limiti di giunto e collisioni): una posa fuori limite cambierebbe
    l'inquadratura di centinaia di pixel rispetto alla previsione.
    """
    node.calibrate_camera_geometry(node.get_state())
    cx, cy = node.cam_K[0, 2], node.cam_K[1, 2]
    state = node.get_state()
    cands = []
    for dj in np.arange(-0.60, 0.6001, 0.02):
        for dq in np.arange(-0.60, 0.6001, 0.05):
            q = list(goal)
            q[0] = goal[0] + dj
            q[3] = goal[3] + dq
            rs = ms.put_joints(state, tuple(q))
            view = node.projected_lever(rs, center)
            if view is None:
                continue
            u, v = view['pixel']
            if not (0.0 <= u < node.cam_W and 0.0 <= v < node.cam_H):
                continue
            score = (max(abs(u - cx), abs(v - cy))
                     + 40.0 * abs(dj) + 30.0 * abs(dq))
            if view['occ'] < 8.0:
                score += 2000.0
            cands.append((score, tuple(float(x) for x in q), view, dj, dq))
    cands.sort(key=lambda c: c[0])
    out = []
    for cand in cands:
        if len(out) >= max_n:
            break
        req = ms.GetStateValidity.Request()
        req.robot_state = ms.put_joints(state, cand[1])
        req.group_name = 'arm'
        if ms.call(node, node.valid, req).valid:
            out.append((cand[1], cand[2], cand[3], cand[4]))
    if not out:
        print('   aim: nessuna posa valida porta la leva nel FOV', flush=True)
        return [goal]
    for i, (q, view, dj, dq) in enumerate(out[:3], 1):
        u, v = view['pixel']
        print(f'   aim candidata {i}: j1 {dj:+.2f} polso {dq:+.2f} -> leva '
              f'({u:.0f},{v:.0f}) margine={view["margin"]:.0f} px '
              f'occl={view["occ"]:+.0f} px', flush=True)
    return [c[0] for c in out]


class Capture(Node):
    def __init__(self):
        super().__init__('capture_gripper_openings')
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/arm_controller/follow_joint_trajectory')
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 '/gripper_controller/follow_joint_trajectory')
        self.latest = None
        self.caminfo = None
        self.joints = {}
        from sensor_msgs.msg import JointState
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.joints.update(dict(zip(m.name, m.position))), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, '/camera/rgbd/camera_info',
                                 self.on_info, 10)
        self.create_subscription(Image, '/camera/rgbd/image', self.on_image,
                                 qos_profile_sensor_data)
        self.depth = None
        self.create_subscription(Image, '/camera/rgbd/depth_image', self.on_depth,
                                 qos_profile_sensor_data)
        self.get_logger().info('In attesa di un frame RGB...')

    def on_info(self, msg):
        self.caminfo = msg

    def on_image(self, msg):
        self.latest = msg

    def on_depth(self, msg):
        self.depth = msg

    def lever_pixel(self, center, timeout=5.0):
        """Proietta il centro leva (base_link) nel frame ottico attuale."""
        if self.caminfo is None:
            return None
        if not self.tf_buffer.can_transform('base_link',
                                            'gemini_camera_optical_frame',
                                            rclpy.time.Time()):
            return None
        tf = self.tf_buffer.lookup_transform('base_link',
                                             'gemini_camera_optical_frame',
                                             rclpy.time.Time()).transform
        q = tf.transform if hasattr(tf, 'transform') else tf
        x, y, z, w = q.rotation.x, q.rotation.y, q.rotation.z, q.rotation.w
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        p = np.array([q.translation.x, q.translation.y, q.translation.z])
        pc = R.T @ (np.asarray(center, dtype=float) - p)
        if pc[2] <= 0.03:
            return None
        K = np.asarray(self.caminfo.k, dtype=float).reshape(3, 3)
        u = K[0, 0] * pc[0] / pc[2] + K[0, 2]
        v = K[1, 1] * pc[1] / pc[2] + K[1, 2]
        return float(u), float(v), float(pc[2])

    def sensor_ok(self, u, v, z, tol=0.06, half=4):
        """La telecamera VEDE davvero la leva in (u,v) a profondita' z?

        Serve perche' a certe pose il render RGB lascia mezze immagini vuote
        (geometria tagliata dal near plane della camera stessa) mentre la
        depth e' corretta. Senza questa verifica si pubblicano frame in cui
        la leva non c'e', pur con una proiezione giusta.
        Ritorna (ok, rgb_std, depth_med).
        """
        msg = self.wait_frame()
        w, h = msg.width, msg.height
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if buf.size != w * h * 3:
            return False, float('nan'), float('nan')
        img = buf.reshape(h, w, 3)
        uc, vc = int(round(u)), int(round(v))
        if not (half <= uc < w - half and half <= vc < h - half):
            return False, float('nan'), float('nan')
        patch = img[vc - half:vc + half + 1, uc - half:uc + half + 1].reshape(-1, 3)
        rgb_std = float(patch.std(axis=0).mean())
        dmed = float('nan')
        if self.depth is not None:
            dbuf = np.frombuffer(bytes(self.depth.data),
                                 np.float32 if '32F' in self.depth.encoding else np.uint16)
            if dbuf.size == self.depth.width * self.depth.height:
                d = dbuf.reshape(self.depth.height, self.depth.width).astype(np.float32)
                if '32F' not in self.depth.encoding:
                    d = d / 1000.0
                dp = d[vc - half:vc + half + 1, uc - half:uc + half + 1]
                dp = dp[np.isfinite(dp)]
                if dp.size:
                    dmed = float(np.median(dp))
        depth_ok = (dmed == dmed) and abs(dmed - z) < tol
        return depth_ok and rgb_std > 3.0, rgb_std, dmed

    def wait_frame(self, timeout=15.0):
        t0 = time.time()
        while self.latest is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest is None:
            raise RuntimeError('Nessun frame da /camera/rgbd/image')
        return self.latest

    def send(self, client, names, positions, seconds):
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'action server assente per {names}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        from builtin_interfaces.msg import Duration
        d = Duration()
        d.sec = int(seconds)
        d.nanosec = int((seconds - int(seconds)) * 1e9)
        pt.time_from_start = d
        goal.trajectory.points = [pt]
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('goal rifiutato')
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=max(30.0, seconds * 6))
        return True

    def goto(self, goal_q, seconds=6.0):
        """Traiettoria lineare morbida dallo stato attuale ai giunti goal."""
        from sensor_msgs.msg import JointState
        cur = {}
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: cur.update(dict(zip(m.name, m.position))), 10)
        t0 = time.time()
        while not all(j in cur for j in JOINTS) and time.time() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not all(j in cur for j in JOINTS):
            raise RuntimeError('/joint_states non disponibile')
        q0 = np.array([cur[j] for j in JOINTS])
        q1 = np.array(goal_q, dtype=float)
        n = 60
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        from builtin_interfaces.msg import Duration
        for i in range(1, n + 1):
            a = i / n
            a = a * a * a * (10 - 15 * a + 6 * a * a)  # min-jerk
            pt = JointTrajectoryPoint()
            pt.positions = list(q0 + a * (q1 - q0))
            d = Duration()
            t = seconds * i / n
            d.sec = int(t)
            d.nanosec = int((t - int(t)) * 1e9)
            pt.time_from_start = d
            goal.trajectory.points.append(pt)
        if not self.arm.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('/arm_controller assente')
        fut = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('goal braccio rifiutato')
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=seconds * 6)
        self.get_logger().info('Posa di arrivo raggiunta.')

    def wait_settled(self, goal, tol=0.02, timeout=25.0):
        """Aspetta che i giunti siano DAVVERO sul goal prima di fotografare.

        Con movimenti di polso grandi il controller arriva corto (misurato fino
        a 0.29 rad): lo scatto allora mostra una posa diversa da quella con cui
        si proietta, e sembra un errore di camera/render che non esiste.
        """
        t0 = time.time()
        err = float('nan')
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(j in self.joints for j in JOINTS):
                err = max(abs(self.joints[j] - g) for j, g in zip(JOINTS, goal))
                if err <= tol:
                    return err
            time.sleep(0.2)
        return err

    def current_joints(self, timeout=5.0):
        from sensor_msgs.msg import JointState
        cur = {}
        sub = self.create_subscription(JointState, '/joint_states',
                                       lambda m: cur.update(dict(zip(m.name, m.position))), 10)
        t0 = time.time()
        while not all(j in cur for j in JOINTS) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return cur

    def snapshot(self, path, value=None, center=None, label=None):
        msg = self.wait_frame()
        w, h = msg.width, msg.height
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if buf.size != w * h * 3:
            raise RuntimeError(f'dati immagine inattesi: {buf.size} != {w*h*3} '
                               f'(encoding={getattr(msg, "encoding", "?")})')
        img = buf.reshape(h, w, 3)
        if getattr(msg, 'encoding', 'rgb8') == 'bgr8':
            bgr = np.ascontiguousarray(img)
        else:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if center is not None:
            alvo = self.lever_pixel(center)
            if alvo is not None:
                self.draw_target(bgr, alvo, w, h)
        if value is not None:
            cv2.putText(bgr, f'opening={value*1000:.0f} mm', (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if label:
            cv2.putText(bgr, label, (10, h - 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        # riferimento fisso al centro ottico: serve a vedere che il mirino
        # si muove davvero dentro il FOV (non e' ancorato all'immagine)
        cv2.drawMarker(bgr, (w // 2, h // 2), (160, 160, 160),
                       cv2.MARKER_TILTED_CROSS, 18, 1)
        if not cv2.imwrite(path, bgr):
            raise RuntimeError(f'scrittura immagine fallita: {path}')
        if center is not None:
            alvo = self.lever_pixel(center)
            if alvo is not None:
                inside = (0.0 <= alvo[0] < w) and (0.0 <= alvo[1] < h)
                return w, h, (float(alvo[0]), float(alvo[1]), float(alvo[2]), inside)
        return w, h, None

    @staticmethod
    def draw_target(bgr, alvo, w, h):
        """Mirino libero nel FOV: segue la proiezione, si ferma al bordo con
        una freccia se la leva e' fuori immagine."""
        u, v, z = float(alvo[0]), float(alvo[1]), float(alvo[2])
        inside = (0.0 <= u < w) and (0.0 <= v < h)
        color = (0, 255, 0) if inside else (0, 165, 255)
        uc = int(round(min(max(u, 14.0), w - 15.0)))
        vc = int(round(min(max(v, 14.0), h - 15.0)))
        cv2.circle(bgr, (uc, vc), 26, color, 2)
        cv2.drawMarker(bgr, (uc, vc), color, cv2.MARKER_CROSS, 20, 2)
        if not inside:
            dx = int(max(-90.0, min(90.0, (u - uc))))
            dy = int(max(-90.0, min(90.0, (v - vc))))
            cv2.arrowedLine(bgr, (uc, vc), (uc + dx, vc + dy), color, 2, tipLength=0.35)
        testo = (f'leva u={u:.0f} v={v:.0f} z={z:.2f}m  '
                 + ('nel FOV' if inside else 'FUORI FOV'))
        cv2.putText(bgr, testo, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def main():
    args = parse_args()
    rclpy.init()

    if args.levers or args.lever_frames:
        if args.levers and args.lever_frames:
            raise SystemExit('usa --levers oppure --lever-frames, non entrambi')
        frames = args.lever_frames
        levers = parse_levers(args.levers) if args.levers else None
        n_levers = len(frames) if frames else len(levers)
        if args.lever_names:
            names = args.lever_names
        elif frames:
            names = [f[:-5] if f.endswith('_knob') else f for f in frames]
        else:
            names = [f'{i:02d}' for i in range(n_levers)]
        if len(names) != n_levers:
            raise SystemExit('--lever-names deve avere lo stesso numero di leve')
        print(f'IK per {n_levers} leve (standoff {args.standoff * 100:.0f} cm)...',
              flush=True)
        arrivals, centers = compute_arrivals(levers, args.standoff, frames=frames,
                                              seed=args.seed_joints,
                                              aim_camera=args.aim_camera)
        if args.wrist_offset or args.roll_offset:
            # Il view search del demo ruota il polso tenendo ferma la punta;
            # qui basta la rotazione per giudicare la VISIBILITA' (la punta
            # si sposta, la leva no).
            arrivals = [tuple(list(q[:3])
                              + [q[3] + args.wrist_offset, q[4] + args.roll_offset])
                        for q in arrivals]
        node = Capture()
        try:
            if args.gripper_open is not None:
                # Stessa apertura della validazione a posa fissa
                # (grip_open_XXXmm.png): la vista leva va giudicata con le
                # dita nella posa in cui il demo le userebbe davvero.
                node.send(node.grip, ['slider_joint'], [args.gripper_open], 1.5)
                time.sleep(args.settle)
                print(f'apertura dita = {args.gripper_open * 1000:.0f} mm', flush=True)
            print(f'{"leva":>16} | errore | posizione leva nel frame | file', flush=True)
            frame_list = frames or [None] * n_levers
            for name, cands, frame, center0 in zip(names, arrivals, frame_list, centers):
                node.get_logger().info(f'Vista leva {name}...')
                center = lever_from_tf(node, frame) if frame else center0
                err = float('nan')
                goal = cands[0]
                frame_ok = False
                for k, cand in enumerate(cands, 1):
                    node.goto(cand, seconds=args.move_seconds)
                    time.sleep(min(args.settle, 1.0))
                    err = node.wait_settled(cand, tol=args.settle_tol)
                    goal = cand
                    if not (err == err) or err > args.settle_tol:
                        continue
                    alvo = node.lever_pixel(center)
                    if alvo is None:
                        print(f'{name:>16} | candidata {k}/{len(cands)}: leva non proiettabile',
                              flush=True)
                        continue
                    ok, s_rgb, d_meas = node.sensor_ok(alvo[0], alvo[1], alvo[2])
                    if ok:
                        frame_ok = True
                        if k > 1 or len(cands) > 1:
                            print(f'{name:>16} | candidata {k}/{len(cands)}: frame valido '
                                  f'(rgb_std={s_rgb:.1f}, depth={d_meas:.2f} m)', flush=True)
                        break
                    print(f'{name:>16} | candidata {k}/{len(cands)}: frame NON valido '
                          f'(rgb_std={s_rgb:.1f}, depth={d_meas:.2f} m vs z={alvo[2]:.2f} m)',
                          flush=True)
                if not frame_ok:
                    det = ', '.join(f'{j}={node.joints.get(j, float("nan")):+.3f}/'
                                    f'{g:+.3f}' for j, g in zip(JOINTS, goal))
                    print(f'{name:>16} | ATTENZIONE: nessuna candidata da un frame valido '
                          f'[{det}]', flush=True)
                node.latest = None
                path = os.path.join(args.out_dir, f'lever_{name}.png')
                w, h, alvo = node.snapshot(path, center=center, value=args.gripper_open,
                                           label=f'leva {name} @ {args.standoff * 100:.0f} cm')
                if alvo is None:
                    fov = 'leva non proiettabile (TF/ottica)'
                else:
                    u, v, z, inside = alvo
                    fov = (f'u={u:4.0f} v={v:4.0f} z={z:.2f}m '
                           + ('NEL FOV' if inside else 'FUORI FOV'))
                print(f'{name:>16} | {err:6.3f} | {fov:30s} | {path}  ({w}x{h})',
                      flush=True)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    node = Capture()
    try:
        if args.goto_arrival is not None:
            node.goto(args.goto_arrival)
            time.sleep(1.0)
        node.get_logger().info('Frame di controllo (dita chiuse)...')
        node.wait_frame()
        print(f'{"opening":>8} | file', flush=True)
        for value in args.openings:
            node.send(node.grip, ['slider_joint'], [value], 1.5)
            time.sleep(args.settle)
            node.latest = None
            path = f'{args.out_dir}/grip_open_{int(round(value*1000)):03d}mm.png'
            w, h, alvo = node.snapshot(path, value=value, center=args.lever_center,
                                   label=args.label)
            extra = ''
            if alvo is not None:
                u, v, z, inside = alvo
                extra = (f'  leva u={u:.0f} v={v:.0f} z={z:.2f}m '
                         + ('NEL FOV' if inside else 'FUORI FOV'))
            print(f'{value:8.3f} | {path}  ({w}x{h}){extra}', flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()