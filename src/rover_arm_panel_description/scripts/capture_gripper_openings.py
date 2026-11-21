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
      --lever-names main_switch lever1 --out-dir /tmp

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


def compute_arrivals(levers, standoff, frames=None):
    """IK di pre-approach con il modulo del demo (richiede MoveIt attivo).
    Con frames, le posizioni sono lette live da TF e restituite insieme."""
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
        q_start = ms.q_from_state(original)
        if frames:
            levers = [lever_from_tf(node, f) for f in frames]
        arrivals = []
        for center in levers:
            target = center - np.array((standoff, 0.0, 0.0))
            arrivals.append(node.find_joint_goal(q_start, target, original))
        return arrivals, levers
    finally:
        node.destroy_node()


class Capture(Node):
    def __init__(self):
        super().__init__('capture_gripper_openings')
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/arm_controller/follow_joint_trajectory')
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 '/gripper_controller/follow_joint_trajectory')
        self.latest = None
        self.caminfo = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, '/camera/rgbd/camera_info',
                                 self.on_info, 10)
        self.create_subscription(Image, '/camera/rgbd/image', self.on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info('In attesa di un frame RGB...')

    def on_info(self, msg):
        self.caminfo = msg

    def on_image(self, msg):
        self.latest = msg

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
                u, v, z = alvo
                cv2.circle(bgr, (int(round(u)), int(round(v))), 32, (0, 255, 0), 2)
                cv2.drawMarker(bgr, (int(round(u)), int(round(v))),
                               (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
                cv2.putText(bgr, f'CHECK RGB-D z={z:.2f}m', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if value is not None:
            cv2.putText(bgr, f'opening={value*1000:.0f} mm', (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if label:
            cv2.putText(bgr, label, (10, h - 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imwrite(path, bgr)
        return w, h


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
        arrivals, centers = compute_arrivals(levers, args.standoff, frames=frames)
        node = Capture()
        try:
            print(f'{"leva":>16} | errore | file', flush=True)
            frame_list = frames or [None] * n_levers
            for name, goal, frame, center0 in zip(names, arrivals, frame_list, centers):
                node.get_logger().info(f'Vista leva {name}...')
                node.goto(goal, seconds=args.move_seconds)
                time.sleep(args.settle)
                cur = node.current_joints()
                err = max(abs(cur[j] - g) for j, g in zip(JOINTS, goal)) if cur else float('nan')
                if err > 0.05:
                    print(f'{name:>16} | ATTENZIONE: giunti fermi a {err:.3f} rad dal goal',
                          flush=True)
                node.latest = None
                center = lever_from_tf(node, frame) if frame else center0
                path = os.path.join(args.out_dir, f'lever_{name}.png')
                w, h = node.snapshot(path, center=center,
                                     label=f'leva {name} @ {args.standoff * 100:.0f} cm')
                print(f'{name:>16} | {err:6.3f} | {path}  ({w}x{h})', flush=True)
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
            w, h = node.snapshot(path, value=value, center=args.lever_center)
            print(f'{value:8.3f} | {path}  ({w}x{h})', flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()