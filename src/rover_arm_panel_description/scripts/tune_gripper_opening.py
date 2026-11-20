#!/usr/bin/env python3
"""Tuning headless dell'apertura dita (slider_joint) per la visibilita' leva.

Prerequisito: demo pannello gia' avviata e allineata (move_group + camera +
pose pannello pubblicate). Consigliato in headless e plan-only:

  ros2 launch rover_arm_panel_description demo_rover_panel_depth.launch.py \\
      use_gazebo:=true use_nav2:=false headless:=true

Poi:

  ros2 run rover_arm_panel_description tune_gripper_opening.py

Il nodo NON muove il braccio: per ogni apertura valuta, nella posa di
pre-approach (8 cm) calcolata con la stessa IK del demo:
  - occ   : distanza in px tra leva e bordo delle dita proiettate
            (negativo = dito davanti alla leva)
  - Tip   : spostamento della punta FK rispetto alle dita chiuse (cm),
            causato dalla rotazione del dito sul proprio giunto
Stampa una tabella e suggerisce l'apertura piu' piccola che libera la leva
senza spostare la punta oltre 2 cm.
"""
import argparse
import copy
import importlib.util
import math
import os

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory

OPENINGS = (0.0, 0.005, 0.010, 0.015, 0.020, 0.025, 0.030, 0.040)
MAX_TIP_SHIFT_M = 0.030
SLIDER = 'slider_joint'


def load_demo_module():
    path = os.path.join(
        get_package_share_directory('rover_arm_panel_description'),
        'launch', 'main_switch_pre_approach_AUTO.py')
    spec = importlib.util.spec_from_file_location('ms_auto', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def with_slider(rs, value):
    new = copy.deepcopy(rs)
    names = list(new.joint_state.name)
    pos = list(new.joint_state.position)
    if SLIDER in names:
        pos[names.index(SLIDER)] = float(value)
    new.joint_state.position = pos
    return new


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lever-center', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='Centro leva in base_link (salta l attesa di /panel/*)')
    parser.add_argument('--target', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='Target punta in base_link (default: centro - 8 cm)')
    parser.add_argument('--arrival-joints', nargs=5, type=float, metavar='Q',
                        help='Se presente, valuta questa posa di arrivo invece '
                             'di risolvere la IK (demo non allineata)')
    parser.add_argument('--images', action='store_true',
                        help='Salva le immagini di debug delle 3 aperture migliori')
    parser.add_argument('--out-dir', default='/tmp',
                        help='Cartella per le immagini (default /tmp)')
    return parser.parse_args()


def project(n, R_cam, p_cam, point):
    pc = R_cam.T @ (np.asarray(point, dtype=float) - p_cam)
    if pc[2] <= 0.03:
        return None
    fx, fy = n.cam_K[0, 0], n.cam_K[1, 1]
    cx, cy = n.cam_K[0, 2], n.cam_K[1, 2]
    return fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy, float(pc[2])


def render_view(ms, n, state, center, opening, occ, shift_cm, path):
    """Vista PREDETTA (FK + calibrazione camera): leva rossa, dita gialle."""
    import cv2

    req = ms.GetPositionFK.Request()
    req.header.frame_id = 'base_link'
    req.fk_link_names = ['ee_base', 'ee_finger1', 'ee_finger2']
    req.robot_state = state
    res = ms.call(n, n.fk, req)
    poses = {k: v.pose for k, v in zip(res.fk_link_names, res.pose_stamped)}

    ee = poses['ee_base']
    R_ee = ms.rotation_matrix(ee.orientation)
    p_ee = np.array((ee.position.x, ee.position.y, ee.position.z), float)
    p_cam = p_ee + R_ee @ n.camera_mount_translation
    R_cam = R_ee @ n.camera_mount_rotation

    H, W = n.cam_H, n.cam_W
    img = np.full((H, W, 3), 45, dtype=np.uint8)
    cv2.rectangle(img, (2, 2), (W - 3, H - 3), (90, 90, 90), 1)

    c = np.asarray(center, dtype=float)
    lever_pts = (c, c + np.array((0., 0., -0.025)), c + np.array((0., 0., 0.015)))
    proj = [project(n, R_cam, p_cam, q) for q in lever_pts]
    if proj[0] is not None:
        (u0, v0, z0) = proj[0]
        (u2, v2, _) = proj[2] if proj[2] is not None else proj[0]
        height_px = max(6, int(abs(v2 - v0)))
        width_px = max(4, int(height_px * 0.35))
        cv2.line(img, (int(u0), int(v0)), (int(u2), int(v2)),
                 (0, 0, 200), width_px)
        cv2.line(img, (int(u0), int(v0)), (int(u2), int(v2)),
                 (0, 0, 255), 2)

        for name in ('ee_finger1', 'ee_finger2'):
            fp = poses[name]
            p0 = np.array((fp.position.x, fp.position.y, fp.position.z), float)
            for sw in np.linspace(0.0, 1.0, 9):
                local = (ms.TCP_LOCAL[0], ms.TCP_LOCAL[1] * float(sw), ms.TCP_LOCAL[2])
                pf = p0 + np.array(ms.rot(fp.orientation, local))
                pr = project(n, R_cam, p_cam, pf)
                if pr is None or pr[2] > z0 + 0.01:
                    continue
                uf, vf, zf = pr
                r = max(2, int(n.cam_K[0, 0] * ms.FINGER_RADIUS_M / zf))
                cv2.circle(img, (int(uf), int(vf)), r, (0, 190, 255), -1)
                cv2.circle(img, (int(uf), int(vf)), r, (0, 120, 180), 2)

        cv2.drawMarker(img, (int(u0), int(v0)), (0, 255, 0),
                       cv2.MARKER_CROSS, 22, 2)
        cv2.putText(img, f'opening={opening*1000:.0f} mm  occ={occ:+.0f} px  '
                         f'tip={shift_cm:.1f} cm  z={z0:.2f} m',
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, 'VISTA PREDETTA (FK): leva=rossa, dita=gialle',
                    (12, H - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.imwrite(path, img)
        return True
    return False


def main():
    args = parse_args()
    ms = load_demo_module()
    rclpy.init()
    n = ms.Test()
    try:
        for srv in (n.scene, n.fk, n.valid, n.plan):
            if not srv.wait_for_service(timeout_sec=60.0):
                raise RuntimeError('Servizio MoveIt assente: ' + srv.srv_name)
        original = n.get_state()
        q_start = ms.q_from_state(original)
        n.calibrate_camera_geometry(original)

        if args.lever_center is not None:
            center = np.asarray(args.lever_center, dtype=float)
            if args.target is not None:
                target = np.asarray(args.target, dtype=float)
            else:
                target = center + 0.08 * np.asarray([-1.0, 0.0, 0.0])
            d = center - target
            n.center0, n.normal0 = center, d / max(float(np.linalg.norm(d)), 1e-9)
        else:
            print('Attendo allineamento pannello...', flush=True)
            deadline = n.get_clock().now().nanoseconds / 1e9 + 180.0
            while not (n.aligned and n.measurements_ready()):
                if n.get_clock().now().nanoseconds / 1e9 > deadline:
                    raise RuntimeError('Timeout: /panel/align_done o pose leva assenti')
                rclpy.spin_once(n, timeout_sec=0.2)
            target = n.get_target()

        if args.arrival_joints is not None:
            goal = tuple(float(v) for v in args.arrival_joints)
            print('Posa di arrivo fornita da CLI (IK saltata).', flush=True)
        else:
            goal = n.find_joint_goal(q_start, target, original)
        arrival = ms.put_joints(original, goal)
        print('Posa di arrivo (8 cm):', ms.fmt(goal), flush=True)
        print(f'{"apertura":>9} | {"occ":>6} | {"margine":>7} | {"z_cam":>6} | {"Tip":>6}',
              flush=True)
        print('-' * 48, flush=True)

        tip0 = n.fingertip_fk(with_slider(arrival, 0.0))
        rows = []
        for value in OPENINGS:
            st = with_slider(arrival, value)
            tip = n.fingertip_fk(st)
            shift = math.dist(tip, tip0) * 100.0
            try:
                view = n.projected_lever(st, n.center0)
            except Exception as exc:  # calibrazione/collisione
                print(f'{value:9.3f} | errore: {exc}', flush=True)
                continue
            if view is None:
                print(f'{value:9.3f} | leva non proiettabile (fuori range)', flush=True)
                continue
            rows.append((value, view['occ'], view['margin'], view['zcam'], shift))
            print(f'{value:9.3f} | {view["occ"]:6.1f} | {view["margin"]:7.0f} | '
                  f'{view["zcam"]:6.2f} | {shift:6.1f}', flush=True)

        good = [r for r in rows
                if r[1] >= ms.MIN_OCCLUSION_CLEAR_PX and r[4] <= MAX_TIP_SHIFT_M * 100.0]
        if args.images:
            # le 3 aperture PIU' PICCOLE che liberano la leva (trade-off col Tip)
            clear = [r for r in rows if r[1] >= ms.MIN_OCCLUSION_CLEAR_PX]
            picks = sorted(clear, key=lambda r: r[0])[:3]
            if len(picks) < 3:
                rest = [r for r in rows if r not in picks]
                rest.sort(key=lambda r: -r[1])
                picks += rest[:3 - len(picks)]
            for value, occ, margin, zcam, shift in picks:
                st = with_slider(arrival, value)
                path = os.path.join(args.out_dir,
                                    f'tune_open_{int(round(value * 1000)):03d}mm.png')
                if render_view(ms, n, st, n.center0, value, occ, shift, path):
                    print(f'immagine: {path}', flush=True)
        print()
        if good:
            best = min(good, key=lambda r: r[0])
            print(f'CONSIGLIATA apertura={best[0]:.3f} m '
                  f'(occ={best[1]:.0f} px, shift punta={best[4]:.1f} cm)', flush=True)
            print('Uso: --open-gripper true --gripper-open-pos '
                  f'{best[0]:.3f}', flush=True)
        else:
            print('Nessuna apertura soddisfa i criteri: tenere le dita chiuse '
                  'e affidarsi al roll del polso.', flush=True)
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()