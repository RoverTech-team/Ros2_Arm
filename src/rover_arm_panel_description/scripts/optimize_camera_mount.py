#!/usr/bin/env python3
"""Ottimizzatore CMA-ES del montaggio camera (6 DOF) per la visibilita' del pannello.

Il problema: a 6-8 cm dalla leva il polso deve restare fuori dall'immagine ma la
leva deve essere dentro il FOV, con il dito che non la copre. Il montaggio
attuale (offset fisso) non ci riesce; qui si cerca una terna COMPLETA
(traslazione + rotazione) relativa a ee_base.

Come funziona
-------------
1. Per ogni leva e per ogni distanza di pre-approach costruisce la posa di
   arrivo con la STESSA IK del demo, poi chiama FK una volta sola: le pose di
   ee_base e delle dita NON dipendono dal montaggio camera, quindi il costo e'
   puro numpy e CMA-ES puo' fare migliaia di valutazioni in pochi secondi.
2. La camera e' parametrizzata da 6 numeri normalizzati in [-1, 1]:
      traslazione (x, y, z) attorno all'offset calibrato  (+-0.15 m)
      rotazione    (roll, pitch, yaw) in coordinate ee_base (+-35 deg)
   quindi il supporto puo' essere montato anche INCLINATO, in qualunque verso.
3. Costo ROBUSTO (non una media di comodo):
      - penalita' hinge normalizzate su margine FOV e occlusione delle dita;
      - profondita' fuori range -> penalita' piena;
      - aggregazione media + caso peggiore, cosi' una singola leva che resta
        nascosta non viene compensata dalle altre;
      - regolarizzazione debole sull'entita' dello spostamento, per non
        scegliere un montaggio assurdo a parita' di visibilita'.
4. Riporta la terna migliore, la tabella per leva e il delta da applicare in
   ee_base, oltre a un file JSON con tutti i numeri.

Prerequisiti: demo pannello avviata (MoveIt + camera + pose depth), oppure
--levers per passare i centri a mano.

  ros2 run rover_arm_panel_description optimize_camera_mount.py \\
      --standoffs 0.08 0.06 --generations 60

  ros2 run rover_arm_panel_description optimize_camera_mount.py \\
      --levers "0.677 -0.114 0.175" "0.677 -0.056 0.176" --json /tmp/mount.json

Dipendenze: numpy, cmaes (pip install cmaes).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory

from cmaes import CMA

CAMERA_MIN_DEPTH_M = 0.08
CAMERA_MAX_DEPTH_M = 1.20
DEFAULT_OPENINGS = (0.0,)
SLIDER = 'slider_joint'
FINGER_BOX_CENTER = (0.016, 0.070, -0.011)   # centro box dito nel frame del link
FINGER_BOX_HALF = (0.021, 0.070, 0.011)      # semi-estensioni: box 4.2 x 14 x 2.2 cm

# Limiti del montaggio, espressi in coordinate ee_base.
MAX_TRANSLATION_M = 0.15
MAX_ROTATION_RAD = math.radians(35.0)

# Soglie del demo (main_switch_pre_approach_AUTO.py).
OCCLUSION_CLEAR_PX = 8.0
FOV_MARGIN_RULE = 0.12          # 12% del lato corto dell'immagine
FOV_MARGIN_MIN_PX = 60.0
FOV_MARGIN_MAX_PX = 110.0

# Pesi del costo.
WORST_CASE_WEIGHT = 0.5
REG_TRANSLATION_WEIGHT = 0.02
REG_ROTATION_WEIGHT = 0.02


# ---------------------------------------------------------------------------
# Geometria di base
# ---------------------------------------------------------------------------

def load_demo_module():
    """Carica il modulo del demo (IK/FK/proiezione gia' validati)."""
    path = os.path.join(
        get_package_share_directory('rover_arm_panel_description'),
        'launch', 'main_switch_pre_approach_AUTO.py')
    import importlib.util
    spec = importlib.util.spec_from_file_location('ms_auto', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))


def rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c)))


def rpy_from_matrix(R: np.ndarray) -> Tuple[float, float, float]:
    """Angoli (roll, pitch, yaw) con convenzione R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


@dataclass(frozen=True)
class Mount:
    """Terna della camera ottica rispetto a ee_base."""

    translation: np.ndarray
    rotation: np.ndarray

    def rpy(self) -> Tuple[float, float, float]:
        return rpy_from_matrix(self.rotation)

    def as_dict(self) -> Dict[str, List[float]]:
        roll, pitch, yaw = self.rpy()
        return {
            'translation_m': [round(float(v), 4) for v in self.translation],
            'rpy_rad': [round(roll, 4), round(pitch, 4), round(yaw, 4)],
            'rpy_deg': [round(math.degrees(v), 1) for v in (roll, pitch, yaw)],
        }


@dataclass(frozen=True)
class ParamSpace:
    """Mappa un vettore normalizzato in [-1, 1] su una terna di montaggio."""

    base_translation: np.ndarray
    base_rotation: np.ndarray
    max_translation_m: float = MAX_TRANSLATION_M
    max_rotation_rad: float = MAX_ROTATION_RAD
    size: int = 6

    def bounds(self) -> np.ndarray:
        return np.tile(np.array([-1.0, 1.0]), (self.size, 1))

    def to_mount(self, x: Sequence[float]) -> Mount:
        x = np.asarray(x, dtype=float)
        if x.size != self.size:
            raise ValueError(f'attesi {self.size} parametri, ricevuti {x.size}')
        x = np.clip(x, -1.0, 1.0)
        translation = self.base_translation + x[:3] * self.max_translation_m
        roll, pitch, yaw = (x[3:] * self.max_rotation_rad)
        delta = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
        return Mount(translation=translation,
                     rotation=delta @ self.base_rotation)


# ---------------------------------------------------------------------------
# Scene: pose di arrivo precalcolate (indipendenti dal montaggio)
# ---------------------------------------------------------------------------

@dataclass
class Scene:
    name: str
    standoff_m: float
    opening_m: float
    lever_points: np.ndarray      # (3, 3) centro + estremi della leva
    ee_translation: np.ndarray    # (3,)
    ee_rotation: np.ndarray       # (3, 3)
    finger_boxes: np.ndarray      # (2, 8, 3) vertici dei box dito
    finger_centers: np.ndarray    # (2, 3) centri dei box dito


@dataclass
class SceneMetrics:
    margin_px: float
    occlusion_px: float
    depth_m: float
    depth_ok: bool
    penalty: float


def with_slider(ms, robot_state, value: float):
    import copy
    new = copy.deepcopy(robot_state)
    names = list(new.joint_state.name)
    pos = list(new.joint_state.position)
    if SLIDER in names:
        pos[names.index(SLIDER)] = float(value)
    new.joint_state.position = pos
    return new


def fk_poses(ms, node, robot_state):
    req = ms.GetPositionFK.Request()
    req.header.frame_id = 'base_link'
    req.fk_link_names = ['ee_base', 'ee_finger1', 'ee_finger2']
    req.robot_state = robot_state
    result = ms.call(node, node.fk, req)
    if result.error_code.val != ms.MoveItErrorCodes.SUCCESS:
        raise RuntimeError('FK fallita (codice %s)' % result.error_code.val)
    return {k: v.pose for k, v in zip(result.fk_link_names, result.pose_stamped)}


def build_scene(ms, node, original, q_start, name, center, standoff, opening,
                joints=None):
    """IK verso il pre-approach e FK delle pose utili alla proiezione."""
    center = np.asarray(center, dtype=float)
    target = center - np.array((standoff, 0.0, 0.0))
    if joints is None:
        joints = node.find_joint_goal(q_start, target, original)
    state = ms.put_joints(original, joints)
    if opening:
        state = with_slider(ms, state, opening)
    poses = fk_poses(ms, node, state)

    ee = poses['ee_base']
    ee_t = np.array((ee.position.x, ee.position.y, ee.position.z), float)
    ee_R = ms.rotation_matrix(ee.orientation)

    boxes = []
    centers = []
    for link in ('ee_finger1', 'ee_finger2'):
        fp = poses[link]
        p0 = np.array((fp.position.x, fp.position.y, fp.position.z), float)
        Rf = ms.rotation_matrix(fp.orientation)
        c = p0 + Rf @ np.array(FINGER_BOX_CENTER)
        corners = [c + Rf @ (np.array(FINGER_BOX_HALF) * np.array((sx, sy, sz)))
                   for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]
        boxes.append(np.asarray(corners))
        centers.append(c)
    lever = np.array((center,
                      center + np.array((0.0, 0.0, -0.025)),
                      center + np.array((0.0, 0.0, 0.015))))

    return Scene(name=name, standoff_m=standoff, opening_m=opening,
                 lever_points=lever, ee_translation=ee_t, ee_rotation=ee_R,
                 finger_boxes=np.asarray(boxes),
                 finger_centers=np.asarray(centers))


# ---------------------------------------------------------------------------
# Modello di costo robusto
# ---------------------------------------------------------------------------

class ViewCost:
    """Costo hinge, media + caso peggiore, su tutte le scene."""

    def __init__(self, space: ParamSpace, scenes: List[Scene], K: np.ndarray,
                 width: int, height: int,
                 margin_req_px: float, occlusion_req_px: float = OCCLUSION_CLEAR_PX):
        self.space = space
        self.scenes = scenes
        self.K = np.asarray(K, dtype=float)
        self.width = int(width)
        self.height = int(height)
        self.margin_req = float(margin_req_px)
        self.occlusion_req = float(occlusion_req_px)
        self.fx, self.fy = float(self.K[0, 0]), float(self.K[1, 1])
        self.cx, self.cy = float(self.K[0, 2]), float(self.K[1, 2])
        self._cache: Dict[Tuple[float, ...], float] = {}

    # -- proiezione ------------------------------------------------------
    def _project(self, mount: Mount, scene: Scene, points: np.ndarray):
        p_cam = scene.ee_translation + scene.ee_rotation @ mount.translation
        R_cam = scene.ee_rotation @ mount.rotation
        flat = np.asarray(points, dtype=float).reshape(-1, 3)
        pc = (flat - p_cam) @ R_cam          # (N, 3) nel frame camera
        z = pc[:, 2]
        safe = np.where(np.abs(z) < 1e-6, 1e-6, z)
        u = self.fx * pc[:, 0] / safe + self.cx
        v = self.fy * pc[:, 1] / safe + self.cy
        return u, v, z

    # -- metriche --------------------------------------------------------
    def metrics(self, mount: Mount) -> List[SceneMetrics]:
        rows: List[SceneMetrics] = []
        for scene in self.scenes:
            u, v, z = self._project(mount, scene, scene.lever_points)
            depth_ok = bool(np.all((z >= CAMERA_MIN_DEPTH_M) & (z <= CAMERA_MAX_DEPTH_M)))
            if not np.all(z > CAMERA_MIN_DEPTH_M):
                margin = -1e3        # leva dietro la camera: fuori, senza numeri enormi
            else:
                margin = float(np.min(np.concatenate((
                    u, self.width - u, v, self.height - v))))
                margin = float(np.clip(margin, -1e3, 1e3))

            # Occlusione: le dita sono lastre 4.2 x 14 x 2.2 cm (non cilindri).
            # Si proietta il box e si misura la distanza della leva dal poligono.
            occlusion = 1e3
            sane = True
            for corners, fcenter in zip(scene.finger_boxes, scene.finger_centers):
                _uc, _vc, zc = self._project(mount, scene, fcenter)
                if zc[0] > z[0] - 0.005:
                    continue                      # dito dietro alla leva
                uu, vv, zz = self._project(mount, scene, corners)
                if np.any(zz <= 0.03):
                    sane = False
                    continue
                hull = cv2.convexHull(np.stack([uu, vv], axis=1).astype(np.float32))
                for ul, vl in zip(u, v):
                    d = float(cv2.pointPolygonTest(hull, (float(ul), float(vl)), True))
                    occlusion = min(occlusion, -d)     # dentro -> negativo = coperta
            occlusion = float(np.clip(occlusion, -1e3, 1e3))

            penalty = 0.0
            penalty += max(0.0, self.margin_req - margin) / self.margin_req
            penalty += max(0.0, self.occlusion_req - occlusion) / self.occlusion_req
            if not depth_ok:
                penalty += 1.0
            if not np.all(sane):
                penalty += 0.5   # camera a ridosso delle dita: montaggio degenere
            rows.append(SceneMetrics(margin_px=margin, occlusion_px=occlusion,
                                     depth_m=float(np.mean(z)), depth_ok=depth_ok,
                                     penalty=float(penalty)))
        return rows

    # -- costo -----------------------------------------------------------
    def __call__(self, x: Sequence[float]) -> float:
        key = tuple(round(float(v), 6) for v in x)
        cached = self._cache.get(key)
        if cached is not None:
            return cached
        mount = self.space.to_mount(x)
        penalties = np.array([m.penalty for m in self.metrics(mount)])
        data = ((1.0 - WORST_CASE_WEIGHT) * float(np.mean(penalties))
                + WORST_CASE_WEIGHT * float(np.max(penalties)))
        delta_t = mount.translation - self.space.base_translation
        norm_t = float(np.linalg.norm(delta_t)) / self.space.max_translation_m
        roll, pitch, yaw = mount.rpy()
        base_rpy = rpy_from_matrix(self.space.base_rotation)
        norm_r = float(np.linalg.norm(np.subtract((roll, pitch, yaw), base_rpy))) \
            / self.space.max_rotation_rad
        value = data + REG_TRANSLATION_WEIGHT * norm_t ** 2 \
            + REG_ROTATION_WEIGHT * norm_r ** 2
        self._cache[key] = value
        return value

    def is_feasible(self, mount: Mount) -> bool:
        return all(m.penalty <= 1e-9 for m in self.metrics(mount))


# ---------------------------------------------------------------------------
# Ottimizzazione
# ---------------------------------------------------------------------------

@dataclass
class CmaesResult:
    best_x: np.ndarray
    best_value: float
    generations: int
    history: List[float] = field(default_factory=list)


def optimize(cost: ViewCost, generations: int, population: int, sigma0: float,
             seed: Optional[int], patience: int, verbose: bool = True) -> CmaesResult:
    space = cost.space
    optimizer = CMA(mean=np.zeros(space.size), sigma=sigma0,
                    bounds=space.bounds(), population_size=population,
                    seed=seed)
    best_x = np.zeros(space.size)
    best_value = float('inf')
    history: List[float] = []
    stale = 0
    for gen in range(1, generations + 1):
        solutions = [optimizer.ask() for _ in range(optimizer.population_size)]
        values = [cost(sol) for sol in solutions]
        optimizer.tell(list(zip(solutions, values)))
        gen_best = int(np.argmin(values))
        if values[gen_best] < best_value - 1e-6:
            best_value = float(values[gen_best])
            best_x = np.asarray(solutions[gen_best], dtype=float)
            stale = 0
        else:
            stale += 1
        history.append(best_value)
        if verbose and (gen == 1 or gen % 5 == 0):
            print(f'  gen {gen:3d} | costo={best_value:.4f} | '
                  f'sigma={getattr(optimizer, "_sigma", float("nan")):.3f}', flush=True)
        if stale >= patience:
            if verbose:
                print(f'  stop: nessun miglioramento da {patience} generazioni', flush=True)
            break
    return CmaesResult(best_x=best_x, best_value=best_value,
                       generations=len(history), history=history)


# ---------------------------------------------------------------------------
# CLI / report
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    scene = p.add_argument_group('scene')
    scene.add_argument('--levers', nargs='+', metavar='"X Y Z"',
                       help='Centri leva in base_link; se assente usa le pose depth '
                            'del demo (main_switch + leve fresche)')
    scene.add_argument('--standoffs', nargs='+', type=float, default=[0.08],
                       help='Distanze di pre-approach in metri (default 0.08)')
    scene.add_argument('--openings', nargs='+', type=float, default=list(DEFAULT_OPENINGS),
                       help='Aperture dita da valutare (default 0 = chiuse)')
    scene.add_argument('--arrival-joints', nargs=5, type=float, metavar='Q',
                       help='Posa di arrivo fissa per tutte le scene (salta la IK)')

    mount = p.add_argument_group('montaggio')
    mount.add_argument('--max-translation', type=float, default=MAX_TRANSLATION_M,
                       help='Spostamento massimo per asse (m, default 0.15)')
    mount.add_argument('--max-rotation-deg', type=float,
                       default=math.degrees(MAX_ROTATION_RAD),
                       help='Inclinazione massima per asse (deg, default 35)')
    mount.add_argument('--start-translation', nargs=3, type=float, metavar='M',
                       help='Offset di partenza (default: calibrazione TF)')

    opt = p.add_argument_group('ottimizzatore')
    opt.add_argument('--generations', type=int, default=60)
    opt.add_argument('--population', type=int, default=0,
                       help='Dimensione popolazione (0 = 4+3ln(n))')
    opt.add_argument('--sigma0', type=float, default=0.30)
    opt.add_argument('--seed', type=int, default=7)
    opt.add_argument('--patience', type=int, default=20,
                       help='Generazioni senza miglioramento prima di fermarsi')

    out = p.add_argument_group('output')
    out.add_argument('--json', help='Scrive il report completo in JSON')
    out.add_argument('--fov-margin', type=float, default=None,
                       help='Margine FOV minimo in px (default: regola del demo)')
    return p.parse_args()


def parse_levers(values: Sequence[str]) -> List[np.ndarray]:
    levers = []
    for raw in values:
        parts = [float(v) for v in raw.replace(',', ' ').split()]
        if len(parts) != 3:
            raise SystemExit(f'leva non valida: {raw!r} (attesi 3 numeri)')
        levers.append(np.asarray(parts))
    return levers


def collect_levers(ms, node) -> List[Tuple[str, np.ndarray]]:
    """main_switch + leve fresche pubblicate dal detector."""
    positions: List[Tuple[str, np.ndarray]] = []
    deadline = time.monotonic() + 180.0
    while time.monotonic() < deadline and not node.measurements_ready():
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.measurements_ready():
        raise RuntimeError('Timeout: pose depth assenti (usa --levers)')
    if node.switch is not None:
        p = node.switch.pose.position
        positions.append(('main_switch', np.array((p.x, p.y, p.z))))
    for name, msg in sorted(node.fresh_levers().items()):
        p = msg.pose.position
        positions.append((name, np.array((p.x, p.y, p.z))))
    return positions


def print_report(space: ParamSpace, scenes: List[Scene], metrics: List[SceneMetrics],
                 best: Mount, margin_req: float):
    print('\n== MONTAGGIO OTTIMO (camera ottica rispetto a ee_base) ==')
    roll, pitch, yaw = best.rpy()
    print(f'  traslazione = ({best.translation[0]:+.4f}, {best.translation[1]:+.4f}, '
          f'{best.translation[2]:+.4f}) m')
    print(f'  rotazione   = roll {math.degrees(roll):+.1f} deg, '
          f'pitch {math.degrees(pitch):+.1f} deg, yaw {math.degrees(yaw):+.1f} deg')
    dt = best.translation - space.base_translation
    base_rpy = rpy_from_matrix(space.base_rotation)
    drpy = np.subtract((roll, pitch, yaw), base_rpy)
    print(f'  delta dal montaggio attuale: '
          f'dt=({dt[0]:+.4f}, {dt[1]:+.4f}, {dt[2]:+.4f}) m; '
          f'drpy=({math.degrees(drpy[0]):+.1f}, {math.degrees(drpy[1]):+.1f}, '
          f'{math.degrees(drpy[2]):+.1f}) deg')

    print(f'\n== VISIBILITA PER SCENA (margine richiesto {margin_req:.0f} px, '
          f'occlusione minima {OCCLUSION_CLEAR_PX:.0f} px) ==')
    print(f'{"scena":>22} | {"standoff":>8} | {"ap":>5} | {"margine":>7} | '
          f'{"occ":>7} | {"z_cam":>6} | esito')
    print('-' * 82)
    for scene, m in zip(scenes, metrics):
        ok = m.penalty <= 1e-9
        print(f'{scene.name:>22} | {scene.standoff_m:8.3f} | {scene.opening_m:5.3f} | '
              f'{m.margin_px:7.0f} | {m.occlusion_px:7.1f} | {m.depth_m:6.2f} | '
              f'{"OK" if ok else "NO"}')

    feasible = all(m.penalty <= 1e-9 for m in metrics)
    print()
    if feasible:
        print('Tutte le scene sono visibili: montaggio utilizzabile.')
    else:
        worst = min(range(len(metrics)), key=lambda i: metrics[i].penalty)
        print('NON tutte le scene sono visibili; peggiore: '
              f'{scenes[worst].name} (margine {metrics[worst].margin_px:.0f} px, '
              f'occ {metrics[worst].occlusion_px:.1f} px).')
    print('URDF: applica dt a camera_mount_joint.xyz (assi ee_base) e drpy a '
          'camera_rover_mount_joint.rpy;')
    print('      poi verifica con tune_gripper_opening.py (senza --stand-values), '
          'che rilegge il mount da TF.')


def main():
    args = parse_args()
    ms = load_demo_module()

    rclpy.init()
    node = ms.Test()
    try:
        for srv in (node.scene, node.fk, node.valid, node.plan):
            if not srv.wait_for_service(timeout_sec=60.0):
                raise RuntimeError('Servizio MoveIt assente: ' + srv.srv_name)
        original = node.get_state()
        q_start = ms.q_from_state(original)
        node.calibrate_camera_geometry(original)

        if node.cam_K is None:
            raise RuntimeError('camera_info assente: impossibile proiettare')
        margin_req = args.fov_margin if args.fov_margin is not None else \
            max(FOV_MARGIN_MIN_PX,
                min(FOV_MARGIN_MAX_PX, FOV_MARGIN_RULE * min(node.cam_W, node.cam_H)))

        if args.levers:
            levers = [(f'leva_{i}', c) for i, c in enumerate(parse_levers(args.levers))]
        else:
            levers = collect_levers(ms, node)

        base_translation = (np.asarray(args.start_translation, dtype=float)
                            if args.start_translation is not None
                            else np.array(node.camera_mount_translation, dtype=float))
        space = ParamSpace(base_translation=base_translation,
                           base_rotation=np.array(node.camera_mount_rotation, dtype=float),
                           max_translation_m=args.max_translation,
                           max_rotation_rad=math.radians(args.max_rotation_deg))

        print(f'Montaggio attuale: t=({base_translation[0]:+.4f}, '
              f'{base_translation[1]:+.4f}, {base_translation[2]:+.4f}) m',
              flush=True)
        print(f'Scene: {len(levers)} leve x {len(args.standoffs)} standoff x '
              f'{len(args.openings)} aperture', flush=True)

        scenes: List[Scene] = []
        for name, center in levers:
            for standoff in args.standoffs:
                for opening in args.openings:
                    label = f'{name}@{standoff:.3f}'
                    if len(args.openings) > 1:
                        label += f'/ap{opening * 1000:.0f}mm'
                    scenes.append(build_scene(ms, node, original, q_start, label,
                                              center, standoff, opening,
                                              joints=args.arrival_joints))

        cost = ViewCost(space, scenes, node.cam_K, node.cam_W, node.cam_H, margin_req)
        population = args.population or (4 + int(3 * math.log(max(2, space.size))))

        print(f'\nCMA-ES: dim={space.size}, popolazione={population}, '
              f'generazioni max={args.generations}', flush=True)
        t0 = time.monotonic()
        result = optimize(cost, args.generations, population, args.sigma0,
                          args.seed, args.patience)
        elapsed = time.monotonic() - t0
        best = space.to_mount(result.best_x)
        metrics = cost.metrics(best)
        print(f'CMA-ES: {result.generations} generazioni in {elapsed:.1f} s, '
              f'costo={result.best_value:.4f}', flush=True)

        print_report(space, scenes, metrics, best, margin_req)

        if args.json:
            payload = {
                'mount': best.as_dict(),
                'delta': {
                    'translation_m': [round(float(v), 4) for v in
                                      (best.translation - space.base_translation)],
                },
                'cost': round(result.best_value, 6),
                'feasible': cost.is_feasible(best),
                'generations': result.generations,
                'thresholds': {
                    'fov_margin_px': round(margin_req, 1),
                    'occlusion_px': OCCLUSION_CLEAR_PX,
                    'depth_m': [CAMERA_MIN_DEPTH_M, CAMERA_MAX_DEPTH_M],
                },
                'bounds': {
                    'translation_m': space.max_translation_m,
                    'rotation_deg': math.degrees(space.max_rotation_rad),
                },
                'scenes': [
                    {'name': s.name, 'standoff_m': s.standoff_m,
                     'opening_m': s.opening_m,
                     'margin_px': round(m.margin_px, 1),
                     'occlusion_px': round(m.occlusion_px, 1),
                     'depth_m': round(m.depth_m, 3),
                     'ok': bool(m.penalty <= 1e-9)}
                    for s, m in zip(scenes, metrics)
                ],
                'history': [round(v, 6) for v in result.history],
            }
            with open(args.json, 'w', encoding='utf-8') as fh:
                json.dump(payload, fh, indent=2)
            print(f'\nReport JSON: {args.json}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()