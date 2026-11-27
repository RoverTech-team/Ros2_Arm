#!/usr/bin/env python3

import math
from collections import deque, Counter

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException


# Oltre questa eta' (secondi di clock di simulazione) un frame RGB e'
# considerato backlog: scartato prima di ArUco/depth per non misurare
# su un'immagine di decine di secondi prima.
MAX_RGB_FRAME_AGE_S = 0.75


# ============================================
# GEOMETRIA: PANNELLO VERTICALE
# ============================================

def enforce_vertical_panel(R_base_panel, p_base_panel, marker_positions):
    """Corregge la posa del pannello imponendo che sia verticale.

    PnP con due soli marker su una fascia orizzontale non riesce a fissare
    bene l'inclinazione avanti/indietro del pannello (qualche grado di
    errore, invisibile nell'errore di riproiezione). Quel piccolo errore,
    moltiplicato per i 40 cm che separano i marker dall'origine del
    pannello, sposta l'origine di circa 1 cm.

    Qui si usano solo i due centri dei marker (ben vincolati dall'immagine):
      - l'asse z del pannello e' l'asse z di base_link (pannello verticale)
      - la rotazione attorno a z (yaw) si ricava dalla retta tra i due marker
      - l'origine si ricava dai centri dei marker con la rotazione corretta

    Ritorna (R, p) oppure None se i due marker coincidono in pianta.
    """

    m0 = marker_positions[0]
    m1 = marker_positions[1]

    # centri dei marker in base_link, dalla posa grezza di PnP
    c0 = R_base_panel @ m0 + p_base_panel
    c1 = R_base_panel @ m1 + p_base_panel

    v = c1 - c0                 # direzione marker0 -> marker1 in base_link
    d = m1 - m0                 # stessa direzione nel frame del pannello

    if math.hypot(v[0], v[1]) < 1e-6:
        return None

    # yaw del pannello = angolo orizzontale misurato - angolo previsto
    # (tiene conto anche dei 2 mm di differenza in y tra i due marker)
    psi = math.atan2(v[1], v[0]) - math.atan2(d[1], d[0])

    ex = np.array([math.cos(psi), math.sin(psi), 0.0])
    ey = np.array([-math.sin(psi), math.cos(psi), 0.0])
    ez = np.array([0.0, 0.0, 1.0])

    R = np.column_stack((ex, ey, ez))

    # origine: media delle due stime ottenute da ciascun marker
    origins = [
        c0 - R @ m0,
        c1 - R @ m1
    ]
    p = np.mean(origins, axis=0)

    return R, p


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()

        # Se True (default) la posa del pannello viene corretta imponendo
        # che sia verticale in base_link (rover su terreno piano).
        # Mettere False per usare la posa grezza di PnP.
        self.declare_parameter('assume_vertical_panel', True)
        self.assume_vertical_panel = bool(
            self.get_parameter('assume_vertical_panel').value
        )

        # ========================================
        # CONFIGURAZIONE PANNELLO
        # ========================================

        self.marker_size = 0.045

        # Centri dei marker nel frame panel_base (valori dall'xacro:
        # aruco_tag_0 y=0.003, aruco_tag_1 y=0.001)
        self.marker_positions = {
            0: np.array([0.050, 0.003, 0.405]),
            1: np.array([0.280, 0.001, 0.405])
        }

        # Centro della faccia frontale del pannello nel frame panel_base
        # (box 0.33 x 0.45 m, faccia frontale a y=0)
        self.panel_face_center = np.array([0.165, 0.0, 0.225])

        # Il pannello ha la faccia frontale a y=0 e la normale uscente
        # e' +y (il retro sta in -y). La camera sta quindi sul lato +y
        # e guarda verso -y.
        self.panel_normal = np.array([0.0, 1.0, 0.0])

        # ========================================
        # CONTROLLI DA MISURARE CON LA DEPTH
        # ========================================
        # Coordinate nominali nel frame panel_base, ricavate da
        # panel.urdf.xacro. NON sono il rilevamento visivo dei comandi.
        # La proiezione PnP identifica la ROI prevista; la depth misura
        # il punto superficiale effettivamente visibile nella ROI.
        # Le coordinate Y sono stime della superficie frontale per
        # selettori in posa iniziale: cambiano con la loro articolazione.
        self.control_points = {
            'main_switch': np.array([0.050, 0.050, 0.240]),
            'lever_switch_1': np.array([0.108, 0.050, 0.240]),
            'lever_switch_2': np.array([0.165, 0.050, 0.240]),
            'lever_switch_3': np.array([0.223, 0.050, 0.240]),
            'lever_switch_4': np.array([0.280, 0.050, 0.240]),
            'rotary_switch_1': np.array([0.055, 0.0375, 0.300]),
            'rotary_switch_2': np.array([0.086, 0.0375, 0.300]),
            'rotary_switch_3': np.array([0.117, 0.0375, 0.300]),
            'rotary_switch_4': np.array([0.148, 0.0375, 0.300]),
            'rotary_switch_5': np.array([0.179, 0.0375, 0.300]),
            'power_switch_1': np.array([0.210, 0.040, 0.300]),
            'power_switch_2': np.array([0.241, 0.040, 0.300]),
        }
        self.control_samples = {
            name: deque(maxlen=25) for name in self.control_points
        }
        self.control_logged = set()
        self.depth_cache = deque(maxlen=8)
        # Visual re-observation trigger after 5 cm approach.
        self.reobserve_started_ns = None
        self.declare_parameter('max_rgb_depth_delta_s', 0.12)
        self.declare_parameter('depth_roi_radius_px', 3)
        self.declare_parameter('depth_prediction_tolerance_m', 0.025)
        self.declare_parameter('min_control_samples', 3)
        self.declare_parameter('enable_gui', True)
        self.enable_gui = bool(self.get_parameter('enable_gui').value)

        # Per un osservatore che guarda verso -y con z verso l'alto,
        # l'asse x del pannello cresce verso SINISTRA nell'immagine.
        # Ordine ArUco: TL, TR, BR, BL (senso orario nell'immagine).
        # Non conosco a priori come la texture e' orientata sulla faccia
        # del box, quindi provo le 4 rotazioni ciclistiche e tengo la
        # migliore (vedi estimate_panel_pose).
        h = self.marker_size / 2.0

        self.candidate_object_points = {}

        for shift in range(4):
            pts = {}
            for marker_id, (cx, cy, cz) in self.marker_positions.items():
                base = np.array([
                    [cx + h, cy, cz + h],   # TL
                    [cx - h, cy, cz + h],   # TR
                    [cx - h, cy, cz - h],   # BR
                    [cx + h, cy, cz - h]    # BL
                ], dtype=np.float64)
                pts[marker_id] = np.roll(base, shift, axis=0)
            self.candidate_object_points[shift] = pts

        # ========================================
        # ARUCO (compatibile con OpenCV < 4.7 e >= 4.7)
        # ========================================

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_5X5_250
        )

        if hasattr(cv2.aruco, 'ArucoDetector'):
            # OpenCV >= 4.7
            self.parameters = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(
                self.dictionary, self.parameters
            )
        else:
            # OpenCV < 4.7
            self.parameters = cv2.aruco.DetectorParameters_create()
            self.aruco_detector = None

        # ========================================
        # CAMERA
        # ========================================

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = None

        # ========================================
        # TF2
        # ========================================

        self.base_frame = 'base_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        # ========================================
        # STATO
        # ========================================

        self.marker_found = False
        self.last_log_time = -1.0
        self.frame_age_logged = False

        # Stima finale: parte quando il C++ segnala che il braccio e' fermo
        # nella posa finale (/panel/align_done). Si raccolgono N frame
        # consecutivi e si prende la mediana (riduce il rumore).
        self.align_done = False
        self.align_done_time = None
        self.final_done = False
        self.final_samples = []
        self.final_pose = None          # (R, p) della stima finale
        self.final_distance = None
        self.n_final_samples = 15

        # Nuova funzionalita' depth: cache posa pannello nel rover quando PnP
        # vede gli ArUco. Dopo il tilt la camera puo' non vedere i marker.
        self.cached_panel_pose_base = None
        self.depth_frames_since_align = 0
        self.depth_reported = False
        # Solo diagnostica: registra PERCHÉ una ROI dei pulsanti viene scartata.
        # Nessuna modifica ai filtri e nessun comando al braccio.
        self.depth_roi_status_counts = {
            name: Counter() for name in self.control_points
        }
        self.depth_roi_last = {}

        # ========================================
        # SUBSCRIBER
        # ========================================

        # Callback pesanti (immagine + depth) fuori dal gruppo del listener TF.
        # Gruppi SEPARATI: se condividessero un MutuallyExclusiveCallbackGroup
        # la depth resterebbe in coda mentre ArUco elabora l'RGB (8 Hz su
        # 1280x800) e la cache depth invecchierebbe oltre max_rgb_depth_delta_s.
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.depth_group = MutuallyExclusiveCallbackGroup()

        # QoS RELIABLE, come il publisher di ros_gz_bridge: creando la
        # sottoscrizione BEST_EFFORT prima che il bridge esista, il
        # discovery non la aggancia e il nodo resta muto per sempre.
        # Il backlog che ne deriva e' gestito dalla guardia MAX_RGB_FRAME_AGE_S
        # in image_callback (i frame vecchi vengono scartati subito).
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgbd/image',
            self.image_callback,
            10,
            callback_group=self.image_group,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgbd/camera_info',
            self.camera_info_callback,
            10
        )

        # Come per l'RGB: RELIABLE per agganciare il publisher del bridge,
        # che nasce dopo questo nodo. La depth resta leggera da convertire.
        self.depth_sub = self.create_subscription(
            Image, '/camera/rgbd/depth_image',
            self.depth_callback, 10,
            callback_group=self.depth_group,
        )

        # Il C++ pubblica con QoS "transient local": stesso QoS qui, cosi'
        # il segnale arriva anche se questo nodo parte dopo.
        done_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.align_done_sub = self.create_subscription(
            Bool,
            '/panel/align_done',
            self.align_done_callback,
            done_qos
        )
        self.reobserve_sub = self.create_subscription(
            Bool, '/panel/request_reobserve', self.reobserve_callback, 10
        )
        self.reobserve_ready_pub = self.create_publisher(Bool, '/panel/reobserve_ready', 10)

        # ========================================
        # PUBLISHER
        # ========================================

        self.marker_pub = self.create_publisher(
            Bool,
            '/panel/markers_detected',
            10
        )

        self.pose_camera_pub = self.create_publisher(
            PoseStamped,
            '/panel/pose_camera',
            10
        )
        self.final_pose_pub = self.create_publisher(
            PoseStamped,
            '/panel/final_pose',
            10
        )       
        self.pose_base_pub = self.create_publisher(
            PoseStamped,
            '/panel/pose_base',
            10
        )
        

        # Posizione dei marker nell'immagine, per il C++ che centra il braccio:
        # data = [errore_orizzontale, errore_verticale, numero_marker_visibili]
        # errori normalizzati: 0 = centro immagine, +-1 = bordo;
        # orizzontale + = marker a destra, verticale + = marker in basso.
        self.image_error_pub = self.create_publisher(
            Float32MultiArray,
            '/panel/image_error',
            10
        )

        # Distanza perpendicolare base_link -> faccia del pannello [m]
        self.distance_pub = self.create_publisher(
            Float32,
            '/panel/distance',
            10
        )

        # Un topic PoseStamped per ogni controllo. La posizione e'
        # misurata da depth; l'orientamento viene stimato dagli ArUco.
        # NON inviare queste pose direttamente a un movimento di contatto.
        self.control_publishers = {
            name: self.create_publisher(
                PoseStamped, f'/panel/controls/{name}/pose_base', 10
            )
            for name in self.control_points
        }

        # Visualizzazione RViz: dati di depth validati, nessun comando al braccio.
        # TRANSIENT_LOCAL permette a RViz di vedere i marker anche se aperto dopo.
        self.rviz_controls_pub = self.create_publisher(
            MarkerArray, '/panel/controls/markers',
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.rviz_marker_last_s = float('-inf')
        self.rviz_first_log = False

        self.get_logger().info(
            'Detector avviato: ArUco 0 e 1 + TF2 '
            f'(pannello verticale imposto: {self.assume_vertical_panel})'
        )

    # ============================================
    # CAMERA INFO
    # ============================================

    @staticmethod
    def stamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def depth_callback(self, msg):
        """Mantiene gli ultimi frame per abbinare timestamp RGB/depth."""
        if (self.reobserve_started_ns is not None
                and self.stamp_ns(msg.header.stamp) <= self.reobserve_started_ns):
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if msg.encoding == '16UC1':
                depth = depth.astype(np.float32) * 0.001  # mm -> m
            elif msg.encoding == '32FC1':
                depth = depth.astype(np.float32, copy=False)
            else:
                self.get_logger().warn(
                    f'Encoding depth non supportato: {msg.encoding}',
                    throttle_duration_sec=5.0)
                return
            self.depth_cache.append((
                self.stamp_ns(msg.header.stamp),
                np.array(depth, copy=True),
                msg.header.frame_id,
            ))
        except Exception as ex:
            self.get_logger().warn(f'Conversione depth fallita: {ex}',
                                   throttle_duration_sec=5.0)

    def depth_for_image(self, msg, shape):
        if not self.depth_cache:
            return None
        stamp = self.stamp_ns(msg.header.stamp)
        best = min(self.depth_cache, key=lambda item: abs(item[0] - stamp))
        dt = abs(best[0] - stamp) * 1e-9
        max_dt = float(self.get_parameter('max_rgb_depth_delta_s').value)
        # Solo per la ri-osservazione a robot FERMO accettiamo fino a 0.17 s.
        # Tutte e due le immagini devono essere successive al trigger.
        if self.reobserve_started_ns is not None:
            if best[0] <= self.reobserve_started_ns or stamp <= self.reobserve_started_ns:
                return None
            max_dt = min(0.17, max_dt + 0.05)
        if dt > max_dt or best[1].shape != tuple(shape[:2]):
            self.get_logger().warn(
                'RGB/depth non sincronizzate o dimensioni diverse: '
                f'dt={dt:.3f}s, RGB={tuple(shape[:2])}, depth={best[1].shape}',
                throttle_duration_sec=5.0)
            return None
        if best[2] and msg.header.frame_id and best[2] != msg.header.frame_id:
            self.get_logger().warn(
                'I frame ottici RGB e depth sono diversi: serve registrazione.',
                throttle_duration_sec=5.0)
            return None
        return best[1]

    def _depth_diag(self, name, reason, pixel=None, z_expected=None,
                    z_observed=None, raw_valid=None, matched=None):
        """Contatore di esiti: non influenza alcuna misura o topic."""
        self.depth_roi_status_counts[name][reason] += 1
        self.depth_roi_last[name] = {
            'pixel': pixel, 'z_expected': z_expected,
            'z_observed': z_observed, 'raw_valid': raw_valid,
            'matched': matched,
        }

    def _depth_diag_all(self, reason):
        for name in self.control_points:
            # Teniamo il pixel dell'ultimo frame con depth valida, anche se
            # quello corrente e' asincrono: serve per vedere la ROI nella GUI.
            self.depth_roi_status_counts[name][reason] += 1

    # ============================================================
    # LEVE ROSSE: SEGMENTAZIONE RGB SOLO PER SCEGLIERE I PIXEL DEPTH
    # ============================================================
    # Le altre 7 ROI continuano a usare il vecchio codice INVARIATO.
    # Non si usa il colore per ricavare coordinate 3D: quelle sono SOLO
    # da depth + CameraInfo + TF, quando il braccio e' fermo.
    @staticmethod
    def find_red_lever_depth(rgb_frame, depth, projected, K, dist):
        """Misure nel frame ottico: {nome: (xyz_camera, z_depth, pixel)}, diagnostica.

        Per evitare falsi pulsanti, richiede CINQUE oggetti rossi verticali
        nella fascia prevista e depth piu' vicina del pannello circostante.
        Se il test non passa, NON pubblica alcuna posa di leva.
        """
        import itertools

        names = tuple(projected)
        diagnostics = {name: 'componenti_rosse_non_identificate' for name in names}
        if (len(names) != 5 or depth.ndim != 2 or rgb_frame is None
                or rgb_frame.shape[:2] != depth.shape):
            return {}, diagnostics

        uv = np.asarray([projected[name] for name in names], dtype=np.float64)
        if not np.all(np.isfinite(uv)):
            return {}, diagnostics
        H, W = depth.shape
        x0 = max(0, int(np.floor(np.min(uv[:, 0]) - 95)))
        x1 = min(W, int(np.ceil(np.max(uv[:, 0]) + 95)))
        y0 = max(0, int(np.floor(np.min(uv[:, 1]) - 60)))
        y1 = min(H, int(np.ceil(np.max(uv[:, 1]) + 60)))
        if x1-x0 < 20 or y1-y0 < 20:
            return {}, diagnostics

        # Comandi simulati rossi (HSV hue attorno a 0/179).
        hsv = cv2.cvtColor(rgb_frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        mask = (cv2.inRange(hsv, (0, 75, 28), (14, 255, 255)) |
                cv2.inRange(hsv, (172, 75, 28), (179, 255, 255)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((3, 3), dtype=np.uint8))
        n, labels, stats, centres = cv2.connectedComponentsWithStats(mask, 8)
        components = []
        # Separiamo steli verticali da selettori tondi, testi e macchie.
        for label in range(1, n):
            lx, ly, w, h, area = [int(v) for v in stats[label]]
            if not (area >= 75 and 5 <= w <= 55 and 16 <= h <= 115
                    and h >= 1.14*w):
                continue
            u = x0 + float(centres[label, 0]); v = y0 + float(centres[label, 1])
            # Vicino alla fila nominale, non ai comandi rossi del piano sopra.
            nearest = float(np.min(np.hypot((uv[:, 0]-u) / 1.8,
                                             uv[:, 1]-v)))
            if nearest > 75:
                continue
            components.append((u, v, label, lx, ly, w, h))

        components.sort(key=lambda item: item[0])
        expected = sorted(names, key=lambda name: projected[name][0])
        if len(components) < 5 or len(components) > 12:
            diagnostics = {name: f'componenti_rosse={len(components)}_non_5'
                           for name in names}
            return {}, diagnostics

        # Assegnazione monotona: le leve non possono scambiarsi di ordine.
        # Se ci sono componenti extra (stelo spezzato in 2), scegliamo la
        # cinquina piu' coerente con le posizioni previste.
        best = None
        for group in itertools.combinations(components, 5):
            errors = [((comp[0]-projected[name][0]) / 70.)**2 +
                      ((comp[1]-projected[name][1]) / 65.)**2
                      for name, comp in zip(expected, group)]
            if any(abs(comp[0]-projected[name][0]) > 75 or
                   abs(comp[1]-projected[name][1]) > 55
                   for name, comp in zip(expected, group)):
                continue
            score = sum(errors)
            if best is None or score < best[0]:
                best = (score, group)
        if best is None:
            return {}, diagnostics

        # Non deduciamo la profondita' da Xacro: campioniamo davvero il sensore.
        results = {}
        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])
        if fx <= 0 or fy <= 0:
            return {}, diagnostics

        for name, comp in zip(expected, best[1]):
            _, _, label, lx, ly, w, h = comp
            comp_mask = (labels[ly:ly+h, lx:lx+w] == label).astype(np.uint8)
            # Escludiamo bordi che mescolano rosso/pannello nella depth.
            inner = cv2.erode(comp_mask, np.ones((3, 3), np.uint8), iterations=1)
            if np.count_nonzero(inner) < 12:
                inner = comp_mask
            gy, gx = np.nonzero(inner)
            gx = gx + x0 + lx
            gy = gy + y0 + ly
            z = depth[gy, gx]
            ok = np.isfinite(z) & (z > 0.05) & (z < 1.00)
            if np.count_nonzero(ok) < 12:
                diagnostics[name] = 'pixel_rossi_senza_depth'
                continue
            gx, gy, z = gx[ok], gy[ok], z[ok].astype(np.float64)
            z_med = float(np.median(z))
            if np.median(np.abs(z-z_med)) > 0.012:
                diagnostics[name] = 'depth_leva_non_omogenea'
                continue

            # Test foreground/background: il rosso deve sporgere rispetto
            # al pannello circostante, altrimenti potremmo campionare il piano.
            full_x = x0 + lx; full_y = y0 + ly
            bx0 = max(0, full_x-9); bx1 = min(W, full_x+w+9)
            by0 = max(0, full_y-9); by1 = min(H, full_y+h+9)
            background = depth[by0:by1, bx0:bx1].astype(np.float64)
            surround_mask = np.ones(background.shape, dtype=bool)
            surround_mask[(full_y-by0):(full_y-by0+h),
                          (full_x-bx0):(full_x-bx0+w)] = False
            bg = background[surround_mask]
            bg = bg[np.isfinite(bg) & (bg > 0.05) & (bg < 1.00)]
            if bg.size < 20:
                diagnostics[name] = 'sfondo_depth_non_verificabile'
                continue
            bg_z = float(np.median(bg))
            relief = bg_z - z_med
            if not (0.006 <= relief <= 0.120):
                diagnostics[name] = ('nessuna_sporgenza_depth_rosso: '
                                     f'leva={z_med:.3f}m piano={bg_z:.3f}m')
                continue

            # L'immagine e la depth hanno stesso frame e dimensioni:
            # ogni pixel ROS/depth -> raggio ottico (con distorsione).
            px = np.column_stack([gx, gy]).astype(np.float64).reshape(-1, 1, 2)
            rays = cv2.undistortPoints(px, K, dist).reshape(-1, 2)
            xyz = np.column_stack((rays[:, 0]*z, rays[:, 1]*z, z))
            pos_cam = np.median(xyz, axis=0)
            if not np.all(np.isfinite(pos_cam)):
                continue
            centre = (int(np.median(gx)), int(np.median(gy)))
            results[name] = (pos_cam, z_med, centre)
            diagnostics[name] = (f'rosso+depth OK: delta_sfondo={relief*1000:.0f}mm')
        return results, diagnostics

    def measure_controls(self, R_camera_panel, p_camera_panel,
                         image_msg, image_shape, rgb_frame=None):
        """PnP -> ROI prevista, depth mediana -> 3D, TF -> base_link.

        Non usa una TF o una posizione globale del pannello come verita'.
        Se manca un pixel depth coerente, NON pubblica un target.
        """
        depth = self.depth_for_image(image_msg, image_shape)
        if depth is None:
            self._depth_diag_all('depth_assente_o_non_sincronizzata')
            return {}
        if self.camera_matrix is None:
            self._depth_diag_all('camera_info_assente')
            return {}

        frame_id = image_msg.header.frame_id or self.camera_frame
        if not frame_id:
            self._depth_diag_all('frame_camera_assente')
            return {}

        try:
            # L'istante dell'immagine conta: la camera e' sul braccio.
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame_id,
                Time.from_msg(image_msg.header.stamp),
                timeout=Duration(seconds=0.10))
        except TransformException as ex:
            # Il progetto usa un restamper dei JointState in Gazebo.
            # Dopo align_done il braccio e' fermo: in questo SOLO caso
            # la TF piu' recente e' un fallback sensato. Durante il
            # movimento NON bisogna mai fondere TF e depth di tempi diversi.
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, frame_id, Time(),
                    timeout=Duration(seconds=0.10))
                self.get_logger().warn(
                    'TF depth non disponibile al timestamp immagine; '
                    'uso latest TF SOLO perche il braccio e fermo. '
                    f'Dettaglio: {ex}', throttle_duration_sec=10.0)
            except TransformException as latest_ex:
                self.get_logger().warn(
                    f'Nessuna TF per depth: {latest_ex}',
                    throttle_duration_sec=5.0)
                self._depth_diag_all('tf_camera_base_assente')
                return {}

        rot = tf.transform.rotation
        R_base_camera = self.quaternion_to_matrix([
            rot.x, rot.y, rot.z, rot.w])
        p_base_camera = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z], dtype=np.float64)

        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        if fx <= 0 or fy <= 0:
            self._depth_diag_all('intrinseci_camera_non_validi')
            return {}
        radius = int(self.get_parameter('depth_roi_radius_px').value)
        radius = max(1, min(radius, 10))
        tolerance = max(0.01, float(
            self.get_parameter('depth_prediction_tolerance_m').value))
        h, w = depth.shape
        measurements = {}
        lower_names = ('main_switch', 'lever_switch_1', 'lever_switch_2',
                       'lever_switch_3', 'lever_switch_4')
        rgb_red_positions = {}
        rgb_red_diagnostics = {}
        if rgb_frame is not None:
            # PnP/camera servono SOLO a delimitare la zona RGB da esaminare.
            # La coordinata 3D restera' una misura depth, mai quella nominale.
            try:
                lower_points = np.asarray([self.control_points[n] for n in lower_names],
                                          dtype=np.float64)
                px, _ = cv2.projectPoints(
                    lower_points, cv2.Rodrigues(R_camera_panel)[0],
                    p_camera_panel.reshape(3, 1),
                    self.camera_matrix, self.dist_coeffs)
                projections = {name: tuple(px[i, 0]) for i, name in enumerate(lower_names)}
                rgb_red_positions, rgb_red_diagnostics = self.find_red_lever_depth(
                    rgb_frame, depth, projections,
                    self.camera_matrix, self.dist_coeffs)
            except (cv2.error, ValueError, IndexError) as exc:
                rgb_red_diagnostics = {name: f'errore_roi_rgb: {exc}'
                                       for name in lower_names}

        for name, point_panel in self.control_points.items():
            if name in lower_names and rgb_frame is not None:
                # Leve: SOLO pixel del vero oggetto rosso, mai il pannello
                # grigio campionato da una crocetta nominale spostata.
                candidate = rgb_red_positions.get(name)
                if candidate is None:
                    self._depth_diag(name, rgb_red_diagnostics.get(
                        name, 'leva_rossa_non_misurata'))
                    continue
                point_camera, z, uv = candidate
                p_base = R_base_camera @ point_camera + p_base_camera
                measurements[name] = (p_base, z, uv)
                self._depth_diag(name, 'misurato_rgb_depth', pixel=uv,
                                 z_observed=z, raw_valid=1, matched=1)
                continue

            point_cam_expected = R_camera_panel @ point_panel + p_camera_panel
            if point_cam_expected[2] <= 0.05:
                self._depth_diag(name, 'punto_dietro_camera',
                                 z_expected=float(point_cam_expected[2]))
                continue
            u = fx * point_cam_expected[0] / point_cam_expected[2] + cx
            v = fy * point_cam_expected[1] / point_cam_expected[2] + cy
            ui, vi = int(round(u)), int(round(v))
            if not (radius <= ui < w-radius and radius <= vi < h-radius):
                self._depth_diag(name, 'roi_fuori_immagine',
                                 pixel=(ui, vi),
                                 z_expected=float(point_cam_expected[2]))
                continue

            # Calibrazione con distorsione: per il pixel previsto usiamo
            # projectPoints, non la proiezione pinhole non distorta.
            pixel, _ = cv2.projectPoints(
                point_panel.reshape(1, 3).astype(np.float64),
                cv2.Rodrigues(R_camera_panel)[0],
                p_camera_panel.reshape(3, 1),
                self.camera_matrix, self.dist_coeffs)
            ui, vi = [int(round(float(vv))) for vv in pixel.reshape(2)]
            if not (radius <= ui < w-radius and radius <= vi < h-radius):
                self._depth_diag(name, 'roi_fuori_immagine_distorta',
                                 pixel=(ui, vi),
                                 z_expected=float(point_cam_expected[2]))
                continue

            roi = depth[vi-radius:vi+radius+1, ui-radius:ui+radius+1]
            raw_good = np.isfinite(roi) & (roi >= 0.05) & (roi <= 1.0)
            raw_n = int(np.count_nonzero(raw_good))
            raw_med = float(np.median(roi[raw_good])) if raw_n else None
            # ATTENZIONE: filtro originale INVARIATO, solo conteggi aggiunti.
            good = raw_good.copy()
            good &= np.abs(roi - point_cam_expected[2]) <= tolerance
            matched_n = int(np.count_nonzero(good))
            if matched_n < 3:
                self._depth_diag(
                    name,
                    'depth_invalida_nella_roi' if raw_n < 3
                    else 'depth_differente_dalla_previsione',
                    pixel=(ui, vi),
                    z_expected=float(point_cam_expected[2]),
                    z_observed=raw_med, raw_valid=raw_n,
                    matched=matched_n)
                continue
            z = float(np.median(roi[good]))
            self._depth_diag(name, 'misurato', pixel=(ui, vi),
                             z_expected=float(point_cam_expected[2]),
                             z_observed=z, raw_valid=raw_n,
                             matched=matched_n)
            # I K di CameraInfo assumono pixel rettificati, come nel
            # sensore Gazebo virtuale. Sulla Gemini REALE servira'
            # usare immagini/CameraInfo rettificate e depth registrata.
            point_camera = np.array([
                (ui - cx) * z / fx,
                (vi - cy) * z / fy,
                z], dtype=np.float64)
            p_base = R_base_camera @ point_camera + p_base_camera
            measurements[name] = (p_base, z, (ui, vi))
        return measurements

    def cached_panel_pose_in_camera(self, image_msg):
        """Ri-proietta la posa ArUco salvata in base_link nella camera attuale.

        Si usa solo dopo /panel/align_done (braccio fermo). In questo modo
        la camera puo' vedere i pulsanti anche se gli ArUco sono fuori FOV.
        """
        if not self.align_done or self.cached_panel_pose_base is None:
            return None
        camera_frame = image_msg.header.frame_id or self.camera_frame
        if not camera_frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, camera_frame,
                Time.from_msg(image_msg.header.stamp),
                timeout=Duration(seconds=0.10))
        except TransformException:
            try:
                # Ammesso soltanto a braccio fermo: stessa ipotesi del
                # measure_controls() originale.
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, camera_frame, Time(),
                    timeout=Duration(seconds=0.10))
            except TransformException as exc:
                self.get_logger().warn(
                    f'DEPTH: TF camera->base_link assente: {exc}',
                    throttle_duration_sec=5.0)
                return None

        rot = tf.transform.rotation
        R_bc = self.quaternion_to_matrix([rot.x, rot.y, rot.z, rot.w])
        t_bc = np.array([tf.transform.translation.x,
                         tf.transform.translation.y,
                         tf.transform.translation.z], dtype=np.float64)
        R_bp, t_bp = self.cached_panel_pose_base
        R_cp = R_bc.T @ R_bp
        t_cp = R_bc.T @ (t_bp - t_bc)
        if not np.all(np.isfinite(t_cp)) or t_cp[2] <= 0.05:
            self.get_logger().warn(
                'DEPTH: posa del pannello proiettata dietro la camera.',
                throttle_duration_sec=5.0)
            return None
        return R_cp, t_cp

    def accumulate_controls(self, measurements, R_panel, stamp):
        min_samples = max(2, int(
            self.get_parameter('min_control_samples').value))
        for name, (position, depth_m, pixel) in measurements.items():
            history = self.control_samples[name]
            history.append(position)
            if len(history) < min_samples:
                continue
            points = np.asarray(history)
            med = np.median(points, axis=0)
            # Non pubblicare se i campioni sono molto discordanti.
            scatter = float(np.median(np.linalg.norm(points - med, axis=1)))
            if scatter > 0.02:
                continue
            self.control_publishers[name].publish(
                self.make_pose(med, R_panel, stamp, self.base_frame))
            if name not in self.control_logged:
                self.control_logged.add(name)
                self.get_logger().info(
                    f'DEPTH {name}: base_link='
                    f'({med[0]:.3f}, {med[1]:.3f}, {med[2]:.3f}) m; '
                    f'Z_camera={depth_m:.3f} m; pixel={pixel}; '
                    f'campioni={len(history)}')

        # Solo visualizzazione: nessuna modifica alla validazione / ai topic pose.
        self.publish_rviz_controls()

    # ============================================
    # MARKER RVIZ: VISUALIZZAZIONE COORDINATE VALIDATE DEPTH
    # ============================================

    def publish_rviz_controls(self):
        """Sfere e brevi etichette per sole misure depth valide in base_link.

        Non si usano mai i punti nominali control_points come coordinate.
        I marker NON rappresentano pose di contatto e non muovono il robot.
        """
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        # Evita pubblicazioni troppo frequenti durante lo streaming RGB-D.
        if now_s - self.rviz_marker_last_s < 0.5:
            return
        self.rviz_marker_last_s = now_s
        valid_count = 0
        min_samples = max(2, int(
            self.get_parameter('min_control_samples').value))
        result = MarkerArray()
        short_names = {
            'main_switch': 'M',
            **{f'lever_switch_{i}': f'L{i}' for i in range(1, 5)},
            **{f'rotary_switch_{i}': f'R{i}' for i in range(1, 6)},
            **{f'power_switch_{i}': f'P{i}' for i in range(1, 3)},
        }
        for i, name in enumerate(self.control_points):
            history = self.control_samples[name]
            position = None
            if len(history) >= min_samples:
                measured = np.asarray(history, dtype=np.float64)
                median = np.median(measured, axis=0)
                scatter = float(np.median(np.linalg.norm(measured-median, axis=1)))
                if (np.all(np.isfinite(median)) and np.isfinite(scatter)
                        and scatter <= 0.020):
                    position = median
            if name.startswith('lever_') or name == 'main_switch':
                rgba = (1.0, 0.3, 0.25, 1.0)  # rosso: leve/main
            elif name.startswith('rotary_'):
                rgba = (0.15, 0.85, 0.95, 1.0)  # turchese: rotary
            else:
                rgba = (0.95, 0.85, 0.18, 1.0)  # giallo: power

            for is_label in (False, True):
                marker = Marker()
                marker.header.frame_id = self.base_frame  # base_link
                marker.header.stamp = now.to_msg()
                marker.ns = 'depth_controls_labels' if is_label else 'depth_controls_points'
                marker.id = i
                # Cancella un vecchio punto se i campioni diventano inaffidabili.
                if position is None:
                    marker.action = Marker.DELETE
                    result.markers.append(marker)
                    continue
                marker.action = Marker.ADD
                marker.type = Marker.TEXT_VIEW_FACING if is_label else Marker.SPHERE
                marker.pose.position.x = float(position[0])
                marker.pose.position.y = float(position[1])
                marker.pose.position.z = float(position[2] + (0.020 if is_label else 0.0))
                marker.pose.orientation.w = 1.0
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
                marker.scale.x = 0.012
                marker.scale.y = 0.012
                marker.scale.z = 0.017 if is_label else 0.012
                if is_label:
                    marker.text = short_names.get(name, name)
                result.markers.append(marker)
            if position is not None:
                valid_count += 1

        # Un solo topic contenente sfere + etichette, latched per RViz.
        self.rviz_controls_pub.publish(result)
        if valid_count > 0 and not self.rviz_first_log:
            self.rviz_first_log = True
            self.get_logger().info(
                f'RViz: pubblicati {valid_count}/{len(self.control_points)} punti '
                'misurati su /panel/controls/markers (MarkerArray, frame base_link).')

    # ============================================
    # REPORT CLI DELLE COORDINATE MISURATE CON DEPTH
    # ============================================

    def log_depth_positions(self):
        """Stampa le sole misure 3D valide, senza inventare target mancanti.

        NON invia comandi all'arm_controller. Usa gli stessi campioni gia'
        verificati da accumulate_controls(), con la stessa soglia scatter.
        Le ROI in immagine sono individuate dai centri nominali del pannello;
        la profondita' e la posizione 3D provengono dai pixel depth misurati.
        """
        pose = self.final_pose if self.final_pose is not None else self.cached_panel_pose_base
        if pose is None:
            self.get_logger().warn(
                'DEPTH: nessuna posa PnP valida del pannello in base_link; '
                'non posso convertire i punti nel frame panel_base.')
            return

        R_panel, p_panel = pose
        min_samples = max(2, int(self.get_parameter('min_control_samples').value))
        lines = [
            '',
            '========== POSIZIONI COMANDI DA CAMERA DEPTH ==========',
            'Unita: metri | base_link: X Y Z | panel_base: X Y Z',
            'Punti misurati via depth: ROI nominali per selettori, RGB per le 5 leve.',
            'NON sono pose di contatto validate per la pressione dei pulsanti.',
            f'Validazione: >= {min_samples} campioni; dispersione mediana <= 0.020 m',
            '-------------------------------------------------------',
        ]
        valid_count = 0
        for name in self.control_points:
            history = self.control_samples[name]
            n = len(history)
            if n < min_samples:
                lines.append(f'{name:<19} NON MISURATO: campioni {n}/{min_samples}')
                continue

            points = np.asarray(history, dtype=np.float64)
            p_base = np.median(points, axis=0)
            scatter = float(np.median(np.linalg.norm(points - p_base, axis=1)))
            if not np.all(np.isfinite(p_base)) or not np.isfinite(scatter) or scatter > 0.020:
                lines.append(f'{name:<19} NON AFFIDABILE: dispersione {scatter*1000:.1f} mm')
                continue

            p_local = R_panel.T @ (p_base - p_panel)
            valid_count += 1
            lines.append(
                f'{name:<19} '
                f'base=({p_base[0]:+.3f}, {p_base[1]:+.3f}, {p_base[2]:+.3f})  '
                f'panel=({p_local[0]:+.3f}, {p_local[1]:+.3f}, {p_local[2]:+.3f})  '
                f'N={n} scatter={scatter*1000:.1f}mm'
            )

        lines.extend([
            '-------------------------------------------------------',
            'DIAGNOSTICA ROI DEPTH (7 selettori originali; 5 leve RGB-guided):',
        ])
        for name in self.control_points:
            counts = self.depth_roi_status_counts[name]
            if not counts:
                lines.append(f'{name:<19} nessun tentativo di misura')
                continue
            common = ', '.join(f'{reason}={n}'
                               for reason, n in counts.most_common(3))
            last = self.depth_roi_last.get(name, {})
            px = last.get('pixel')
            z_exp = last.get('z_expected')
            z_obs = last.get('z_observed')
            px_s = str(px) if px is not None else '-'
            exp_s = f'{z_exp:.3f}' if z_exp is not None else '-'
            obs_s = f'{z_obs:.3f}' if z_obs is not None else '-'
            lines.append(
                f'{name:<19} esiti: {common}; '
                f'ultimo_pixel={px_s}; z_prevista={exp_s}m; '
                f'z_depth_grezza={obs_s}m'
            )
        lines.extend([
            '-------------------------------------------------------',
            f'VALIDI: {valid_count}/{len(self.control_points)}',
            'Ogni comando valido continua a essere pubblicato su:',
            '  /panel/controls/<nome>/pose_base  (geometry_msgs/PoseStamped)',
            '=======================================================',
        ])
        self.get_logger().info('\n'.join(lines))
        if valid_count == 0:
            self.get_logger().warn(
                'DEPTH: nessun comando misurato. Verificare /camera/rgbd/depth_image, '
                'timestamp RGB/depth, TF ottica->base_link e inquadratura dei pulsanti.')

    def camera_info_callback(self, msg):

        self.camera_matrix = np.array(
            msg.k, dtype=np.float64
        ).reshape(3, 3)

        self.dist_coeffs = np.array(
            msg.d, dtype=np.float64
        )

        self.camera_frame = msg.header.frame_id

    # ============================================
    # SEGNALE DAL C++: BRACCIO FERMO, STIMA FINALE
    # ============================================

    def reobserve_callback(self, msg):
        """Fresh perception window. Old medians cannot masquerade as new depth."""
        if not msg.data or not self.align_done:
            return
        for history in self.control_samples.values():
            history.clear()
        self.control_logged.clear()
        self.depth_cache.clear()
        self.reobserve_started_ns = self.get_clock().now().nanoseconds
        self.reobserve_ready_pub.publish(Bool(data=True))
        self.get_logger().info(
            'RIOSSERVAZIONE richiesta: cache depth e storici controlli azzerati; '
            'attendo nuovi frame RGB-D sincronizzati.')

    def align_done_callback(self, msg):

        if msg.data and not self.align_done:

            self.align_done = True
            self.align_done_time = self.get_clock().now().nanoseconds / 1e9
            self.final_samples = []
            for history in self.control_samples.values():
                history.clear()
            self.control_logged.clear()
            self.depth_frames_since_align = 0
            self.depth_reported = False
            self.depth_roi_status_counts = {
                name: Counter() for name in self.control_points
            }
            self.depth_roi_last.clear()
            if self.cached_panel_pose_base is None:
                self.get_logger().warn(
                    'DEPTH: non ho ancora salvato una posa PnP valida del pannello.')

            self.get_logger().info(
                'Braccio fermo: raccolgo '
                f'{self.n_final_samples} frame per la stima finale'
            )

    # ============================================
    # ROTAZIONI E QUATERNION
    # ============================================

    def matrix_to_quaternion(self, R):

        q = np.empty(4, dtype=np.float64)
        trace = np.trace(R)

        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2.0
            q[3] = 0.25 * s
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = (R[0, 2] - R[2, 0]) / s
            q[2] = (R[1, 0] - R[0, 1]) / s

        else:
            i = int(np.argmax(np.diag(R)))
            j = (i + 1) % 3
            k = (i + 2) % 3

            s = np.sqrt(
                1.0 + R[i, i] - R[j, j] - R[k, k]
            ) * 2.0

            q[i] = 0.25 * s
            q[3] = (R[k, j] - R[j, k]) / s
            q[j] = (R[j, i] + R[i, j]) / s
            q[k] = (R[k, i] + R[i, k]) / s

        return q / np.linalg.norm(q)

    def quaternion_to_matrix(self, q):

        x, y, z, w = np.array(q) / np.linalg.norm(q)

        return np.array([
            [
                1 - 2*(y*y + z*z),
                2*(x*y - z*w),
                2*(x*z + y*w)
            ],
            [
                2*(x*y + z*w),
                1 - 2*(x*x + z*z),
                2*(y*z - x*w)
            ],
            [
                2*(x*z - y*w),
                2*(y*z + x*w),
                1 - 2*(x*x + y*y)
            ]
        ])

    # ============================================
    # COSTRUZIONE POSESTAMPED
    # ============================================

    def make_pose(self, position, rotation, stamp, frame):
        # L'header viene costruito da zero (stamp copiato campo per campo):
        # cosi' non si condivide mai l'oggetto header con il messaggio
        # immagine e non lo si modifica per sbaglio.

        msg = PoseStamped()
        msg.header.stamp.sec = stamp.sec
        msg.header.stamp.nanosec = stamp.nanosec
        msg.header.frame_id = frame

        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])

        q = self.matrix_to_quaternion(rotation)

        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])

        return msg

    # ============================================
    # TRASFORMAZIONE CAMERA -> ROVER
    # ============================================

    def transform_to_base(self, R_camera_panel, p_camera_panel, camera_frame):
        # Ritorna (R_base_panel, p_base_panel) oppure None.

        try:
            # Ultima trasformazione disponibile (workaround timestamp)
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                camera_frame,
                Time(),
                timeout=Duration(seconds=0.5)
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'TF2 non disponibile: {ex}',
                throttle_duration_sec=2.0
            )
            return None

        qt = tf.transform.rotation

        R_base_camera = self.quaternion_to_matrix([
            qt.x, qt.y, qt.z, qt.w
        ])

        t_base_camera = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z
        ])

        # B_T_P = B_T_C * C_T_P
        p_base_panel = R_base_camera @ p_camera_panel + t_base_camera
        R_base_panel = R_base_camera @ R_camera_panel

        return R_base_panel, p_base_panel

    # ============================================
    # DISTANZE ROVER -> PANNELLO (in base_link)
    # ============================================

    def compute_distances(self, R, p):

        normal = R[:, 1]    # normale uscente della faccia (+y pannello)

        # distanza perpendicolare dall'origine di base_link al piano
        # della faccia frontale del pannello
        d_face = abs(float(np.dot(normal, p)))

        # centro della faccia frontale in base_link
        c_face = R @ self.panel_face_center + p

        d_center_xy = float(np.linalg.norm(c_face[:2]))
        d_center_3d = float(np.linalg.norm(c_face))

        # rotazione del pannello rispetto al rover: 0 = la faccia guarda
        # esattamente verso il rover (normale = -x di base_link)
        yaw_deg = math.degrees(math.atan2(-normal[1], -normal[0]))

        return d_face, c_face, d_center_xy, d_center_3d, yaw_deg

    # ============================================
    # PUBBLICAZIONE POSA E DISTANZA
    # ============================================

    def publish_pose_and_distance(self, R, p, stamp):

        self.pose_base_pub.publish(
            self.make_pose(p, R, stamp, self.base_frame)
        )

        d_face = self.compute_distances(R, p)[0]

        self.distance_pub.publish(Float32(data=float(d_face)))

    # ============================================
    # STIMA FINALE (mediana su N frame a braccio fermo)
    # ============================================

    def finalize_estimate(self, last_error):

        samples = self.final_samples

        ps = np.array([sm['p'] for sm in samples])
        p_med = np.median(ps, axis=0)

        corrected = all(sm['corrected'] for sm in samples)

        if corrected:
            # yaw: mediana circolare, poi ricostruisco la rotazione verticale
            psis = np.array([sm['psi'] for sm in samples])
            psi = math.atan2(
                float(np.median(np.sin(psis))),
                float(np.median(np.cos(psis)))
            )
            R_med = np.array([
                [math.cos(psi), -math.sin(psi), 0.0],
                [math.sin(psi),  math.cos(psi), 0.0],
                [0.0, 0.0, 1.0]
            ])
        else:
            R_med = samples[-1]['R']

        tilt_med = float(np.median([sm['tilt'] for sm in samples]))
        err_med = float(np.median([sm['error'] for sm in samples]))
        spread_mm = np.std(ps, axis=0) * 1000.0

        self.final_pose = (R_med, p_med)
        self.final_done = True
        
        # Pubblica la posa finale soltanto dopo i 15 campioni
        final_msg = self.make_pose(
           p_med,
           R_med,
           self.get_clock().now().to_msg(),
           self.base_frame
        )

        self.final_pose_pub.publish(final_msg)

        self.get_logger().info(
            'Posa finale pubblicata su /panel/final_pose: '
            'avvicinamento rover abilitato'
        )

        (d_face, c_face, d_center_xy,
         d_center_3d, yaw_deg) = self.compute_distances(R_med, p_med)

        self.final_distance = d_face

        c0 = R_med @ self.marker_positions[0] + p_med
        c1 = R_med @ self.marker_positions[1] + p_med

        if corrected:
            nota = (
                f'pannello verticale imposto '
                f'(inclinazione PnP scartata: {tilt_med:.1f} gradi)'
            )
        else:
            nota = (
                f'posa grezza PnP '
                f'(inclinazione rispetto alla verticale: {tilt_med:.1f} gradi)'
            )

        self.get_logger().info(
            '\n'
            f'===== STIMA FINALE PANNELLO RISPETTO AL ROVER ({self.base_frame}) =====\n'
            f'{nota}\n'
            f'mediana su {len(samples)} frame, braccio fermo; '
            f'dispersione origine (std): '
            f'x={spread_mm[0]:.1f} y={spread_mm[1]:.1f} z={spread_mm[2]:.1f} mm\n'
            f'panel_base origin: '
            f'({p_med[0]:.3f}, {p_med[1]:.3f}, {p_med[2]:.3f}) m\n'
            f'marker 0 centro:   '
            f'({c0[0]:.3f}, {c0[1]:.3f}, {c0[2]:.3f}) m\n'
            f'marker 1 centro:   '
            f'({c1[0]:.3f}, {c1[1]:.3f}, {c1[2]:.3f}) m\n'
            f'DISTANZA dalla faccia del pannello '
            f'(perpendicolare): {d_face:.3f} m\n'
            f'distanza dal centro della faccia: '
            f'orizzontale {d_center_xy:.3f} m, 3D {d_center_3d:.3f} m\n'
            f'centro faccia in {self.base_frame}: '
            f'({c_face[0]:.3f}, {c_face[1]:.3f}, {c_face[2]:.3f}) m\n'
            f'rotazione pannello rispetto al rover (yaw): {yaw_deg:.1f} gradi\n'
            f'Errore riproiezione (mediana) = {err_med:.2f} px'
        )

        # Solo reporting: non modifica detection, pose o traiettorie del braccio.
        self.log_depth_positions()

    # ============================================
    # STIMA PNP (prova le 4 rotazioni degli angoli)
    # ============================================

    def estimate_panel_pose(self, detected):

        image_points = np.vstack([detected[0], detected[1]])

        best = None

        for shift, pts in self.candidate_object_points.items():

            object_points = np.vstack([pts[0], pts[1]])

            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if not success:
                continue

            projected, _ = cv2.projectPoints(
                object_points,
                rvec,
                tvec,
                self.camera_matrix,
                self.dist_coeffs
            )

            projected = projected.reshape(-1, 2)

            error = float(np.sqrt(np.mean(
                np.sum((projected - image_points) ** 2, axis=1)
            )))

            R, _ = cv2.Rodrigues(rvec)
            t = tvec.flatten()

            # La normale del pannello (+y) deve puntare VERSO la camera.
            # Se non lo fa, PnP ha trovato la soluzione "vista da dietro"
            # (pannello capovolto): errore di riproiezione basso ma
            # posa sbagliata.
            marker_center_cam = R @ self.marker_positions[0] + t
            normal_cam = R @ self.panel_normal
            facing = float(np.dot(normal_cam, -marker_center_cam)) > 0.0

            self.get_logger().info(
                f'[candidato shift={shift}] err={error:.2f}px '
                f'z={t[2]:.3f} normale_verso_camera={facing}',
                throttle_duration_sec=2.0
            )

            if t[2] <= 0.0 or not facing:
                continue

            if best is None or error < best['error']:
                best = {
                    'shift': shift,
                    'error': error,
                    'rvec': rvec,
                    'tvec': tvec,
                    'R': R
                }

        return best

    # ============================================
    # RILEVAMENTO MARKER (compatibile con le due API di OpenCV)
    # ============================================

    def detect_markers(self, frame):

        if self.aruco_detector is not None:
            return self.aruco_detector.detectMarkers(frame)

        return cv2.aruco.detectMarkers(
            frame,
            self.dictionary,
            parameters=self.parameters
        )

    # ============================================
    # CALLBACK IMMAGINE
    # ============================================

    def image_callback(self, msg):
        if (self.reobserve_started_ns is not None
                and self.stamp_ns(msg.header.stamp) <= self.reobserve_started_ns):
            return

        # Guardia anti-backlog: se il frame RGB e' vecchio rispetto al clock
        # di simulazione, non puo' essere accoppiato a una depth corrente.
        # Meglio scartarlo subito (la coda BEST_EFFORT consegnera' il prossimo)
        # che consumare secondi di coda e misurare su immagini di minuti fa.
        now_ns = self.get_clock().now().nanoseconds
        frame_age = (now_ns - self.stamp_ns(msg.header.stamp)) * 1e-9
        if now_ns > 0 and frame_age > MAX_RGB_FRAME_AGE_S:
            if not self.frame_age_logged:
                self.frame_age_logged = True
                self.get_logger().warn(
                    f'RGB in ritardo di {frame_age:.1f}s sul clock di simulazione: '
                    'frame scartato (backlog). Le depth restano correnti.')
            return

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        corners, ids, _ = self.detect_markers(frame)

        detected = {}

        if ids is not None:

            cv2.aruco.drawDetectedMarkers(
                frame, corners, ids
            )

            for i, marker_id in enumerate(ids.flatten()):

                marker_id = int(marker_id)

                if marker_id in (0, 1):
                    detected[marker_id] = (
                        corners[i][0].astype(np.float64)
                    )

        both_found = (
            0 in detected and 1 in detected
        )
        valid_panel_pose_this_frame = False

        # In ricerca con un marker: inseguiamo il marker osservato.
        # Con entrambi visibili: dopo il PnP pubblichiamo invece l'errore
        # del CENTRO DEL PANNELLO, non il centro della fascia ArUco in alto.
        # Cosi' il centraggio finale abbassa davvero la visuale sul pannello.
        if detected and not both_found:
            height, width = frame.shape[:2]
            pts = np.vstack(list(detected.values()))
            u, v = pts.mean(axis=0)
            self.image_error_pub.publish(
                Float32MultiArray(data=[
                    float((u - width / 2.0) / (width / 2.0)),
                    float((v - height / 2.0) / (height / 2.0)),
                    1.0
                ])
            )

        # ========================================
        # STOP BRACCIO
        # ========================================

        if both_found and not self.marker_found:

            self.marker_found = True

            self.get_logger().info( # qua viene detto al python di pubblicare il messaggio sul topic su cui il cpp è in ascolto
                'Entrambi gli ArUco trovati: STOP SCANSIONE'
            )

        self.marker_pub.publish(
            Bool(data=self.marker_found)
        )

        if self.marker_found:
            cv2.putText(
                frame,
                'ARUCO TROVATO - STOP RICERCA',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        if self.camera_matrix is None:
            if self.enable_gui:
                cv2.imshow('ArUco Detection', frame)
                cv2.waitKey(1)
            return

        # ========================================
        # STIMA PANNELLO CON 8 ANGOLI
        # ========================================

        if both_found:

            best = self.estimate_panel_pose(detected)

            if best is not None and best['error'] <= 3.0:

                rvec = best['rvec']
                tvec = best['tvec']
                R_camera_panel = best['R']

                # Centro della faccia, proiettato dal PnP appena stimato.
                # NON stiamo usando una posa globale nota del pannello:
                # l'unico dato a priori e' la geometria locale del pannello.
                panel_uv, _ = cv2.projectPoints(
                    self.panel_face_center.reshape(1, 3),
                    rvec, tvec, self.camera_matrix, self.dist_coeffs)
                u_center, v_center = panel_uv.reshape(-1, 2)[0]
                h_img, w_img = frame.shape[:2]
                if np.isfinite(u_center) and np.isfinite(v_center):
                    self.image_error_pub.publish(Float32MultiArray(data=[
                        float((u_center - w_img / 2.0) / (w_img / 2.0)),
                        float((v_center - h_img / 2.0) / (h_img / 2.0)),
                        2.0
                    ]))

                marker0_camera = (
                    R_camera_panel @ self.marker_positions[0]
                    + tvec.flatten()
                )

                self.get_logger().info(
                    f'MARKER 0 RISPETTO ALLA CAMERA (OpenCV): '
                    f'x={marker0_camera[0]:.3f}, '
                    f'y={marker0_camera[1]:.3f}, '
                    f'z={marker0_camera[2]:.3f}',
                    throttle_duration_sec=1.0
                )

                self.pose_camera_pub.publish(
                    self.make_pose(
                        tvec.flatten(),
                        R_camera_panel,
                        msg.header.stamp,
                        msg.header.frame_id
                    )
                )

                result = self.transform_to_base(
                    R_camera_panel,
                    tvec.flatten(),
                    msg.header.frame_id
                )

                if result is not None:

                    R_raw, p_raw = result

                    # inclinazione dell'asse z del pannello stimata da PnP
                    # rispetto alla verticale (serve solo come diagnostica)
                    tilt_deg = math.degrees(math.acos(
                        max(-1.0, min(1.0, float(R_raw[2, 2])))
                    ))

                    R_b, p_b = R_raw, p_raw
                    corrected = False

                    if self.assume_vertical_panel:
                        fixed = enforce_vertical_panel(
                            R_raw, p_raw, self.marker_positions
                        )
                        if fixed is not None:
                            R_b, p_b = fixed
                            corrected = True

                    # Salva posa PnP PRIMA che gli ArUco escano dal campo
                    # visivo. Il rover resta fermo; TF gestisce la camera
                    # che si inclina, senza tornare a cercare i marker.
                    if (np.all(np.isfinite(R_b)) and
                            np.all(np.isfinite(p_b))):
                        self.cached_panel_pose_base = (
                            np.array(R_b, copy=True),
                            np.array(p_b, copy=True))
                        valid_panel_pose_this_frame = True

                    # Solo dopo lo STOP del C++ usiamo la depth per stimare
                    # punti 3D dei controlli: durante il moto evitano dati
                    # temporalmente incoerenti.
                    if self.align_done:
                        controls = self.measure_controls(
                            R_camera_panel,
                            tvec.flatten(), msg, frame.shape, frame)
                        if controls:
                            self.accumulate_controls(
                                controls, R_b, msg.header.stamp)
                            # Solo i target con una misura depth coerente
                            # ricevono un cerchio verde nell'immagine RGB.
                            if self.enable_gui:
                                for label, (_, z_depth, uv) in controls.items():
                                    cv2.circle(frame, uv, 4, (0, 255, 0), 2)
                                    cv2.putText(
                                        frame, label.replace('_switch', '')[:9],
                                        (uv[0]+5, uv[1]-4),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                                        (0, 255, 0), 1)
                        else:
                            self.get_logger().warn(
                                'DEPTH: nessun comando misurabile in questo frame '
                                '(ROI fuori immagine, depth mancante/non allineata, '
                                'o TF non disponibile).',
                                throttle_duration_sec=5.0)

                    sample = {
                        'R': R_b,
                        'p': p_b,
                        'psi': math.atan2(R_b[1, 0], R_b[0, 0]),
                        'tilt': tilt_deg,
                        'error': best['error'],
                        'corrected': corrected
                    }

                    if self.final_done:

                        # stima finale gia' fatta: continuo a ripubblicarla
                        self.publish_pose_and_distance(
                            self.final_pose[0],
                            self.final_pose[1],
                            msg.header.stamp
                        )

                    elif not self.align_done:

                        # braccio ancora in movimento: posa provvisoria
                        # (solo topic e un log raro)
                        d_face, _, _, _, _ = self.compute_distances(R_b, p_b)

                        self.publish_pose_and_distance(
                            R_b, p_b, msg.header.stamp
                        )

                        now = self.get_clock().now().nanoseconds / 1e9

                        if (
                            self.last_log_time < 0
                            or now - self.last_log_time >= 3.0
                        ):
                            self.last_log_time = now

                            self.get_logger().info(
                                f'Stima provvisoria (braccio in movimento): '
                                f'distanza faccia {d_face:.3f} m'
                            )

                    else:

                        # braccio fermo: raccolgo i campioni
                        self.final_samples.append(sample)

                        if len(self.final_samples) >= self.n_final_samples:
                            self.finalize_estimate(best['error'])

                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    0.12
                )

            else:

                self.get_logger().warn(
                    'Posa pannello scartata: nessun candidato valido '
                    '(errore alto o normale non rivolta alla camera)',
                    throttle_duration_sec=2.0
                )

        # Depth-only: nessun nuovo PnP richiesto se i marker sono usciti
        # dalla camera. Usare solo la posa valida salvata da ArUco + TF
        # aggiornata a braccio fermo. La detection originale e' intatta.
        if self.align_done and not valid_panel_pose_this_frame:
            cached_camera_pose = self.cached_panel_pose_in_camera(msg)
            if cached_camera_pose is not None:
                R_cp, t_cp = cached_camera_pose
                controls = self.measure_controls(
                    R_cp, t_cp, msg, frame.shape, frame)
                if controls:
                    R_bp, _ = self.cached_panel_pose_base
                    self.accumulate_controls(controls, R_bp, msg.header.stamp)
                    if self.enable_gui:
                        for label, (_, _, uv) in controls.items():
                            cv2.circle(frame, uv, 4, (0, 255, 0), 2)
                else:
                    _diag = {n: dict(c) for n, c in self.depth_roi_status_counts.items()
                             if n in ('main_switch', 'lever_switch_1')}
                    _last = {n: self.depth_roi_last.get(n)
                             for n in ('main_switch', 'lever_switch_1')}
                    self.get_logger().warn(
                        'DEPTH: nessun pulsante misurabile dalla posa attuale '
                        '(fuori FOV, profondita non valida o mancata sincronizzazione). '
                        f'Diag={_diag} Last={_last}',
                        throttle_duration_sec=5.0)

        # Resoconto indipendente dai 15 frame ArUco finali: servono solo
        # misure depth a braccio fermo, anche se i marker sono fuori FOV.
        if self.align_done and not self.depth_reported:
            self.depth_frames_since_align += 1
            if self.final_done:
                self.depth_reported = True  # finalize_estimate() stampa gia'
            elif self.depth_frames_since_align >= 40:
                self.log_depth_positions()
                self.depth_reported = True

        if self.align_done and not self.final_done:

            now = self.get_clock().now().nanoseconds / 1e9

            if now - self.align_done_time > 15.0:

                # Se i marker sono usciti dal FOV nella posa finale, non ci
                # sono campioni: si ripiega sull'ultima posa valida misurata
                # prima dello stop (la stessa usata dalla misura depth).
                if self.cached_panel_pose_base is not None:
                    R_cache, p_cache = self.cached_panel_pose_base
                    self.final_pose = (R_cache, p_cache)
                    self.final_done = True
                    self.final_pose_pub.publish(self.make_pose(
                        p_cache, R_cache,
                        self.get_clock().now().to_msg(),
                        self.base_frame))
                    self.get_logger().info(
                        'Stima finale: marker fuori FOV dopo lo stop, '
                        'pubblico l\'ultima posa pannello valida (pre-stop).')
                else:
                    self.get_logger().warn(
                        'Stima finale in attesa: nessuna posa valida '
                        '(marker non visibili o scartati?)',
                        throttle_duration_sec=5.0
                    )

        # Crocette MAGENTA = centro nominale della ROI di misura depth.
        # Solo dopo lo stop e senza alcun effetto sulla detection ArUco.
        if self.align_done and self.enable_gui:
            img_h, img_w = frame.shape[:2]
            for name in ('main_switch', 'lever_switch_1',
                         'lever_switch_2', 'lever_switch_3', 'lever_switch_4'):
                diag = self.depth_roi_last.get(name, {})
                uv = diag.get('pixel')
                if uv is not None and 0 <= uv[0] < img_w and 0 <= uv[1] < img_h:
                    cv2.drawMarker(frame, uv, (255, 0, 255),
                                   cv2.MARKER_CROSS, 18, 2)
                    cv2.putText(frame, name, (uv[0]+8, uv[1]-8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                                (255, 0, 255), 1)

        if self.enable_gui:
            cv2.imshow('ArUco Detection', frame)
            cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = ArucoDetector()

    # MultiThreadedExecutor: il listener TF (/tf) deserializza decine di
    # trasformazioni ad alta frequenza. Con l'executor single-thread lo
    # faceva NEL thread dei callback immagine, affamando ArUco/depth
    # (il nodo restava muto per decine di secondi). I callback pesanti
    # stanno in un gruppo separato, il TF resta nel gruppo di default.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        node.destroy_node()
        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
