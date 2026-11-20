#!/usr/bin/env python3
#Abbiamo finalmente isolato il problema: il sistema di previsione della visibilità non rappresenta correttamente quello che la camera vede davvero.!!!!!!!!!!!
"""RoverTech: avvicinamento al main_switch. Ora c'è da debuggare la parte di ristima della posizione con depth dopo la prima stima, dopo il pre-avvicinamento + posizionamento del dito sotto alla leva)


  1. dopo /panel/align_done legge le pose depth di main_switch e delle 4 leve
     (pubblicate dal detector in base_link) e ne ricava la normale del pannello
     dalla fila di leve (NON serve /panel/pose_base);
     
  2. FASE 1: punta a 8 cm dalla leva; 
  3. FASE 2: punta a 5 cm;
  4. FASE 3: ristima la posizione della leva con RGB-D NUOVE (camera ferma);
  5. FASE 4-5: scende davanti alla leva, si infila SOTTO la punta e sale
     spingendola verso l'alto, controllando l'angolo di main_switch.
     
Per ora è stata implementata solo la fase 1,2: il robot appena arriva a 8cm non vede piu il bottone e qua dovrei cambiare
L'idea è che Prima di ogni avvicinamento scelga una soluzione IK che mantenga la leva nell'inquadratura prevista dalla camera. quello fatto qua è che dopo ogni movimento vede se si vede il pulsante, ma non va bene perche se non si vede è tempo perso. bisogna capire come fare.

Per ora, se dopo essersi mosso non vede piu la leva interrompe il movimento e si ferma.

VIEW SEARCH (versione con pitch netto): j2, j3 e ee_base_joint_x ruotano tutti attorno a X, quindi
l'inclinazione di dito e camera dipende SOLO dalla somma q2+q3+q4. Variare q4 da solo non serve:
la IK ricalcola j2/j3 per riportare la punta sul target e annulla quasi tutta la rotazione. Ora la ricerca
scansiona direttamente il pitch netto (WRIST_GRID_RAD, nei due versi) e risolve la IK a pitch FISSO.
Per ogni candidata prevede: leva nel FOV, dito che NON copre la leva (modello di occlusione),
profondita camera-leva. Se la riosservazione fallisce salva /tmp/lever_view_XX.png (cosa vede la camera).

Risolve IK dei 5 giunti direttamente rispetto alla punta fisica (cioè la punta del dito).

Questo script, di default, fa una simulazione "finta" solo con OMPL: per far si che la simulazione avvii davvero devi lanciarla con --execute.
Nessun angolo obiettivo fissato nel codice: ogni run ricomputa la IK, quindi è totalmente indipendente dalla posizione iniziale che viene data al rover. l'unica cosa, è che se si mette il pannello piu distante o piu vicino bisogna valutare i movimenti iniziali fatti dal braccio per trovare gli aruco. 
Inoltre, il pannello troppo lontano non va bene per depth e, se tanto tanto lontano, neanche per RGB. Ho fatto una demo a parte in cui il rover si trova lontano dal pannello ma abbastanza vicino da poter risolvere gli aruco. Da qua, stima la sua posizione rispetto al pannello. IN questo modo se serve quelli di Navigation con Nav2 possono dire al rover di avvicinarsi al pannello. IO uquella parte lho lasciata stare e ho iniziato a lavorare au un'altra demo. 

Plan only: python3 main_switch_pre_approach_AUTO.py --wait-align : qua se vuoi fare solo la prova generale, pero non mouve il braccio su gazebo
Gazebo:    python3 main_switch_pre_approach_AUTO.py --wait-align --execute : qua, se oveit da l'ok allo script, prende la traiettoria e la spara direttamente ai controller di Gazebo. 
Il launch dedicato (demo_rover_panel_depth) avvia questo script automaticamente.

Opzione --final-approach true: seconda traiettoria 8->5 cm (default), SENZA contatto. Quando si testerà su hardware reale si dovranno aggiungere i controllo di forza/impedenza !!!!

----------------------------------------------------------------------------------------------------------------------
"""

"""IMPORTAZIONE DELLE VARIE LIBRERIE"""

""" 1. Le fondamenta di Python (Sistema e Utilità base) """
import argparse
import os
import sys
import time
import math
import copy
import subprocess

""" 2. La matematica seria """
import numpy as np

""" 3. Tutto cio che riguarda Ros2 """
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

""" 4. Librerie /classiche gestiscono i sensori """
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Bool
from rosgraph_msgs.msg import Clock

""" 5. La Visione Artificiale (OpenCV per depth e RGB) """
import cv2
from cv_bridge import CvBridge

""" 6. MoveIt 2 e TF """
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetPositionFK, GetStateValidity
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents
from tf2_ros import Buffer, TransformListener

""" 7. Action Client e Controller """
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

#prendiamo l'URDF e lo traduciamo in numeri
JOINTS = ('j1_joint', 'j2_joint', 'j3_joint', 'ee_base_joint_x', 'ee_base_joint_y')
JOINT_LINKS = ('j1', 'j2', 'j3', 'ee_base_dummy', 'ee_base')
JOINT_AXES = np.array([(0.,0.,1.), (1.,0.,0.), (1.,0.,0.),
                       (1.,0.,0.), (0.,1.,0.)], dtype=float)
FK_LINKS = list(JOINT_LINKS) + ['ee_finger1'] #per aggiungere alla nostra catena il dito finale. Siccome non è un joint non possiamo aggiungerlo nei joints.

TCP_LOCAL = (+0.016, +0.140, -0.011) #offset tra polso e punta del dito, altrimenti vai col polso sul bottone e spacchi tutto perche il dito è montato sul polso e ci sarebbe un misunderstanding
STANDOFF_M = 0.08 #distanza x fase 1
FINAL_STANDOFF_DEFAULT_M = 0.05  #  fase 2
FINAL_APPROACH_MIN_M = 0.045  # preservata la seconda fase 8->5cm gia validata
ALL_CONTROL_NAMES = ('main_switch',) + tuple(f'lever_switch_{i}' for i in range(1,5)) + tuple(f'rotary_switch_{i}' for i in range(1,6)) + ('power_switch_1','power_switch_2') #trucco di python: sul pannello ERC ci sono tanti interruttori e anziche scriverli tutti a mano li unisci con tuple con un ciclo.

#questi due vengono dall'urdf: sono i limiti fisici dei giunti
BOUNDS_LOW = np.array((-np.inf,-1.5808,-1.5808,-0.01,-np.inf))
BOUNDS_HIGH = np.array((np.inf,1.5808,1.5808,3.1516,np.inf))

#i dati qua sotto vengono o da altri urdf tipo pannello oppure li ho sparati io e poi saranno da tunare. 
MAX_GOAL_TIP_ERROR_M = 0.015     # la IK ammette fino a 1.5cm di errore
MAX_START_STILL_RAD = 0.020     # sotto i 0.02 rad, consideri il robot fermo
MAX_TF_FK_ERROR_M = 0.040      # MoveIt e TF coerenti entro 4 cm o si ferma tutto
MAX_DETOUR_M = 0.060           # massima deviazione del dito dal segmento iniziale-finale. cioe, se la tua traiettoria si allontana piu di 6cmd a quella ideale rifiutala
MAX_FINGERTIP_TRAVEL_M = 0.75  # scartare target inattesi molto lontani
MAX_POSE_CHANGE_M = 0.010      # se la percezione cambia durante il calcolo, stop

CAMERA_OPTICAL_FRAME = 'gemini_camera_optical_frame'

# ---- RICERCA DELLA VISTA (view_goal_candidates) ----
# WRIST_GRID_RAD sono ora passi di PITCH NETTO (q2+q3+q4, cioe l'inclinazione di dito+camera),
# provati nei due versi rispetto alla IK di partenza. Non sono piu offset del solo ee_base_joint_x:
# quello veniva annullato da j2/j3 quando la IK riportava la punta sul target.
WRIST_GRID_RAD = (0.1, 0.3, 0.5)
WRIST_PITCH_EXTRA_RAD = (0.8, 1.1)   # passi grandi aggiunti alla griglia (stessa logica)
WRIST_FALLBACK = ((0.15, 0.0), (0.30, 0.0), (-0.15, 0.0), (-0.30, 0.0),
                  (0.0, 0.15), (0.0, -0.15))
MIN_VIEW_MARGIN_PX = 60.0      # la leva deve stare ad almeno 60 px dal bordo immagine
CAMERA_MIN_DEPTH_M = 0.10      # leva piu vicina di cosi alla camera = scartata. DA TARARE sul clip "near" della camera nell'SDF
MIN_OCCLUSION_CLEAR_PX = 8.0   # la leva deve stare ad almeno 8 px dal bordo del dito proiettato
FINGER_RADIUS_M = 0.022        # raggio del dito nel modello di occlusione (box ~4.2 x 2.2 cm)
# ---- ROLL DEL POLSO (ee_base_joint_y) ----
# La camera e' montata sotto il polso: ruotando l'asse di approccio le dita
# escono dal campo visivo. Provato sulle migliori candidate di pitch.
ROLL_GRID_RAD = (0.6, -0.6, 1.2, -1.2)
ROLL_VARIANTS_FROM = 3         # quante candidate di pitch raffinare col roll
# ---- GRIPPER ----
# Apertura dita (slider_joint, limiti URDF -0.01..0.06; le dita mimano con
# multiplier ~31.4 rad/m, quindi 0.04 m = ~1.26 rad di apertura per dito).
GRIPPER_OPEN_POS = 0.04

LEVER_NAMES = tuple(f'lever_switch_{i}' for i in range(1, 5)) #genera array con le leve
POSE_MAX_AGE_S = 180.0         # se il rover è stato fermo piu di 3 min, chiede una nuova acquisizione

#qua sotto il millimetro puro per l'ingaggio meccanico del pannello
LEVER_HALF_THICKNESS_M = 0.003  # la ROI rossa misura la faccia anteriore (box spesso 6 mm)
CONTACT_FACE_CLEARANCE_M = 0.001  # distanza minima dei volumi delle dita dal piano pannello
LIFT_DONE_ANGLE_RAD = 0.30     # main_switch oltre questo angolo = leva alzata
LIFT_RISES_M = (0.010, 0.025, 0.040, 0.055)  # quote punta sopra il bordo basso della leva


#saturazione e ricerca del percors piu breve per i giunti
def clamp_to_limits(q, center):
    q = np.minimum(np.maximum(np.asarray(q, dtype=float), BOUNDS_LOW), BOUNDS_HIGH)
    # J1 e J5 sono continui: la soluzione piu vicina all'attuale.
    for idx in (0, 4):
        q[idx] = center[idx] + math.atan2(math.sin(q[idx]-center[idx]),
                                          math.cos(q[idx]-center[idx]))
    return q

#per stampare bene i numeri sulla cli
def fmt(values):
    return '(' + ', '.join(f'{x:+.3f}' for x in values) + ')'


#GEOMETRIA PROIETTIVA: simula la vista della camera nella sua testa! cioe, in base alla traiettoria pianificata, ti immagini dove sarai e cosa vedrai prendendo il 3D del pannello e portandolo nel 2D della camera tramite geometria proiettiva. Ti dici se muovo il motore di rtot gradi, la camera sara in questa coordinata. quello che vedra del pannello sara sto pezzo 3D, che usando la K della camera diventa questa 2D sul piano. se il bottone cade bene al centro della visuale approva il piano!!
def camera_margin(K, width, height, cam_pose, points, min_z=0.08, max_z=1.2):
    """Margine minimo (px) dal bordo immagine dei punti 3D (base_link).

    Negativo se un punto cade fuori immagine o fuori dal range di profondita.
    cam_pose: Pose del frame OTTICO della camera in base_link.
    """
    q = cam_pose.orientation
    p = cam_pose.position
    t = np.array((p.x, p.y, p.z), dtype=float)
    axes = np.column_stack([np.array(rot(q, tuple(np.eye(3)[i]))) for i in range(3)])
    worst = float('inf')
    for pt in points:
        pc = axes.T @ (np.asarray(pt, dtype=float) - t)
        if not (min_z <= pc[2] <= max_z):
            return -1.0
        u = K[0, 0]*pc[0]/pc[2] + K[0, 2]
        v = K[1, 1]*pc[1]/pc[2] + K[1, 2]
        worst = min(worst, u, width-u, v, height-v)
    return float(worst)


#prendiamo i punti 3D delle 4 leve rosse e calcoliamo l'orientamento geometrico esatto nello spazio rispetto al rover.
#con la SVD troviamo la direzione principale lungo cui sono disposte le leve. tira la perpendicolare e si assicura che guardi nella direzione giusta verso il rover.
def horizontal_row_normal(points, center):
    """Normale (orizzontale) del pannello verticale dalla fila di leve misurate.

    points: Nx3 in base_link (>=3 leve); center: punto della leva usato per
    orientare la normale verso il rover (origine di base_link).
    """
    P = np.asarray(points, dtype=float)[:, :2]
    if len(P) < 3:
        raise RuntimeError('Servono almeno 3 leve misurate per stimare la normale.')
    c = P.mean(axis=0)
    _, sv, vt = np.linalg.svd(P - c)
    if sv[0] < 0.08:
        raise RuntimeError('Leve troppo vicine tra loro: normale non stimabile.')
    d = vt[0]
    n2 = np.array((-d[1], d[0]), dtype=float)
    if float(np.dot(n2, -c)) < 0.0:
        n2 = -n2
    return np.array((n2[0], n2[1], 0.0), dtype=float)


#Per non essere sicuri di bloccarci se non arriva una risposta immediata
def call(node, client, req, timeout=8.0):
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError('Timeout del servizio ' + client.srv_name)
    if future.exception() is not None:
        raise RuntimeError(str(future.exception()))
    return future.result()


#traduci quello che viene visto dalla camera nelle coordinate 3D del mondo
def rot(q, p):
    u = (q.x, q.y, q.z)
    v = q.w
    n2 = sum(z*z for z in u) + v*v
    if not 0.9 < n2 < 1.1:
        raise RuntimeError('Quaternione non valido in FK o posa pannello')
    inv = 1.0 / math.sqrt(n2)
    u = tuple(z*inv for z in u)
    v *= inv
    cross1 = (u[1]*p[2]-u[2]*p[1], u[2]*p[0]-u[0]*p[2], u[0]*p[1]-u[1]*p[0])
    cross2 = (u[1]*cross1[2]-u[2]*cross1[1],
              u[2]*cross1[0]-u[0]*cross1[2],
              u[0]*cross1[1]-u[1]*cross1[0])
    return tuple(p[i] + 2.0*(v*cross1[i] + cross2[i]) for i in range(3))


def rotation_matrix(q):
    """Rotation matrix from geometry_msgs Quaternion (same convention as rot)."""
    return np.column_stack([np.asarray(rot(q, axis), dtype=float)
                            for axis in ((1.,0.,0.), (0.,1.,0.), (0.,0.,1.))])

#scrive i giunti; prende lo stato del robot rs e una lista di nuivi angoli e fa una simulazione "mentale"
def put_joints(rs, q):
    new = copy.deepcopy(rs)
    names = list(new.joint_state.name)
    vals = list(new.joint_state.position)
    for name, value in zip(JOINTS, q):
        vals[names.index(name)] = float(value)
    new.joint_state.position = vals
    return new


#legge i valori dei giunti
def q_from_state(rs):
    mapping = dict(zip(rs.joint_state.name, rs.joint_state.position))
    if not all(j in mapping for j in JOINTS):
        raise RuntimeError('Mancano uno o piu giunti del gruppo arm nello stato MoveIt')
    return tuple(mapping[j] for j in JOINTS)

#da ros a float
def duration_seconds(t):
    return float(t.sec) + float(t.nanosec)*1e-9

#il contrario
def duration_msg(t):
    sec = int(math.floor(t))
    nanosec = int(round((t-sec)*1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


class Test(Node): #sto nodo ascolta l'ambiente, decide la traiettoria chiamando i servizi cpp e una volta che moveit gli da il piano mette self.frozen su true
    def __init__(self):
        super().__init__('main_switch_pre_approach_auto') #crea il nodo
        self.panel = self.switch = None #crea le variabili per memorizzare la posa 3D che all'inizio non si sa quale sia quindi metti None
        self.t_panel = self.t_switch = 0.0 #stesso ma per l'interruttore
        self.aligned = False  #non è davanti al pannello
        self.wait_align = False 
        self.main_switch_angle = None
        self.main_switch_peak = None
        self.control_poses = {}      # nome -> (PoseStamped, tempo monotonic)
        self.frozen = False          # True dopo get_target: ignora nuovi dati visivi dopo aver deciso una traiettoria
        self.center0 = self.normal0 = None
        self._pitch_signs = None     # (s3, s4): segno degli assi mondo di j3 e q4 rispetto a j2
        for control in ALL_CONTROL_NAMES:
            if control != "main_switch":
                self.create_subscription(PoseStamped, f"/panel/controls/{control}/pose_base", #crea subscriber per ogni pulsante
                                         lambda m, c=control: self.on_other_control(c, m), 10)
        qos_align = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/panel/align_done', self.on_align, qos_align)
        self.create_subscription(PoseStamped, '/panel/pose_base', self.on_panel, 10)
        self.create_subscription(JointState, '/panel/joint_states', self.on_panel_joints, 10)
        self.create_subscription(PoseStamped, '/panel/controls/main_switch/pose_base', self.on_switch, 10)
        self.scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.valid = self.create_client(GetStateValidity, '/check_state_validity')
        self.plan = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.cam_K = None
        self.cam_W = self.cam_H = 0
        # Mount between ee_base and optical camera, calibrated ONCE while stationary.
        # It is rigid: the calibration lets FK predict the camera pose for any IK.
        self.camera_mount_translation = None
        self.camera_mount_rotation = None
        self.create_subscription(CameraInfo, '/camera/rgbd/camera_info',
                                 self.on_cam_info, qos_profile_sensor_data)

    def on_cam_info(self, msg):
        K = np.asarray(msg.k, dtype=float).reshape(3, 3)
        if np.all(np.isfinite(K)) and K[0, 0] > 0 and K[1, 1] > 0 and msg.width > 0:
            self.cam_K, self.cam_W, self.cam_H = K, int(msg.width), int(msg.height)

    def on_other_control(self, control, msg):
        if self.frozen or (self.wait_align and not self.aligned):
            return
        if msg.header.frame_id == "base_link":
            self.control_poses[control] = (msg, time.monotonic())

    def on_panel_joints(self, msg):
        if 'main_switch' in msg.name:
            i = list(msg.name).index('main_switch')
            if i < len(msg.position):
                self.main_switch_angle = float(msg.position[i])
                self.main_switch_peak = max(self.main_switch_peak, self.main_switch_angle) if self.main_switch_peak is not None else self.main_switch_angle

    def on_align(self, msg): #cancella tutti i dati in memoria appena il rover ha smesso di muoversi
        if msg.data and not self.aligned:
            self.aligned = True
            # Serve una nuova stima DOPO l'ultimo evento align_done.
            self.panel = self.switch = None
            self.control_poses.clear()
            self.t_panel = self.t_switch = 0.0
            print('ALIGN DONE: attendo nuove pose del pannello e della leva.', flush=True)

    def on_panel(self, msg):
        if self.frozen or (self.wait_align and not self.aligned):
            return
        self.panel, self.t_panel = msg, time.monotonic()

    def on_switch(self, msg):
        if self.frozen or (self.wait_align and not self.aligned):
            return
        self.switch, self.t_switch = msg, time.monotonic()

    def fresh_levers(self):
        now = time.monotonic()
        return {c: m for c, (m, t) in self.control_poses.items()
                if c in LEVER_NAMES and now - t <= POSE_MAX_AGE_S}

    def measurements_ready(self): #controlla che leva sia visibile+rover allineato altrimenti blocca tutto
    #ho pensato potesse essere lui il motivo per cui non si ferma 
        if self.wait_align and not self.aligned:
            return False
        if self.switch is None or time.monotonic() - self.t_switch > POSE_MAX_AGE_S:
            return False
        return len(self.fresh_levers()) >= 3

    def get_target(self): #calcola il punto 3D dove far finire l'end effector
        print('Attendo /panel/align_done e pose depth di main_switch + leve...' if self.wait_align
              else 'Attendo pose depth di main_switch + leve...', flush=True)
        deadline = time.monotonic() + (600.0 if self.wait_align else 25.0)
        while time.monotonic() < deadline and not self.measurements_ready():
            rclpy.spin_once(self, timeout_sec=0.15)
        if not self.measurements_ready():
            raise RuntimeError('Non ricevo le pose depth di main_switch e leve dopo '
                               'l allineamento. Nessun movimento.')
        # Il detector pubblica le pose in burst: lascia arrivare anche le altre.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.switch.header.frame_id != 'base_link':
            raise RuntimeError('La posa del main_switch deve essere in base_link.')
        lever = self.switch.pose.position
        center = np.array([lever.x, lever.y, lever.z], dtype=float)
        if not np.all(np.isfinite(center)):
            raise RuntimeError('Coordinate main_switch non valide.')
        levers = self.fresh_levers()
        pts = [(m.pose.position.x, m.pose.position.y, m.pose.position.z)
               for m in levers.values()]
        if not np.all(np.isfinite(np.asarray(pts, dtype=float))):
            raise RuntimeError('Coordinate leve non valide.')
        normal = horizontal_row_normal(pts, center)
        normal /= np.linalg.norm(normal)
        if np.dot(normal, -center) <= 0.05:
            raise RuntimeError('Normale non rivolta verso il rover. STOP.')
        target = center + STANDOFF_M * normal
        # Congela: da qui in poi le pose del detector (che continua a pubblicare
        # col braccio in moto) NON cambiano il target. La ristima la fa la FASE 3.
        self.center0, self.normal0, self.frozen = center.copy(), normal.copy(), True
        ang = math.degrees(math.atan2(normal[1], -normal[0]))
        print(f'Main switch (depth): {fmt(center)}', flush=True)
        print(f'Normale dalla fila di {len(pts)} leve (verso rover): {fmt(normal)} '
              f'[{ang:+.1f} deg rispetto a -X]', flush=True)
        print('TARGET AUTONOMO punta a 8 cm:', fmt(target), flush=True)
        return tuple(float(v) for v in target)

    def get_state(self): #fa foto dello stato attuale
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE
        return call(self, self.scene, req).scene.robot_state

    def fingertip_fk(self, rs): #calcola coordinate finali in cui andrebbe a trovarsi la pinza
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['ee_finger1']
        req.robot_state = rs
        out = call(self, self.fk, req)
        if out.error_code.val != MoveItErrorCodes.SUCCESS or len(out.pose_stamped) != 1:
            raise RuntimeError(f'FK punta non valida: code={out.error_code.val}')
        pose = out.pose_stamped[0].pose
        offset = rot(pose.orientation, TCP_LOCAL)
        p = pose.position
        return tuple(a+b for a, b in zip((p.x, p.y, p.z), offset))

    def fingertip_and_jacobian(self, q, original): #calcola la jacobiana
        """FK del vero punto dito e Jacobiano geometrico dei 5 giunti."""
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = FK_LINKS
        req.robot_state = put_joints(original, q)
        out = call(self, self.fk, req)
        if out.error_code.val != MoveItErrorCodes.SUCCESS or len(out.pose_stamped) != len(FK_LINKS):
            raise RuntimeError('MoveIt FK fallita durante IK automatica.')
        poses = {name: value.pose for name, value in zip(out.fk_link_names, out.pose_stamped)}
        f = poses['ee_finger1']
        tip = np.array((f.position.x, f.position.y, f.position.z)) + \
              np.array(rot(f.orientation, TCP_LOCAL))
        J = np.zeros((3, len(JOINTS)), dtype=float)
        for i, (link, axis) in enumerate(zip(JOINT_LINKS, JOINT_AXES)):
            pose = poses[link]
            origin = np.array((pose.position.x, pose.position.y, pose.position.z))
            axis_world = np.array(rot(pose.orientation, axis))
            J[:, i] = np.cross(axis_world, tip - origin)
        return tip, J

    def solve_ik(self, qseed, qstart, target, original, stop_tolerance=0.006, free=None):
        """Damped least-squares Jacobian, medesimo solver verificato con successo."""
        q = clamp_to_limits(qseed, qstart)
        best = (float('inf'), q.copy())
        damping = 0.045
        for _ in range(45):
            tip, jac = self.fingertip_and_jacobian(q, original)
            error = target - tip
            dist = float(np.linalg.norm(error))
            if dist < best[0]:
                best = (dist, q.copy())
            if dist < stop_tolerance:
                break
            if free is not None:
                jac = jac * np.array([1.0 if i in free else 0.0 for i in range(len(JOINTS))])
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping*damping*np.eye(3),error)
            if not np.all(np.isfinite(dq)):
                break
            dq *= min(1.0,0.22/max(float(np.max(np.abs(dq))),1e-12))
            improved = False
            for scale in (1.0,0.5,0.25):
                trial = clamp_to_limits(q + scale*dq, qstart)
                tip_trial,_ = self.fingertip_and_jacobian(trial, original)
                if float(np.linalg.norm(target-tip_trial)) < dist-0.00002:
                    q = trial
                    damping = max(0.015,damping*0.85)
                    improved = True
                    break
            if not improved:
                damping = min(0.25,damping*2.0)
                if damping >= 0.24:
                    break
        return best

    def wrist_pitch_signs(self, state):
        """Verifica che gli assi MONDO di j2, j3 e ee_base_joint_x siano paralleli e ne ricava i segni.

        Pitch netto = q2 + s3*q3 + s4*q4 (s3, s4 = +1 se l'asse mondo e' concorde con j2, -1 se opposto).
        Se gli assi non sono paralleli, il concetto di 'pitch netto' non vale: errore.
        """
        if self._pitch_signs is not None:
            return self._pitch_signs
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['j2', 'j3', 'ee_base_dummy']
        req.robot_state = state
        out = call(self, self.fk, req)
        if out.error_code.val != MoveItErrorCodes.SUCCESS or len(out.pose_stamped) != 3:
            raise RuntimeError('FK degli assi del polso non disponibile.')
        poses = {name: value.pose for name, value in zip(out.fk_link_names, out.pose_stamped)}
        ax = {link: np.array(rot(poses[link].orientation, JOINT_AXES[i]))
              for link, i in (('j2', 1), ('j3', 2), ('ee_base_dummy', 3))}
        d3 = float(np.dot(ax['j2'], ax['j3']))
        d4 = float(np.dot(ax['j2'], ax['ee_base_dummy']))
        print(f'WRIST PITCH: assi mondo j2.j3={d3:+.2f}, j2.q4={d4:+.2f} (devono essere +-1)', flush=True)
        if abs(abs(d3)-1.0) > 0.05 or abs(abs(d4)-1.0) > 0.05:
            raise RuntimeError('Assi di j2/j3/q4 non paralleli: pitch netto non definito.')
        self._pitch_signs = (1.0 if d3 > 0 else -1.0, 1.0 if d4 > 0 else -1.0)
        return self._pitch_signs

    def solve_ik_pitch(self, qseed, qstart, target, original, theta, s3, s4, stop_tolerance=0.004):
        """IK a PITCH NETTO FISSO: q2 + s3*q3 + s4*q4 = theta.

        j1 libero; j2 e j3 si muovono e q4 compensa (dq4 = -s4*(dq2 + s3*dq3)), quindi durante
        la IK l'inclinazione di dito e camera NON cambia. (Con q4 libero, la IK normale
        ricalcola j2/j3 e disfa quasi tutta la rotazione del polso.)
        Ritorna (errore_punta_m, q). Se q4 batte sul limite il pitch reale != theta: controllalo fuori.
        """
        target = np.asarray(target, dtype=float)

        def with_pitch(qq):
            qq = np.array(qq, dtype=float)
            qq[3] = s4*(theta - qq[1] - s3*qq[2])
            return clamp_to_limits(qq, qstart)

        q = with_pitch(qseed)
        best = (float('inf'), q.copy())
        damping = 0.045
        for _ in range(45):
            tip, J = self.fingertip_and_jacobian(q, original)
            error = target - tip
            dist = float(np.linalg.norm(error))
            if dist < best[0]:
                best = (dist, q.copy())
            if dist < stop_tolerance:
                break
            # colonne accoppiate: muovere j2/j3 muove anche q4 per tenere il pitch
            Jc = np.column_stack((J[:, 0], J[:, 1] - s4*J[:, 3], J[:, 2] - s4*s3*J[:, 3]))
            dc = Jc.T @ np.linalg.solve(Jc @ Jc.T + damping*damping*np.eye(3), error)
            if not np.all(np.isfinite(dc)):
                break
            dc *= min(1.0, 0.22/max(float(np.max(np.abs(dc))), 1e-12))
            improved = False
            for scale in (1.0, 0.5, 0.25):
                trial = q.copy()
                trial[0] += scale*dc[0]
                trial[1] += scale*dc[1]
                trial[2] += scale*dc[2]
                trial = with_pitch(trial)
                tip_t, _ = self.fingertip_and_jacobian(trial, original)
                if float(np.linalg.norm(target - tip_t)) < dist - 0.00002:
                    q = trial
                    damping = max(0.015, damping*0.85)
                    improved = True
                    break
            if not improved:
                damping = min(0.25, damping*2.0)
                if damping >= 0.24:
                    break
        return best[0], best[1]

    def find_joint_goal(self, q_start, target, original, precision=False):
        q0 = np.array(q_start, dtype=float)
        t = np.array(target, dtype=float)
        seeds = [
            ('polso dritto/alto', np.array([q0[0], q0[1], q0[2], 0.0, 0.0])),
            ('polso pitch su 1', np.array([q0[0], q0[1], q0[2], -0.35, 0.0])),
            ('polso pitch su 2', np.array([q0[0], q0[1], q0[2], -0.70, 0.0])),
            ('configurazione attuale', np.array([q0[0], q0[1], q0[2], q0[3], 0.0])),
        ]
        valid_solutions = []
        print('\nIK AUTOMATICA DELLA PUNTA: ricerca angoli dal target LIVE...', flush=True)
        for name, seed in seeds:
            dist, q = self.solve_ik(seed, q0, t, original, stop_tolerance=(.002 if precision else .006), free=(0,1,2,3))
            req = GetStateValidity.Request()
            req.robot_state = put_joints(original,q)
            req.group_name = 'arm'
            response = call(self, self.valid, req)
            print(f'  {name}: errore punta={dist*100:.2f} cm, '
                  f'collisioni={"NO" if response.valid else "SI"}, '
                  f'angoli={fmt(q)}', flush=True)
            if response.valid and dist <= (.004 if precision else MAX_GOAL_TIP_ERROR_M):
                valid_solutions.append((dist, q.copy(), name))
                if dist < (.002 if precision else .006):
                    break
        if not valid_solutions:
            raise RuntimeError('IK della punta non trovata entro 1.5 cm senza collisioni. '
                               'Nessun comando al robot.')
        dist, q, source = min(valid_solutions, key=lambda x: x[0])
        print(f'IK SELEZIONATA: {source}; errore={dist*100:.2f} cm; '
              f'GOAL GIUNTI={fmt(q)}', flush=True)
        return tuple(float(v) for v in q)

    def calibrate_camera_geometry(self, robot_state):
        """Calibrate fixed ee_base -> optical_camera using ACTUAL TF and FK.

        Important: the SDF Gazebo conversion may lump away a fixed optical joint,
        therefore asking MoveIt for FK(gemini_camera_optical_frame) is not safe.
        No camera Xacro, detector, marker or depth algorithm is altered here.
        """
        if self.camera_mount_translation is not None:
            return
        deadline=time.monotonic()+6.0
        while (self.cam_K is None or not self.tf_buffer.can_transform(
                'base_link', CAMERA_OPTICAL_FRAME, Time())) and time.monotonic()<deadline:
            rclpy.spin_once(self, timeout_sec=0.12)
        if self.cam_K is None:
            raise RuntimeError('VISION GUARD: /camera/rgbd/camera_info mancante; non muovo.')
        if not self.tf_buffer.can_transform('base_link', CAMERA_OPTICAL_FRAME, Time()):
            raise RuntimeError('VISION GUARD: TF camera ottica mancante; non invento orientamenti.')
        tf_cam=self.tf_buffer.lookup_transform('base_link', CAMERA_OPTICAL_FRAME, Time()).transform
        req=GetPositionFK.Request()
        req.header.frame_id='base_link'
        req.fk_link_names=['ee_base']
        req.robot_state=robot_state
        result=call(self,self.fk,req)
        if result.error_code.val!=MoveItErrorCodes.SUCCESS or len(result.pose_stamped)!=1:
            raise RuntimeError('VISION GUARD: FK di ee_base non disponibile.')
        ee=result.pose_stamped[0].pose
        p_ee=np.array((ee.position.x,ee.position.y,ee.position.z),float)
        R_ee=rotation_matrix(ee.orientation)
        p_cam=np.array((tf_cam.translation.x,tf_cam.translation.y,tf_cam.translation.z),float)
        R_cam=rotation_matrix(tf_cam.rotation)
        if not np.all(np.isfinite(p_cam)):
            raise RuntimeError('VISION GUARD: TF camera non valida.')
        # A rigid camera must be within 30 cm of the flange.
        translation=R_ee.T@(p_cam-p_ee)
        if np.linalg.norm(translation)>0.30:
            raise RuntimeError('VISION GUARD: ee_base e camera non coerenti (offset >30 cm).')
        self.camera_mount_translation=translation
        self.camera_mount_rotation=R_ee.T@R_cam
        print(f'VISION GUARD: TF camera calibrata su ee_base; offset={fmt(translation)}',flush=True)

    def projected_lever(self, state, center):
        """Predict where the *already measured* switch will appear in new camera view.

        This is a projection to rank IK poses; it is NOT a new depth measurement.
        The existing FreshDepthObserver checks the REAL RGB-D after movement.

        Ritorna None se la leva e' dietro la camera / fuori range di profondita, altrimenti un dict:
          margin : margine minimo (px) dal bordo immagine dei 3 punti della leva
          mid    : penalita' di centratura (0 = leva al centro)
          pixel  : (u, v) del centro leva
          zcam   : profondita camera->leva (m)
          occ    : distanza (px) tra la leva e il bordo del dito proiettato, solo per le parti del dito
                   PIU VICINE alla camera della leva; negativo = il dito copre la leva
          elev   : elevazione del dito rispetto all'orizzontale (gradi, + = punta verso l'alto)
        """
        if self.camera_mount_translation is None or self.cam_K is None:
            raise RuntimeError('VISION GUARD: calibrazione camera non eseguita.')
        req=GetPositionFK.Request()
        req.header.frame_id='base_link'
        req.fk_link_names=['ee_base','ee_finger1','ee_finger2']
        req.robot_state=state
        result=call(self,self.fk,req)
        if result.error_code.val!=MoveItErrorCodes.SUCCESS or len(result.pose_stamped)!=3:
            raise RuntimeError('VISION GUARD: FK camera/dita incompleta.')
        poses={k:v.pose for k,v in zip(result.fk_link_names,result.pose_stamped)}
        if not all(k in poses for k in ('ee_base','ee_finger1','ee_finger2')):
            raise RuntimeError('VISION GUARD: link FK inattesi.')
        ee=poses['ee_base']
        R_ee=rotation_matrix(ee.orientation)
        p_ee=np.array((ee.position.x,ee.position.y,ee.position.z),float)
        p_cam=p_ee+R_ee@self.camera_mount_translation
        R_cam=R_ee@self.camera_mount_rotation
        fx,fy=self.cam_K[0,0],self.cam_K[1,1]
        cx,cy=self.cam_K[0,2],self.cam_K[1,2]
        c=np.array(center,dtype=float)
        # Cover the centre, lower and upper portions of this 3.5-cm switch.
        controls=(c,c+np.array((0.,0.,-0.025)),c+np.array((0.,0.,0.015)))
        projections=[]   # (u, v, z_camera)
        for p in controls:
            pc=R_cam.T@(p-p_cam)
            if not (CAMERA_MIN_DEPTH_M<pc[2]<1.2):
                return None
            u=fx*pc[0]/pc[2]+cx
            v=fy*pc[1]/pc[2]+cy
            if not np.isfinite(u+v):
                return None
            projections.append((float(u),float(v),float(pc[2])))
        margin=min(min(u,self.cam_W-u,v,self.cam_H-v) for u,v,_ in projections)
        u,v,zcam=projections[0]
        mid_penalty=((u-self.cam_W*.50)/self.cam_W)**2+((v-self.cam_H*.51)/self.cam_H)**2
        # Occlusione da parte delle dita: ogni dito e' un segmento lungo la linea centrale del suo box
        # (x,z di TCP_LOCAL, y da 0 a TCP_LOCAL[1]) con raggio FINGER_RADIUS_M. Conta solo se il punto
        # del dito e' piu vicino alla camera della leva (altrimenti sta dietro e non copre nulla).
        occ=float('inf')
        for name in ('ee_finger1','ee_finger2'):
            fp=poses[name]
            p0=np.array((fp.position.x,fp.position.y,fp.position.z),float)
            for s in np.linspace(0.0,1.0,9):
                local=(TCP_LOCAL[0],TCP_LOCAL[1]*float(s),TCP_LOCAL[2])
                pf=p0+np.array(rot(fp.orientation,local))
                pcf=R_cam.T@(pf-p_cam)
                if pcf[2]<0.03:
                    continue
                uf=fx*pcf[0]/pcf[2]+cx
                vf=fy*pcf[1]/pcf[2]+cy
                radius_px=fx*FINGER_RADIUS_M/pcf[2]
                for ul,vl,zl in projections:
                    if pcf[2]>zl+0.01:
                        continue
                    occ=min(occ,math.hypot(ul-uf,vl-vf)-radius_px)
        fdir=np.array(rot(poses['ee_finger1'].orientation,(0.,1.,0.)),float)
        elev=math.degrees(math.asin(float(np.clip(fdir[2],-1.0,1.0))))
        return dict(margin=float(margin),mid=float(mid_penalty),pixel=(float(u),float(v)),
                    zcam=float(zcam),occ=float(min(occ,999.0)),elev=float(elev))

    def view_goal_candidates(self, state, q_start, original_goal, target, center, max_n=4):
        """Candidate di arrivo a 8 cm ordinate per visibilita PREVISTA della leva.

        Scansiona il PITCH NETTO q2+q3+q4 (griglia WRIST_GRID_RAD nei due versi) con IK a pitch fisso,
        cosi' l'inclinazione di dito+camera cambia davvero. Per ognuna: punta sul target, nessuna
        collisione, leva nel FOV, dito che non copre la leva. La riosservazione RGB-D reale resta
        l'ultima parola (search_visible_arrival). Se non c'e' nessuna candidata NON blocca il
        movimento: ripiega sulla IK normale.
        """
        self.calibrate_camera_geometry(state)
        q0 = np.asarray(q_start, dtype=float)
        base = np.asarray(original_goal, dtype=float).copy()
        base[4] = 0.0
        goal = np.asarray(target, dtype=float)
        min_margin = max(60., min(110., .12*min(self.cam_W, self.cam_H)))
        try:
            s3, s4 = self.wrist_pitch_signs(state)
        except RuntimeError as exc:
            print(f'VIEW SEARCH: {exc} RIPIEGO: IK normale senza ricerca di visibilita.', flush=True)
            return [tuple(float(v) for v in original_goal)]

        def pitch_of(qq):
            return float(qq[1] + s3*qq[2] + s4*qq[3])

        theta_goal = pitch_of(base)
        steps = sorted(set(WRIST_GRID_RAD) | set(WRIST_PITCH_EXTRA_RAD))
        deltas = [0.0]
        for d in steps:
            deltas += [d, -d]

        print(f'\nVIEW SEARCH (pitch netto q2+q3+q4; partenza IK = {theta_goal:+.3f} rad, '
              f'pitch attuale = {pitch_of(q0):+.3f} rad):', flush=True)
        valid = []       # (score, err, view, q, pitch_reale)
        evaluated = 0
        for d in deltas:
            theta = theta_goal + d
            seed = base.copy()
            seed[3] = s4*(theta - seed[1] - s3*seed[2])
            err, q = self.solve_ik_pitch(seed, q0, goal, state, theta, s3, s4, stop_tolerance=.004)
            q[4] = 0.0
            evaluated += 1
            real = pitch_of(q)
            tag = f'  pitch={theta:+.2f} (d={d:+.2f})'
            if err > .010:
                print(f'{tag}: punta non raggiungibile con questo pitch (errore {err*100:.1f} cm)', flush=True)
                continue
            if abs(real - theta) > 0.03:
                print(f'{tag}: limite giunto (q4={q[3]:+.3f}), pitch ottenibile {real:+.2f}', flush=True)
                continue
            rs = put_joints(state, q)
            view = self.projected_lever(rs, center)
            if view is None:
                print(f'{tag}: leva dietro la camera o fuori range di profondita '
                      f'(min {CAMERA_MIN_DEPTH_M:.2f} m)', flush=True)
                continue
            line = (f'{tag}: q={fmt(q)}, dito {view["elev"]:+.0f} deg, z_cam={view["zcam"]:.2f} m, '
                    f'margine={view["margin"]:.0f}px, libero dal dito={view["occ"]:+.0f}px')
            if view['margin'] < min_margin:
                print(line + '  -> leva fuori/al bordo del FOV', flush=True)
                continue
            request = GetStateValidity.Request()
            request.robot_state = rs
            request.group_name = 'arm'
            if not call(self, self.valid, request).valid:
                print(line + '  -> collisione', flush=True)
                continue
            score = view['mid'] + 0.02*abs(real - theta_goal) + 2.0*err
            valid.append((float(score), float(err), view, np.asarray(q), real))
            print(line + ('  OK' if view['occ'] >= MIN_OCCLUSION_CLEAR_PX
                          else '  -> dito davanti alla leva'), flush=True)

        if not valid:
            # Mai bloccare il movimento per la sola ricerca di visibilita'.
            print(f'VIEW SEARCH: nessuna candidata valida ({evaluated} esaminate). '
                  'RIPIEGO: uso la IK normale senza garanzia di visibilita.', flush=True)
            return [tuple(float(v) for v in original_goal)]

        # ---- VARIANTI DI ROLL sulle migliori candidate di pitch ----
        # Le candidate sopra hanno tutte roll=0 (q[4]=0): con la camera sotto
        # il polso le dita restano davanti alla leva. Qui si ruota l'asse di
        # approccio (ee_base_joint_y) tenendo la punta sul target, con IK a
        # roll FISSO (free=[0,1,2,3]), e si rivaluta la visibilita'.
        roll_from = sorted(valid, key=lambda item: (-item[2]['occ'], item[0]))
        roll_from = roll_from[:ROLL_VARIANTS_FROM]
        for _, _, _, q_pitch, _ in roll_from:
            for phi in ROLL_GRID_RAD:
                seed = np.asarray(q_pitch, dtype=float).copy()
                seed[4] = phi
                err, q = self.solve_ik(seed, q0, goal, state,
                                       stop_tolerance=.004, free=[0, 1, 2, 3])
                q[4] = phi
                evaluated += 1
                tag = f'  roll={phi:+.2f}'
                if err > .010:
                    print(f'{tag}: punta non raggiungibile con questo roll '
                          f'(errore {err*100:.1f} cm)', flush=True)
                    continue
                rs = put_joints(state, q)
                # Verifica con la STESSA FK usata dalla pianificazione (non
                # solo con il residuo interno del solver): se la punta non e'
                # sul target qui, la candidata verrebbe scartata dopo.
                err_real = math.dist(self.fingertip_fk(rs), goal)
                if err_real > .010:
                    print(f'{tag}: FK reale fuori target '
                          f'({err_real*100:.1f} cm, solver {err*100:.2f} cm)',
                          flush=True)
                    continue
                err = err_real
                view = self.projected_lever(rs, center)
                if view is None:
                    print(f'{tag}: leva dietro la camera o fuori range di profondita',
                          flush=True)
                    continue
                line = (f'{tag}: q={fmt(q)}, dito {view["elev"]:+.0f} deg, '
                        f'z_cam={view["zcam"]:.2f} m, margine={view["margin"]:.0f}px, '
                        f'libero dal dito={view["occ"]:+.0f}px')
                if view['margin'] < min_margin:
                    print(line + '  -> leva fuori/al bordo del FOV', flush=True)
                    continue
                request = GetStateValidity.Request()
                request.robot_state = rs
                request.group_name = 'arm'
                if not call(self, self.valid, request).valid:
                    print(line + '  -> collisione', flush=True)
                    continue
                score = view['mid'] + 0.02*abs(phi) + 2.0*err
                valid.append((float(score), float(err), view, np.asarray(q), pitch_of(q)))
                print(line + ('  OK' if view['occ'] >= MIN_OCCLUSION_CLEAR_PX
                              else '  -> dito davanti alla leva'), flush=True)

        pool = [c for c in valid if c[2]['occ'] >= MIN_OCCLUSION_CLEAR_PX]
        if pool:
            pool.sort(key=lambda item: (item[0], item[1]))
        else:
            print('VIEW SEARCH: il dito copre la leva in TUTTE le candidate valide; '
                  'provo quelle con piu spazio libero (verifica con /tmp/lever_view_XX.png).', flush=True)
            pool = sorted(valid, key=lambda item: -item[2]['occ'])
        selected = pool[:max_n]
        print(f'VIEW SEARCH: {len(valid)} configurazioni IK valide; '
              f'provero al massimo {len(selected)} pose.', flush=True)
        for i, (score, err, view, q, real) in enumerate(selected, 1):
            print(f'  candidata {i}: pitch={real:+.2f}, pixel previsto=({view["pixel"][0]:.0f},{view["pixel"][1]:.0f}), '
                  f'margine={view["margin"]:.0f}px, libero dal dito={view["occ"]:+.0f}px, '
                  f'errore punta={err*100:.2f}cm, giunti={fmt(q)}', flush=True)
        return [tuple(float(v) for v in item[3]) for item in selected]

    def view_preserving_goal(self, state, q_start, original_goal, target, center):
        """Compatibility path used by the existing second approach phase."""
        return self.view_goal_candidates(state, q_start, original_goal,
                                         target, center, max_n=1)[0]

    def fk_tip_camera(self, rs):
        """FK della punta fisica e del frame ottico camera (None se MoveIt non lo conosce)."""
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['ee_finger1', CAMERA_OPTICAL_FRAME]
        req.robot_state = rs
        out = call(self, self.fk, req)
        poses = {nm: ps.pose for nm, ps in zip(out.fk_link_names, out.pose_stamped)}
        if 'ee_finger1' not in poses:
            return np.array(self.fingertip_fk(rs), dtype=float), None
        f = poses['ee_finger1']
        tip = np.array((f.position.x, f.position.y, f.position.z)) + \
              np.array(rot(f.orientation, TCP_LOCAL))
        return tip, poses.get(CAMERA_OPTICAL_FRAME)
 
    def wrist_candidates(self, original, q0, tip0, center, max_n): #se ha i dati dellac amera prova matematicamente tutte le combinazioni possibili di piegaturea del polso definiti sopra. Problema: non viene mai chiamata nei main! 
        """Configurazioni con polso inclinato e punta INVARIATA da cui la leva si vede.

        1) classifica le inclinazioni di ee_base_joint_x/y per margine di visibilita;
        2) per le migliori ricalcola j1..j3 (IK) per riportare la punta dov'era;
        3) tiene solo stati validi in MoveIt con la leva ben dentro l'immagine.
        """
        up = np.array((0.0, 0.0, 1.0))
        pts = [center, center + up*0.02, center - up*0.03]  # perno .. bordo basso della leva
        end = time.monotonic() + 3.0
        while self.cam_K is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        ranked = []
        use_fallback = self.cam_K is None
        if not use_fallback:
            for d4 in WRIST_GRID_RAD:
                for d5 in WRIST_GRID_RAD:
                    if d4 == 0.0 and d5 == 0.0:
                        continue
                    q = np.array(q0, dtype=float)
                    q[3] += d4
                    q[4] += d5
                    if not (BOUNDS_LOW[3] <= q[3] <= BOUNDS_HIGH[3]):
                        continue
                    _, cam = self.fk_tip_camera(put_joints(original, q))
                    if cam is None:
                        use_fallback = True
                        break
                    m = camera_margin(self.cam_K, self.cam_W, self.cam_H, cam, pts)
                    if m >= MIN_VIEW_MARGIN_PX:
                        ranked.append((m, abs(d4)+abs(d5), q))
                if use_fallback:
                    break
        if use_fallback:
            print('  (FK camera non disponibile: provo inclinazioni di polso predefinite)', flush=True)
            for d4, d5 in WRIST_FALLBACK:
                q = np.array(q0, dtype=float)
                q[3] += d4
                q[4] += d5
                if BOUNDS_LOW[3] <= q[3] <= BOUNDS_HIGH[3]:
                    ranked.append((0.0, abs(d4)+abs(d5), q))
        else:
            ranked.sort(key=lambda x: (-min(x[0], 200.0), x[1]))
        out = []
        for m, _, q in ranked[:max_n+3]:
            dist, qc = self.solve_ik(q, q0, tip0, original, stop_tolerance=0.002, free=(0, 1, 2))
            if dist > 0.004:
                continue
            req = GetStateValidity.Request()
            req.robot_state = put_joints(original, qc)
            req.group_name = 'arm'
            if not call(self, self.valid, req).valid:
                continue
            m2 = m
            if not use_fallback:
                _, cam = self.fk_tip_camera(put_joints(original, qc))
                m2 = camera_margin(self.cam_K, self.cam_W, self.cam_H, cam, pts) if cam else -1.0
                if m2 < MIN_VIEW_MARGIN_PX:
                    continue
            out.append((m2, np.array(qc, dtype=float)))
            if len(out) >= max_n:
                break
        return out

    def check_state(self, rs, desc):
        req = GetStateValidity.Request()
        req.robot_state = rs
        req.group_name = 'arm'
        out = call(self, self.valid, req)
        if not out.valid:
            contacts = [(c.contact_body_1, c.contact_body_2) for c in out.contacts[:5]]
            raise RuntimeError(f'{desc} NON valido in MoveIt; contatti={contacts}')

    def check_fingers_above_panel_face(self, rs, panel_origin, panel_normal, clearance=0.002):
        """Use both real collision boxes from rover_arm.urdf.xacro; do not cross panel face."""
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['ee_finger1', 'ee_finger2']
        req.robot_state = rs
        result = call(self, self.fk, req)
        if result.error_code.val != MoveItErrorCodes.SUCCESS or len(result.pose_stamped) != 2:
            raise RuntimeError('FK dita non disponibile per controllo pannello.')
        for pos in result.pose_stamped:
            p = pos.pose.position
            xyz = np.array((p.x,p.y,p.z), dtype=float)
            for x in (-0.005, 0.037):
                for y in (0.0,0.140):
                    for z in (-0.022,0.0):
                        corner = xyz + np.array(rot(pos.pose.orientation, (x,y,z)))
                        if float(np.dot(corner-panel_origin, panel_normal)) < clearance:
                            raise RuntimeError('STOP contatto: volume di un dito attraverserebbe il piano frontale del pannello.')


def distance_from_segment(p, a, b):
    ab = tuple(y-x for x, y in zip(a, b))
    ap = tuple(x-y for x, y in zip(p, a))
    denom = sum(x*x for x in ab)
    if denom < 1e-14:
        return math.dist(p, a)
    t = max(0.0, min(1.0, sum(x*y for x, y in zip(ap, ab)) / denom))
    projection = tuple(x+t*y for x, y in zip(a, ab))
    return math.dist(p, projection)


def open_gripper(node, position=GRIPPER_OPEN_POS):
    """Apre le dita (slider_joint) per liberare la vista della camera.

    La camera e' solidale al polso e le dita, da chiuse, coprono la leva
    nella posa di pre-approach (vedi /tmp/lever_view_*.png). Aprire le dita
    le allontana dall'asse ottico senza cambiare la punta di contatto
    (ee_finger1 resta il riferimento di FK).
    """
    from trajectory_msgs.msg import JointTrajectoryPoint

    client = ActionClient(node, FollowJointTrajectory,
                          '/gripper_controller/follow_joint_trajectory')
    if not client.wait_for_server(timeout_sec=5.0):
        print('GRIPPER: action /gripper_controller non disponibile; '
              "dita lasciate com'erano.", flush=True)
        return False
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['slider_joint']
    point = JointTrajectoryPoint()
    point.positions = [float(position)]
    point.time_from_start = duration_msg(1.5)
    goal.trajectory.points = [point]
    goal.goal_time_tolerance = duration_msg(2.0)
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    if not future.done() or future.exception() or not future.result().accepted:
        print('GRIPPER: apertura rifiutata dal controller; continuo.', flush=True)
        return False
    result = future.result().get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=10.0)
    print(f'GRIPPER: dita aperte a slider_joint={position:.3f} m.', flush=True)
    time.sleep(0.5)
    return True


def verify_gazebo_controller(node):
    # Check two independent pieces of evidence for simulated controller.
    out = subprocess.run(['gz', 'service', '-l'], capture_output=True,
                         text=True, timeout=10, check=False)
    if out.returncode != 0 or '/world/panel_demo/' not in out.stdout:
        raise RuntimeError('Mondo Gazebo panel_demo non rilevato: esecuzione vietata.')
    cli = node.create_client(GetParameters, '/controller_manager/get_parameters')
    if not cli.wait_for_service(timeout_sec=6):
        raise RuntimeError('controller_manager non disponibile.')
    req = GetParameters.Request()
    req.names = ['use_sim_time']
    ans = call(node, cli, req, timeout=7).values
    if not ans or ans[0].type != ParameterType.PARAMETER_BOOL or not ans[0].bool_value:
        raise RuntimeError('controller_manager NON confermato in tempo simulato.')
    controller_cli = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if not controller_cli.wait_for_service(timeout_sec=5):
        raise RuntimeError('list_controllers non disponibile.')
    controllers = call(node, controller_cli, ListControllers.Request(), timeout=7).controller
    arm = [c for c in controllers if c.name == 'arm_controller']
    if len(arm) != 1 or arm[0].state != 'active' or 'JointTrajectoryController' not in arm[0].type:
        raise RuntimeError('Il controller del braccio Gazebo non e attivo/atteso.')
    print('SICUREZZA: Gazebo panel_demo + arm_controller attivo in sim time confermati.', flush=True)


def retime_trajectory(joint_trajectory, contact_only=False):
    """Temporizza da zero il percorso *geometrico* fornito da OMPL.

    Il servizio GetMotionPlan puo' restituire waypoint non temporizzati
    (per esempio t=0 per tutti i punti). Non modifichiamo gli angoli ne'
    i segmenti pianificati: imponiamo nuovi tempi conservativi ed
    interpoliamo lungo ogni segmento senza superare i suoi estremi.

    Il controller Jazzy usa interpolazione lineare per waypoint contenenti
    solo posizioni, per cui la traiettoria temporizzata rimane sullo stesso
    percorso in joint-space. La funzione NON invia alcun movimento.
    """
    from trajectory_msgs.msg import JointTrajectoryPoint

    traj = copy.deepcopy(joint_trajectory)
    original = list(traj.points)
    if len(original) < 2:
        raise RuntimeError('Servono almeno due punti OMPL per il movimento.')

    names = list(traj.joint_names)
    if len(names) != len(JOINTS) or set(names) != set(JOINTS):
        raise RuntimeError('Piano OMPL con giunti inattesi: NON invio al controller.')
    for p in original:
        if len(p.positions) != len(names) or not all(math.isfinite(v) for v in p.positions):
            raise RuntimeError('Waypoint OMPL con posizioni mancanti o non finite.')

    input_times = [duration_seconds(p.time_from_start) for p in original]
    input_timed = (all(math.isfinite(t) and t >= 0 for t in input_times)
                   and all(b > a for a,b in zip(input_times, input_times[1:])))
    if not input_timed:
        print('INFO: punti OMPL senza tempi strettamente crescenti: '
              'ricostruisco i tempi dal percorso geometrico.', flush=True)
    else:
        print('INFO: ritimo lentamente anche il piano OMPL temporizzato.', flush=True)

    # Limiti cautelativi per la *traiettoria comandata* in simulazione;
    # non sostituiscono i limiti dinamici del modello / la verifica fisica.
    max_speed = 0.300 if contact_only else 0.200  # originale invariato nelle fasi 1 e 2
    max_accel = 0.400 if contact_only else 0.250
    segment_dt = []
    for a, b in zip(original, original[1:]):
        max_delta = max(abs(v-u) for u,v in zip(a.positions, b.positions))
        if not math.isfinite(max_delta):
            raise RuntimeError('Incremento angolare non finito nel piano OMPL.')
        # Smoothstep f(u)=3u^2-2u^3: max f'=1.5, max |f''|=6.
        dt = max(3.0, 1.5*max_delta/max_speed,
                 math.sqrt(6.0*max_delta/max_accel))
        segment_dt.append(dt)
    duration = sum(segment_dt)
    min_duration = 2.0 if contact_only else 3.0
    if duration < min_duration:
        scale = min_duration/duration
        segment_dt = [dt*scale for dt in segment_dt]
        duration = sum(segment_dt)
    if not (0.0 < duration <= 90.0):
        raise RuntimeError(f'Durata conservativa non valida ({duration:.2f} s).')

    first = JointTrajectoryPoint()
    first.positions = [float(v) for v in original[0].positions]
    first.time_from_start = duration_msg(0.0)
    new_points = [first]
    elapsed = 0.0
    for seg, dt in enumerate(segment_dt):
        qa = original[seg].positions
        qb = original[seg+1].positions
        max_delta = max(abs(b-a) for a,b in zip(qa,qb))
        subdivisions = max(25, math.ceil(max_delta/0.015))
        for step in range(1, subdivisions+1):
            u = step/subdivisions
            smooth = u*u*(3.0-2.0*u)
            pt = JointTrajectoryPoint()
            pt.positions = [float(a+(b-a)*smooth) for a,b in zip(qa,qb)]
            pt.time_from_start = duration_msg(elapsed+u*dt)
            new_points.append(pt)
        elapsed += dt

    # Verifiche esplicite sul comando che sara' inviato, non sui tempi OMPL.
    generated = [duration_seconds(p.time_from_start) for p in new_points]
    if any(b <= a for a,b in zip(generated,generated[1:])):
        raise RuntimeError('BUG retiming: tempi generati non crescenti. STOP.')
    if any(len(p.velocities) or len(p.accelerations) for p in new_points):
        raise RuntimeError('BUG retiming: campi velocita/accelerazione inattesi. STOP.')
    if list(new_points[0].positions) != list(original[0].positions):
        raise RuntimeError('BUG retiming: configurazione iniziale diversa. STOP.')
    if max(abs(a-b) for a,b in zip(new_points[-1].positions, original[-1].positions)) > 1e-9:
        raise RuntimeError('BUG retiming: configurazione finale diversa. STOP.')
    # Conserviamo solo posizioni, ordinate per i 5 giunti validati.
    traj.points = new_points
    traj.header.stamp.sec = 0
    traj.header.stamp.nanosec = 0
    return traj, duration


def execute_in_gazebo(node, traj, target, initial_q, standoff_m=STANDOFF_M,
                       frozen_panel=None, contact_only=False):
    verify_gazebo_controller(node)
    # Recheck that nothing has moved while the services were running.
    q_now = q_from_state(node.get_state())
    delta = max(abs(a-b) for a,b in zip(initial_q,q_now))
    if delta > 0.025:
        raise RuntimeError(f'Stato cambiato durante la pianificazione ({delta:.3f} rad): STOP.')
    if frozen_panel is None:
        raise RuntimeError('Geometria pannello congelata mancante: STOP.')
    # La camera puo' perdere gli ArUco durante il movimento; il rover e' fermo
    # (verificato) e la geometria e' quella misurata prima del primo movimento.
    center, normal = frozen_panel
    new_target = tuple(a+standoff_m*n for a,n in zip(center, normal))
    if math.dist(new_target, target) > 0.010:
        raise RuntimeError('Target incoerente con la geometria congelata: STOP.')

    controller = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    if not controller.wait_for_server(timeout_sec=8.0):
        raise RuntimeError('Action controller Gazebo non disponibile.')
    safe_traj, duration = retime_trajectory(traj, contact_only=contact_only)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = safe_traj
    goal.goal_time_tolerance = duration_msg(4.0)
    print(f'ESECUZIONE CONTROLLATA: {len(traj.points)} waypoint OMPL, '
          f'{len(safe_traj.points)} punti ricampionati lungo lo stesso percorso, '
          f'durata {duration:.1f} s, SOLO controller Gazebo.',flush=True)
    print('Se appare movimento anomalo, metti immediatamente in pausa Gazebo.', flush=True)
    f = controller.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    if not f.done() or f.exception():
        raise RuntimeError('Controller non ha risposto alla richiesta.')
    handle = f.result()
    if not handle.accepted:
        raise RuntimeError('Controller ha rifiutato il goal.')
    result_future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=max(120.0, duration*6.0))
    except KeyboardInterrupt:
        print('Interruzione richiesta, tento di cancellare il goal...',flush=True)
        f_cancel = handle.cancel_goal_async()
        try:
            rclpy.spin_until_future_complete(node, f_cancel, timeout_sec=4.0)
        except Exception:
            pass
        raise
    if not result_future.done():
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node,cancel,timeout_sec=4)
        raise RuntimeError('Timeout controller; cancellazione richiesta. Controllare Gazebo.')
    out = result_future.result()
    print(f'CONTROLLER: status={out.status}, error_code={out.result.error_code}, '
          f'msg={out.result.error_string}',flush=True)
    if out.status != GoalStatus.STATUS_SUCCEEDED or out.result.error_code != 0:
        raise RuntimeError('Esecuzione non confermata dal controller.')
    time.sleep(0.3)
    current = node.get_state()
    current_q = q_from_state(current)
    last_q = tuple(safe_traj.points[-1].positions[list(safe_traj.joint_names).index(j)] for j in JOINTS)
    qerr = max(abs(a-b) for a,b in zip(last_q,current_q))
    tip = node.fingertip_fk(current)
    err = math.dist(tip,target)
    print('GIUNTI FINALI LETTI:',fmt(current_q),flush=True)
    print('FK PUNTA REALE FINALE:',fmt(tip),flush=True)
    print(f'ERRORE GIUNTI: {qerr:.4f} rad; errore punta/target LIVE: {err*100:.2f} cm',flush=True)
    if qerr > (0.018 if contact_only else 0.035) or err > (0.007 if contact_only else 0.025):
        print('ATTENZIONE: il controller ha riportato successo, ma lo stato finale e diverso dal previsto.',flush=True)
        return 3
    print('SUCCESS: il controller Gazebo ha eseguito il piano e la punta e vicina al target.',flush=True)
    return 0




def rover_pose_in_odom(node):
    """Check rover has NOT moved between perception and phase 2."""
    if not node.tf_buffer.can_transform('odom', 'base_link', Time()):
        raise RuntimeError('TF odom->base_link assente: seconda fase vietata.')
    t = node.tf_buffer.lookup_transform('odom', 'base_link', Time()).transform
    p = np.array((t.translation.x, t.translation.y, t.translation.z), dtype=float)
    q = np.array((t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w), dtype=float)
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 0.9:
        raise RuntimeError('Posa rover in odom non valida: STOP.')
    return p, q / np.linalg.norm(q)

def require_rover_stationary(node, prior):
    current = rover_pose_in_odom(node)
    dist = float(np.linalg.norm(current[0] - prior[0]))
    dot = float(np.clip(abs(np.dot(current[1], prior[1])), 0.0, 1.0))
    angle = 2.0 * math.acos(dot)
    if dist > 0.01 or angle > 0.03:
        raise RuntimeError(f'Rover mosso tra misura e seconda fase ({dist:.3f} m, {angle:.3f} rad): STOP.')


def plan_and_maybe_execute(n, target, args, original, q_start, start_tip,
                           frozen_panel, standoff_m, is_second=False,
                           contact_phase=False, phase_label=None, panel_face=None,
                           joint_goal=None):
    """IK e OMPL per una fase; la seconda parte dalla configurazione raggiunta nella prima."""
    phase = phase_label or ('FASE 2 - AVVICINAMENTO FINALE' if is_second else 'FASE 1 - PRE-APPROACH')
    print('\n' + '=' * 60 + '\n' + phase + '\n' + '=' * 60, flush=True)
    if joint_goal is not None:
        GOAL_JOINTS = tuple(float(v) for v in joint_goal)
        print('Giunti ARRIVO imposti (aggiustamento polso):', fmt(GOAL_JOINTS), flush=True)
    else:
        GOAL_JOINTS = n.find_joint_goal(q_start, target, original, precision=contact_phase)
        if (args.keep_visible == 'true' and not contact_phase
                and phase_label is None and joint_goal is None):
            # Vision-oriented IK BEFORE moving: keep the camera pointed toward
            # the already measured main_switch while preserving fingertip target.
            # NB: NON si applica quando arriva una candidata esplicita da
            # VIEW SEARCH, altrimenti si perde il roll scelto per la visibilita'.
            GOAL_JOINTS = n.view_preserving_goal(
                original, q_start, GOAL_JOINTS, target, frozen_panel[0])
    print('Giunti ARRIVO trovati dalla IK LIVE:', fmt(GOAL_JOINTS),flush=True)
    final_state = put_joints(original, GOAL_JOINTS)
    n.check_state(final_state, 'Stato finale')
    tip_goal = n.fingertip_fk(final_state)
    err = math.dist(tip_goal, target)
    print('FK punta per GIUNTI ARRIVO:', fmt(tip_goal), flush=True)
    print(f'ERRORE DELLA PUNTA: {err*100:.2f} cm', flush=True)
    if err > (.004 if contact_phase else MAX_GOAL_TIP_ERROR_M):
        raise RuntimeError('La configurazione IK non porta la punta sul target LIVE. STOP.')
    
    req = GetMotionPlan.Request()
    motion = req.motion_plan_request
    motion.group_name = 'arm'
    motion.pipeline_id = 'ompl'
    motion.allowed_planning_time = 12.0
    motion.num_planning_attempts = 3
    motion.max_velocity_scaling_factor = 0.10
    motion.max_acceleration_scaling_factor = 0.10
    # Stato esplicito: niente interpolazione da una posa iniziale immaginaria.
    motion.start_state = copy.deepcopy(original)
    motion.start_state.is_diff = False
    motion.workspace_parameters.header.frame_id = 'base_link'
    for key in ('x','y','z'):
        setattr(motion.workspace_parameters.min_corner, key, -3.0)
        setattr(motion.workspace_parameters.max_corner, key, +3.0)
    goal_constraints = Constraints()
    goal_constraints.name = 'IK_punta_verificata_joint_goal'
    for joint, angle in zip(JOINTS, GOAL_JOINTS):
        c = JointConstraint()
        c.joint_name = joint
        c.position = float(angle)
        c.tolerance_above = 0.008
        c.tolerance_below = 0.008
        c.weight = 1.0
        goal_constraints.joint_constraints.append(c)
    motion.goal_constraints = [goal_constraints]
    print('\nRICHIESTA A OMPL (solo pianificazione; esecuzione subordinata ai controlli).', flush=True)
    result = call(n, n.plan, req, timeout=35.0).motion_plan_response
    print(f'RESULT OMPL: error_code={result.error_code.val}, '
          f'planning_time={result.planning_time:.2f} s', flush=True)
    if result.error_code.val != MoveItErrorCodes.SUCCESS:
        print('ESITO: PIANIFICAZIONE FALLITA. IK finale gia verificata; '
              'analizzare i messaggi OMPL del launch.', flush=True)
        return 1, None
    traj = result.trajectory.joint_trajectory
    names = list(traj.joint_names)
    points = list(traj.points)
    if not points:
        raise RuntimeError('MoveIt ha segnalato successo ma la traiettoria e vuota.')
    if not all(j in names for j in JOINTS):
        raise RuntimeError('Traiettoria senza tutti i 5 giunti del braccio.')
    def to_q(point):
        return tuple(float(point.positions[names.index(j)]) for j in JOINTS)
    q_first = to_q(points[0])
    q_last = to_q(points[-1])
    print(f'PUNTI TRAIETTORIA: {len(points)}', flush=True)
    print('Prima configurazione piano:', fmt(q_first), flush=True)
    print('Ultima configurazione piano:', fmt(q_last), flush=True)
    print('Scarto primo punto dallo stato reale:',
          f'{max(abs(x-y) for x,y in zip(q_first,q_start)):.4f} rad', flush=True)
    if max(abs(x-y) for x,y in zip(q_first,q_start)) > 0.06:
        raise RuntimeError('Il piano non parte dallo stato che abbiamo misurato.')
    if max(abs(x-y) for x,y in zip(q_last,GOAL_JOINTS)) > 0.017:
        raise RuntimeError('OMPL ha terminato su angoli diversi dal goal IK.')
    fk_final = n.fingertip_fk(put_joints(original, q_last))
    err_final = math.dist(fk_final, target)
    print('FK PUNTA a fine piano:', fmt(fk_final), flush=True)
    print(f'DISTANZA FINALE PUNTA/TARGET: {err_final*100:.2f} cm', flush=True)
    if err_final > (.004 if contact_phase else MAX_GOAL_TIP_ERROR_M):
        raise RuntimeError('La traiettoria termina lontano dal target della punta.')
    
    # Ricontrolla dettagliatamente il percorso, interpolando ogni segmento
    # fino a un massimo di 0.02 rad per giunto. E' un controllo campionato,
    # NON una prova formale di sicurezza continua.
    center, normal = frozen_panel
    detour = 0.0
    checked = 0
    min_standoff = float('inf')
    for idx in range(len(points)-1):
        qa, qb = to_q(points[idx]), to_q(points[idx+1])
        k = max(1,math.ceil(max(abs(a-b) for a,b in zip(qa,qb))/0.02))
        for step in range(k+1):
            alpha = step/k
            q = tuple(a+(b-a)*alpha for a,b in zip(qa,qb))
            rs = put_joints(original, q)
            n.check_state(rs, f'Segmento {idx} punto {step}/{k}')
            if contact_phase and panel_face is not None:
                n.check_fingers_above_panel_face(rs, *panel_face, clearance=CONTACT_FACE_CLEARANCE_M)
            tip_p = n.fingertip_fk(rs)
            detour = max(detour,distance_from_segment(tip_p,start_tip,target))
            signed_standoff = sum((tip_p[i]-center[i])*normal[i] for i in range(3))
            min_standoff = min(min_standoff,signed_standoff)
            checked+=1
    print(f'CHECK PERCORSO: {checked} stati interpolati validi (campionamento, non garanzia continua).',flush=True)
    if contact_phase:
        print(f'PERCORSO CONTATTO: deviazione={detour*100:.1f}cm; distanza dal riferimento di controllo={min_standoff*100:.1f}cm; ingombri dita verificati.',flush=True)
    else:
        print(f'DEVIAZIONE MASSIMA PUNTA: {detour*100:.1f} cm; MIN DISTANZA NORMALE dalla leva/piano: {min_standoff*100:.1f} cm',flush=True)
    if detour > (0.025 if is_second else MAX_DETOUR_M):
        raise RuntimeError('Percorso fa una deviazione troppo ampia: non eseguo.')
    if min_standoff < (-0.003 if contact_phase else 0.04):
        raise RuntimeError('La punta supera il limite di avvicinamento di questa fase: STOP.')
    # Il controllo del retiming vale anche in modalita' dry-run.
    # Prima ci bloccavamo in --execute su tempi OMPL non temporizzati.
    dry_traj, dry_duration = retime_trajectory(traj, contact_only=(contact_phase or phase_label is not None))
    print(f'RETIMING CONTROLLATO: {len(dry_traj.points)} punti, '
          f'{dry_duration:.1f} s, tempi strettamente crescenti; '
          f'prima/ultima posa invariate.', flush=True)
    if not args.execute:
        print('ESITO: PIANO VALIDATO; NON ESEGUITO (solo simulazione).', flush=True)
        return 0, q_last
    code = execute_in_gazebo(n, traj, target, q_start,
                             standoff_m=standoff_m,
                             frozen_panel=frozen_panel,
                             contact_only=(contact_phase or phase_label is not None))
    return code, q_last



def search_visible_arrival(n, args, target, original, q_start, start_tip, frozen, depth):
    """Gazebo-only trial of multiple *valid* 8-cm arrival configurations.

    No pose is accepted from projection alone. Each motion must pass the
    existing MoveIt, TCP, collision, detour, and controller checks; after a
    completed motion, four new RGB-D frames must agree on the visible lever.
    A controller error or unexpected state change causes an immediate STOP.
    """
    print('\n' + '='*60 + '\nFASE 1 - RICERCA CONFIGURAZIONE CON LEVA VISIBILE'
          + '\n' + '='*60, flush=True)
    first_goal = n.find_joint_goal(q_start, target, original)
    goals = n.view_goal_candidates(original, q_start, first_goal, target,
                                    frozen[0], max_n=args.max_wrist_tries)
    rover_before = rover_pose_in_odom(n)
    for index, goal in enumerate(goals, 1):
        print(f'\nVIEW SEARCH - PROVA {index}/{len(goals)}', flush=True)
        require_rover_stationary(n, rover_before)
        state = n.get_state() if args.execute else original
        q_now = q_from_state(state) if args.execute else q_start
        tip_now = n.fingertip_fk(state) if args.execute else start_tip
        n.check_state(state, 'Stato iniziale della ricerca')
        try:
            code, goal_q = plan_and_maybe_execute(
                n, target, args, state, q_now, tip_now, frozen,
                STANDOFF_M, joint_goal=goal)
        except RuntimeError as exc:
            # Only retry if the failed planning request produced NO movement.
            actual_q = q_from_state(n.get_state()) if args.execute else q_now
            if args.execute and max(abs(a-b) for a,b in zip(q_now,actual_q)) > .025:
                raise RuntimeError('VIEW SEARCH: errore con movimento inatteso; '
                                   'non provo altre pose.') from exc
            print(f'VIEW SEARCH: candidata non pianificabile: {exc}', flush=True)
            continue
        if code == 1:
            print('VIEW SEARCH: OMPL non ha trovato un percorso; provo la prossima.', flush=True)
            continue
        if code != 0:
            raise RuntimeError('VIEW SEARCH: esecuzione incerta; non provo altre pose.')
        if not args.execute:
            print('VIEW SEARCH: piano trovato. In --plan-only non posso confermare '
                  'la visibilita RGB-D dalla posa finale.', flush=True)
            return 0, goal_q
        time.sleep(.4)
        require_rover_stationary(n, rover_before)
        q_settled = q_from_state(n.get_state())
        time.sleep(.4)
        q_check = q_from_state(n.get_state())
        if max(abs(a-b) for a,b in zip(q_settled, q_check)) > MAX_START_STILL_RAD:
            raise RuntimeError('VIEW SEARCH: braccio ancora in movimento: STOP.')
        print('VIEW SEARCH: misuro la leva con RGB e depth NUOVE...', flush=True)
        try:
            depth.observe(seconds=6.0)
        except RuntimeError as exc:
            print(f'VIEW SEARCH: posa {index} non confermata dalla RGB-D: {exc}', flush=True)
            # Gazebo only: the rover and frozen target must still be stationary.
            # Alternate paths receive the same safety validations as the first.
            continue
        print(f'VIEW SEARCH SUCCESS: leva verificata da 4 RGB-D nuove, '
              f'configurazione {index}: {fmt(q_check)}', flush=True)
        print('VIEW SEARCH TERMINATA: passo alla fase successiva prevista dal launch.',flush=True)
        return 0, q_check
    raise RuntimeError('VIEW SEARCH: nessuna delle configurazioni provate ha '
                       'prodotto una misura RGB-D valida. STOP: nessuna fase 2.')


class FreshDepthObserver:
    """Independently measure the red main-switch with NEW RGB and depth frames.

    Does NOT change the original 12/12 detector or reuse its cached samples.
    Uses its first measured point only for ROI projection and plausibility gating.
    """
    def __init__(self, node, expected_base):
        self.n = node
        self.old = np.asarray(expected_base, dtype=float)
        self.bridge = CvBridge()
        self.caminfo = None
        self.depth_frames = []
        self.results = []
        self.seen_rgb = set()
        self.sim_ns = 0
        self.min_ns = None
        self.last_error = 'nessun frame elaborato'
        self.debug_idx = 0          # contatore dei PNG di debug in /tmp
        self.debug_saved = False    # un solo PNG per ogni observe()
        node.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, '/camera/rgbd/camera_info', self.on_info, qos_profile_sensor_data)
        node.create_subscription(Image, '/camera/rgbd/depth_image', self.on_depth, qos_profile_sensor_data)
        node.create_subscription(Image, '/camera/rgbd/image', self.on_rgb, qos_profile_sensor_data)

    @staticmethod
    def stamp_ns(stamp):
        return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)

    def on_clock(self, msg):
        self.sim_ns = self.stamp_ns(msg.clock)

    def on_info(self, msg):
        K = np.asarray(msg.k, dtype=float).reshape(3,3)
        if np.all(np.isfinite(K)) and K[0,0] > 0 and K[1,1] > 0:
            self.caminfo = (K, (np.asarray(msg.d,dtype=float).reshape(-1,1) if len(msg.d) else np.zeros((5,1))),
                            int(msg.width),int(msg.height))

    def on_depth(self, msg):
        s = self.stamp_ns(msg.header.stamp)
        if self.min_ns is None or s < self.min_ns:
            return
        try:
            depth = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
            if msg.encoding == '16UC1':
                depth = depth.astype(np.float32)*0.001
            elif msg.encoding == '32FC1':
                depth = depth.astype(np.float32)
            else:
                self.last_error = 'depth encoding inatteso: '+str(msg.encoding)
                return
            self.depth_frames.append((s, depth, msg.header.frame_id))
            self.depth_frames = self.depth_frames[-10:]
        except Exception as e:
            self.last_error = f'conversione depth: {e}'

    def on_rgb(self, msg):
        s = self.stamp_ns(msg.header.stamp)
        if self.min_ns is None or s < self.min_ns or s in self.seen_rgb:
            return
        self.seen_rgb.add(s)
        if not self.depth_frames or self.caminfo is None:
            return
        dtime, depth, dframe = min(self.depth_frames, key=lambda x: abs(x[0]-s))
        if abs(s-dtime) > 400_000_000 or dtime < self.min_ns:
            self.last_error = 'RGB/depth fuori sincronismo (>0.18 s)'
            return
        if dframe and msg.header.frame_id and dframe != msg.header.frame_id:
            self.last_error = f'frame RGB/depth differenti: {msg.header.frame_id}/{dframe}'
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if bgr.shape[:2] != depth.shape[:2]:
                self.last_error = 'RGB e depth di risoluzioni differenti: non registrate'
                return
            new = self.measure_one(bgr, depth, msg.header.frame_id, msg.header.stamp)
            if new is not None:
                self.results.append((s, *new))
                self.results = self.results[-12:]
        except Exception as e:
            self.last_error = f'osservazione RGB-D scartata: {e}'

    def save_debug(self, bgr, u, v, reason):
        """Salva UN'immagine della camera per ogni observe() fallita, col pixel previsto della leva."""
        if self.debug_saved:
            return
        self.debug_saved = True
        try:
            img = bgr.copy()
            pt = (int(round(u)), int(round(v)))
            cv2.circle(img, pt, 32, (0, 255, 0), 2)
            cv2.drawMarker(img, pt, (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
            cv2.putText(img, reason[:60], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            path = f'/tmp/lever_view_{self.debug_idx:02d}.png'
            self.debug_idx += 1
            if cv2.imwrite(path, img):
                print(f'DEBUG: immagine camera col pixel previsto della leva salvata in {path}', flush=True)
        except Exception as e:
            print(f'DEBUG: salvataggio immagine fallito: {e}', flush=True)

    def measure_one(self, bgr, depth, optical_frame, stamp):
        K, D, width, height = self.caminfo
        H,W = depth.shape[:2]
        if not optical_frame or (width and W != width) or (height and H != height):
            self.last_error = 'CameraInfo/immagine/frame non coerenti'
            return None
        # Sensor-stamped TF if present; latest only when manipulator is verified stationary.
        try:
            tf = self.n.tf_buffer.lookup_transform('base_link', optical_frame, Time.from_msg(stamp))
        except Exception:
            try:
                tf = self.n.tf_buffer.lookup_transform('base_link', optical_frame, Time())
            except Exception:
                self.last_error = 'TF camera -> base_link assente'
                return None
        q = tf.transform.rotation
        position = tf.transform.translation
        t = np.array((position.x,position.y,position.z),float)
        # R columns are local optical axes in base_link.
        axes = np.column_stack([np.array(rot(q, tuple(np.eye(3)[i]))) for i in range(3)])
        camera_pt = axes.T @ (self.old-t)
        if camera_pt[2] <= 0.04:
            self.last_error = 'Leva dietro la camera o troppo vicina'
            return None
        u,v = cv2.projectPoints(camera_pt.reshape(1,3), np.zeros(3), np.zeros(3), K, D)[0].reshape(-1,2)[0]
        # Salva la prima immagine RGB-D elaborata
        # anche quando il riconoscimento funziona
        if not self.debug_saved:
            self.save_debug(
                bgr,
                u,
                v,
                f'CHECK RGB-D z={camera_pt[2]:.2f}m'
            )
        if not (20 <= u < W-20 and 20 <= v < H-20):
            self.last_error = f'Leva fuori FOV: pixel previsto=({u:.0f}, {v:.0f})'
            self.save_debug(bgr, u, v, f'fuori FOV z={camera_pt[2]:.2f}m')
            return None
        radius = 160
        x0=max(0,int(u)-radius);x1=min(W,int(u)+radius+1)
        y0=max(0,int(v)-radius);y1=min(H,int(v)+radius+1)
        hsv=cv2.cvtColor(bgr[y0:y1,x0:x1], cv2.COLOR_BGR2HSV)
        mask=(cv2.inRange(hsv,(0,75,28),(14,255,255)) |
              cv2.inRange(hsv,(172,75,28),(179,255,255)))
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((2,2),np.uint8))
        n,labels,stats,centers=cv2.connectedComponentsWithStats(mask,8)
        candidates=[]
        for i in range(1,n):
            lx,ly,w,h,area=(int(x) for x in stats[i])
            du=x0+float(centers[i,0])-u;dv=y0+float(centers[i,1])-v
            d=math.hypot(du,dv)
            # 80 px: a 8 cm la leva rossa occupa ~60x45 px e il suo centroide
            # puo' stare ~60 px dal pixel previsto (che cade sull'estremo).
            if area>=12 and h>=4 and d<=80:
                candidates.append((d,i,lx,ly,w,h,area))
        if not candidates:
            self.last_error='Rosso della leva non riconoscibile nella nuova RGB'
            self.save_debug(bgr, u, v, f'rosso assente z={camera_pt[2]:.2f}m')
            return None
        candidates.sort()
        _,label,lx,ly,w,h,_=candidates[0]
        comp=(labels[ly:ly+h,lx:lx+w]==label)
        gy,gx=np.nonzero(comp)
        gx=gx+x0+lx;gy=gy+y0+ly
        zz=depth[gy,gx]
        valid=np.isfinite(zz)&(zz>0.05)&(zz<1.5)&(np.abs(zz-camera_pt[2])<0.07)
        if np.count_nonzero(valid)<12:
            self.last_error='Troppi pochi pixel rossi con profondita valida'
            return None
        gx,gy,zz=gx[valid],gy[valid],zz[valid].astype(float)
        if np.median(np.abs(zz-np.median(zz)))>0.012:
            self.last_error='Profondita rossa non omogenea'
            return None
        # The colored lever must protrude beyond the panel/background.
        b0=max(0,x0+lx-7);b1=min(W,x0+lx+w+7)
        a0=max(0,y0+ly-7);a1=min(H,y0+ly+h+7)
        bg=depth[a0:a1,b0:b1].astype(float)
        bgmask=np.ones(bg.shape,bool)
        bgmask[y0+ly-a0:y0+ly+h-a0, x0+lx-b0:x0+lx+w-b0]=False
        surrounding=bg[bgmask]
        surrounding=surrounding[np.isfinite(surrounding)&(surrounding>.05)&(surrounding<1.5)]
        relief=float(np.median(surrounding)-np.median(zz)) if len(surrounding)>15 else 0.0
        if relief < .004 or relief > .12:
            self.last_error=f'Rosso senza sporgenza verificata: {relief*1000:.1f} mm'
            return None
        pixel=np.column_stack((gx,gy)).astype(float).reshape(-1,1,2)
        rays=cv2.undistortPoints(pixel,K,D).reshape(-1,2)
        cam_points=np.column_stack((rays[:,0]*zz,rays[:,1]*zz,zz))
        world=cam_points @ axes.T+t
        median=np.median(world,axis=0)
        if float(np.linalg.norm(median-self.old))>.035:
            self.last_error='Nuova RGB-D troppo diversa dalla posizione iniziale (>3.5 cm)'
            return None
        # Lower portion of the 3D red surface, not a nominal Xacro coordinate.
        limit=float(np.quantile(world[:,2],0.24))
        lower=np.median(world[world[:,2]<=limit],axis=0)
        if float(np.linalg.norm(lower-median))>.045:
            self.last_error='Estremita inferiore incoerente con la leva'
            return None
        return median,lower

    def observe(self, seconds=9.0):
        self.results=[];self.seen_rgb=set();self.depth_frames=[]
        self.min_ns=None
        self.debug_saved=False
        # Wait for /clock THEN reject frames whose stamp predates this observation.
        clock_before=self.sim_ns
        deadline=time.monotonic()+3.0
        while (self.sim_ns<=0 or self.sim_ns<=clock_before) and time.monotonic()<deadline:
            rclpy.spin_once(self.n, timeout_sec=.10)
        if self.sim_ns<=clock_before:
            raise RuntimeError('Non ricevo /clock: impossibile validare la freschezza depth.')
        self.min_ns=self.sim_ns+1
        print('RIOSSERVAZIONE: solo RGB e depth con timestamp SIM successivo allo STOP.',flush=True)
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self.n,timeout_sec=.09)
            if len(self.results)>=4:
                last=self.results[-4:]
                # Distinct new images: already enforced by stamp.
                mids=np.array([c for _,c,_ in last])
                lows=np.array([b for _,_,b in last])
                center=np.median(mids,axis=0)
                tip=np.median(lows,axis=0)
                scatter=max(np.max(np.linalg.norm(mids-center,axis=1)),
                            np.max(np.linalg.norm(lows-tip,axis=1)))
                if scatter<=.009:
                    print(f'NUOVA MISURA RGB-D (4 frame distinti): centro={fmt(center)} '
                          f'parte inferiore={fmt(tip)}; scatter={scatter*1000:.1f} mm',flush=True)
                    return center,tip
                self.last_error=f'4 campioni non stabili, dispersione {scatter*1000:.1f} mm'
        raise RuntimeError('RIOSSERVAZIONE NON RIUSCITA: '+self.last_error)


def lever_visible_now(depth, seconds=4.0):
    """Verifica REALE (RGB + depth nuove): il rosso della leva si misura e sporge dal pannello."""
    try:
        depth.observe(seconds=seconds)
        return True
    except RuntimeError as e:
        print('   leva non misurabile:', e, flush=True)
        return False


def ensure_lever_visible(n, args, depth, frozen, tag):
    """Stop at once if NEW RGB-D cannot see the switch after movement.

    No more blind wrist corrections; visibility is planned before moving.
    Detector, marker selection and RGB-D measurement functions stay untouched.
    """
    if args.keep_visible != 'true' or not args.execute:
        return None
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(0.6)
    print(f'VISION GUARD (RGB-D DOPO {tag}): verifico realmente la leva...',flush=True)
    if not lever_visible_now(depth, seconds=6.0):
        raise RuntimeError('VISION GUARD: main_switch NON visibile dopo il movimento. '
                           f'STOP senza altre manovre; motivo RGB-D: {depth.last_error}')
    print('VISION GUARD OK: main_switch misurato in RGB e depth NUOVE.',flush=True)
    return None


def perform_contact_extension(n, args, frozen, rover_initial, observed_initial, depth):
    """Dopo 8 -> 5 cm: ristima la leva con RGB-D NUOVE, poi la solleva dal basso.

    Comanda SOLO /arm_controller (mai il giunto del pannello). Il contatto e'
    un esperimento in simulazione, non una garanzia di azionamento.
    """
    if not args.execute:
        print('SOLO PLAN: fase di ri-osservazione RGB-D richiede la camera ferma IN GAZEBO; STOP normale.',flush=True)
        return 0
    if args.final_approach != 'true':
        raise RuntimeError('--attempt-contact richiede --final-approach true')
    require_rover_stationary(n, rover_initial)
    before=n.main_switch_angle
    n.main_switch_peak=before
    if before is None or not (-1.70<before<-1.35):
        raise RuntimeError('main_switch non e in posizione OFF: contatto non autorizzato.')
    center0,normal0=(np.asarray(v,float) for v in frozen)
    up=np.array((0.0,0.0,1.0))
    # L'unico prior di posizione e' la leva misurata all'inizio (solo per la ROI).
    print('\nFASE 3: ristima della leva con RGB-D NUOVE dopo l avvicinamento a '
          f'{args.final_standoff*100:.1f} cm.',flush=True)
    q_before_obs=q_from_state(n.get_state())
    time.sleep(.45)
    if max(abs(a-b) for a,b in zip(q_before_obs,q_from_state(n.get_state()))) > .010:
        raise RuntimeError('Braccio non fermo prima della nuova depth: STOP.')
    rclpy.spin_once(n,timeout_sec=.2)
    try:
        measure,lower=depth.observe(seconds=7.0)
    except RuntimeError as first_error:
        if args.camera_lift<=0:
            raise
        print(f'Prima ri-osservazione fallita: {first_error}. Provo UN SOLO micro-sollevamento '+
              f'di {args.camera_lift*1000:.0f} mm mantenendo la distanza dal pannello.',flush=True)
        state=n.get_state();q=q_from_state(state)
        tip=np.asarray(n.fingertip_fk(state),float)
        lifted=tip+up*args.camera_lift
        ref_center=lifted-args.final_standoff*normal0
        code,_=plan_and_maybe_execute(n,tuple(lifted),args,state,q,tuple(tip),
                 (tuple(ref_center),tuple(normal0)),args.final_standoff,
                 is_second=True,phase_label=f'FASE 3B - MICRO-SOLLEVAMENTO {args.camera_lift*1000:.0f} mm')
        if code!=0:
            raise RuntimeError('Non posso sollevare la camera senza un piano valido.')
        time.sleep(.5)
        require_rover_stationary(n,rover_initial)
        measure,lower=depth.observe(seconds=9.0)
    state=n.get_state()
    tip_current=np.array(n.fingertip_fk(state),float)
    qnow=q_from_state(state)
    time.sleep(.35)
    if max(abs(a-b) for a,b in zip(qnow,q_from_state(n.get_state())))>.010:
        raise RuntimeError('Braccio non fermo dopo seconda misurazione: STOP.')
    require_rover_stationary(n,rover_initial)
    # Piano frontale del pannello ~3 mm dietro la faccia anteriore misurata della leva.
    face=measure-normal0*LEVER_HALF_THICKNESS_M
    face_check=(face,normal0)
    shift=np.linalg.norm(measure-np.asarray(observed_initial,float))
    print('NUOVA POSIZIONE LEVA (ristimata, centro):',fmt(measure),flush=True)
    print('Bordo inferiore leva (ristimato):',fmt(lower),flush=True)
    print(f'Scarto rispetto alla stima iniziale: {shift*1000:.1f} mm',flush=True)
    if measure[2]-lower[2] < 0.005:
        raise RuntimeError('La ROI rossa non mostra una estremita inferiore affidabile: STOP.')
    if args.contact_mode == 'observe':
        print('SUCCESS RIOSSERVAZIONE: acquisizione RGB-D nuova e stabile. '+
              'Modalita observe: nessuna traiettoria di contatto inviata.',flush=True)
        return 0
    if n.main_switch_angle is None or n.main_switch_angle > -1.35:
        raise RuntimeError('La leva non e piu OFF dopo ri-osservazione: STOP prima del contatto.')

    def small_move(goal,label,max_delta=0.18):
        nonlocal state,tip_current,qnow
        target=tuple(float(v) for v in goal)
        if np.linalg.norm(np.asarray(target)-tip_current)>.075:
            raise RuntimeError(label+': tratta >7.5 cm: STOP.')
        # Il riferimento sintetico serve SOLO ai controlli interni della traiettoria;
        # il target nasce dalla NUOVA misura depth.
        virtual=(tuple(np.asarray(goal)-normal0*.012),tuple(normal0))
        status,_=plan_and_maybe_execute(n,target,args,state,qnow,tuple(tip_current),
                         virtual,.012,is_second=True,contact_phase=True,
                         phase_label=label,panel_face=face_check)
        if status!=0:
            raise RuntimeError(label+': MoveIt/controller ha rifiutato la manovra.')
        state=n.get_state();qnow=q_from_state(state)
        tip_current=np.asarray(n.fingertip_fk(state),float)
        require_rover_stationary(n,rover_initial)
        for _ in range(8):rclpy.spin_once(n,timeout_sec=.08)
        angle=n.main_switch_angle
        if angle is not None:
            delta=max(angle,n.main_switch_peak or angle)-before
            print(f'main_switch: {before:+.4f} -> {angle:+.4f} rad, delta={delta:+.4f}',flush=True)
            if abs(delta)>max_delta:
                raise RuntimeError('Leva si e mossa troppo rapidamente: STOP.')
        return angle

    drop=args.under_drop
    cdepth=args.contact_depth
    standoff_now=float(np.dot(tip_current-lower,normal0))
    print(f'\nFASE 4: sotto la leva (discesa {drop*1000:.0f} mm sotto il bordo basso, '
          f'profondita contatto {cdepth*1000:+.1f} mm dalla faccia anteriore).',flush=True)
    small_move(lower-up*drop+normal0*standoff_now,'FASE 4A - DISCESA davanti alla leva')
    small_move(lower-up*drop+normal0*0.012,'FASE 4B - AVVICINAMENTO sotto la punta della leva')
    small_move(lower-up*drop+normal0*cdepth,'FASE 4C - INGRESSO sotto la leva')
    reached=False
    for k,rise in enumerate(LIFT_RISES_M,1):
        angle=small_move(lower+up*rise+normal0*cdepth,
                         f'FASE 5.{k} - SALITA (punta {rise*1000:+.0f} mm sul bordo basso)',
                         max_delta=3.5)
        if angle is not None and angle>LIFT_DONE_ANGLE_RAD:
            reached=True
            break
    time.sleep(1.0)
    for _ in range(10):rclpy.spin_once(n,timeout_sec=.1)
    final=n.main_switch_angle
    if reached or (final is not None and final>LIFT_DONE_ANGLE_RAD):
        print(f'SUCCESS: main_switch alzato in Gazebo (angolo finale {final:+.3f} rad).',flush=True)
        try:
            small_move(tip_current+normal0*0.05,'FASE 6 - RITIRATA DAL PANNELLO',max_delta=3.5)
        except RuntimeError as e:
            print('Ritirata non eseguita:',e,flush=True)
        return 0
    print('LEVA NON ALZATA: angolo finale '+(f'{final:+.3f} rad' if final is not None else 'n/d')+
          '. Prova a variare --contact-depth (es. -0.002) o --under-drop; '+
          'nessun comando e stato dato al pannello.',flush=True)
    return 3


def main():
    parser = argparse.ArgumentParser(description='Pre-approach autonomo: ArUco/depth LIVE -> IK punta -> OMPL -> Gazebo')
    parser.add_argument('--execute', action='store_true', help='Avvicinamento automatico SOLO nel Gazebo panel_demo')
    parser.add_argument('--wait-align', action='store_true', help='Attendi /panel/align_done con pose successive per la demo')
    parser.add_argument('--plan-only', action='store_true', help='Compatibilita launch: solo piano; nessun movimento')
    parser.add_argument('--final-approach', choices=('true', 'false'),
                        default=('true' if os.environ.get('ROVER_FINAL_APPROACH', '').strip().lower() in ('1','true','yes') else 'false'),
                        help='true: dopo gli 8 cm pianifica anche una seconda tratta (default 5 cm)')
    parser.add_argument('--final-standoff', type=float, default=FINAL_STANDOFF_DEFAULT_M,
                        help='Distanza secondo target, metri, ammessa 0.045..0.075')
    parser.add_argument('--attempt-contact', choices=('true','false'), default='false',
                        help='Dopo 8->5cm: reobserve RGB-D reale e prova contatto SOLO in Gazebo')
    parser.add_argument('--contact-mode', choices=('observe','touch'), default='observe',
                        help='observe=solo nuova depth; touch=esegui anche i piccoli movimenti di contatto')
    parser.add_argument('--camera-lift', type=float, default=0.006,
                        help='Unico micro-sollevamento di fallback se la nuova depth non vede la leva (0..0.008 m)')
    parser.add_argument('--contact-depth', type=float, default=-0.001,
                        help='Profondita della punta rispetto alla faccia anteriore della leva (m); '
                             '<0 = verso il pannello. Ammessi -0.004..+0.010')
    parser.add_argument('--under-drop', type=float, default=0.025,
                        help='Quanto la punta scende sotto il bordo basso della leva (m), 0.015..0.040')
    parser.add_argument('--keep-visible', choices=('true','false'), default='true',
                        help='true: sceglie IK con visuale prevista e verifica depth vera dopo 8 e 5 cm; se fallisce STOP')
    parser.add_argument('--gripper-open-pos', type=float, default=GRIPPER_OPEN_POS,
                        help='Apertura slider_joint (m) usata da --open-gripper true; '
                             'limiti URDF -0.01..0.06 (default 0.04)')
    parser.add_argument('--open-gripper', choices=('true','false'), default='false',
                        help='true: apre le dita (slider_joint) prima della ricerca vista '
                             'cosi non coprono la camera nella posa finale')
    parser.add_argument('--max-wrist-tries', type=int, default=4,
                        help='Numero massimo di configurazioni di arrivo da provare, con verifica RGB-D (1..8)')
    args = parser.parse_args()
    if not (1 <= args.max_wrist_tries <= 8):
        parser.error('--max-wrist-tries ammesso tra 1 e 8')
    if not (-0.004 <= args.contact_depth <= 0.010):
        parser.error('--contact-depth ammesso tra -0.004 e +0.010 m')
    if not (0.0 <= args.gripper_open_pos <= 0.06):
        parser.error('--gripper-open-pos ammesso tra 0.0 e 0.06 m')
    if not (0.015 <= args.under_drop <= 0.040):
        parser.error('--under-drop ammesso tra 0.015 e 0.040 m')
    if not (0.0 <= args.camera_lift <= .008):
        parser.error("--camera-lift ammesso solo tra 0 e 8 mm")
    if args.attempt_contact == "true" and args.final_approach != "true":
        parser.error("--attempt-contact true richiede --final-approach true")
    if args.execute and args.plan_only:
        parser.error('--execute e --plan-only sono incompatibili')
    if not (math.isfinite(args.final_standoff) and
            FINAL_APPROACH_MIN_M <= args.final_standoff <= STANDOFF_M):
        parser.error('--final-standoff deve essere compreso fra 0.045 e 0.075 m')
    rclpy.init()
    n = Test()
    n.wait_align = args.wait_align
    try:
        print('MODALITA:', 'GAZEBO --execute (con preflight)' if args.execute else 'SOLO PLAN: nessun movimento', flush=True)
        print('FINAL_APPROACH:', args.final_approach, 'STANDOFF FINALE:', args.final_standoff, 'm', flush=True)
        # Non verificare il controller prima che Gazebo abbia finito l'avvio.
        # execute_in_gazebo fara' il check subito prima dell'invio.
        for srv in (n.scene, n.fk, n.valid, n.plan):
            if not srv.wait_for_service(timeout_sec=120.0):
                raise RuntimeError('Servizio MoveIt assente: ' + srv.srv_name)

        target = n.get_target()
        original = n.get_state()
        q_start = q_from_state(original)
        print('Giunti PARTENZA:', fmt(q_start), flush=True)
        time.sleep(1.2)
        refreshed = n.get_state()
        q_new = q_from_state(refreshed)
        if max(abs(a-b) for a,b in zip(q_new, q_start)) > MAX_START_STILL_RAD:
            raise RuntimeError('Il braccio sta ancora muovendosi: STOP; attendi allineamento.')
        original, q_start = refreshed, q_new
        n.check_state(original, 'Stato iniziale')
        start_tip = n.fingertip_fk(original)
        print('FK punta PARTENZA:', fmt(start_tip), flush=True)
        if math.dist(start_tip,target) > MAX_FINGERTIP_TRAVEL_M:
            raise RuntimeError('Target troppo distante dalla punta: STOP (nessuna esecuzione).')

        # Block diagnosis when RViz TF and FK disagree substantially.
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and not n.tf_buffer.can_transform('base_link', 'ee_finger1', Time()):
            rclpy.spin_once(n, timeout_sec=0.1)
        if not n.tf_buffer.can_transform('base_link', 'ee_finger1', Time()):
            raise RuntimeError('TF base_link -> ee_finger1 non disponibile. '
                               'Prima verifica la discrepanza RViz/Gazebo.')
        tf = n.tf_buffer.lookup_transform('base_link', 'ee_finger1', Time()).transform
        offset = rot(tf.rotation, TCP_LOCAL)
        start_tip_tf = tuple(a+b for a,b in zip((tf.translation.x, tf.translation.y, tf.translation.z), offset))
        print('TF punta PARTENZA:', fmt(start_tip_tf), flush=True)
        if math.dist(start_tip, start_tip_tf) > MAX_TF_FK_ERROR_M:
            raise RuntimeError('Stato MoveIt e TF discordano: STOP prima della pianificazione.')

        # Apertura dita PRIMA della ricerca vista: senno' la camera (montata
        # sotto il polso) inquadra le dita invece della leva.
        if args.open_gripper == 'true':
            open_gripper(n, args.gripper_open_pos)
            # La punta e' definita come offset nel frame del dito: ruotando
            # il dito (apertura) la punta FK si sposta. Rileggo lo stato DOPO
            # l'apertura, altrimenti la ricerca vista e la pianificazione
            # userebbero due stati diversi (16 cm di errore punta).
            time.sleep(0.8)
            refreshed = n.get_state()
            q_new = q_from_state(refreshed)
            if max(abs(a-b) for a, b in zip(q_new, q_start)) > MAX_START_STILL_RAD:
                raise RuntimeError('Il braccio si e mosso durante l apertura dita: STOP.')
            original, q_start = refreshed, q_new
            n.check_state(original, 'Stato dopo apertura dita')
            start_tip = n.fingertip_fk(original)
            print('FK punta dopo apertura dita:', fmt(start_tip), flush=True)

        # Fotografia della posa statica ottenuta DOPO l'allineamento.
        center0 = n.center0.copy()
        normal0 = n.normal0.copy()
        frozen = (tuple(float(x) for x in center0), tuple(float(x) for x in normal0))
        rover_initial = rover_pose_in_odom(n) if args.final_approach == 'true' else None
        # Osservatore RGB-D indipendente (usato per i controlli di visibilita e la FASE 3).
        depth = FreshDepthObserver(n, center0)
        if args.keep_visible == 'true':
            code, last_q = search_visible_arrival(
                n, args, target, original, q_start, start_tip, frozen, depth)
        else:
            code, last_q = plan_and_maybe_execute(
                n, target, args, original, q_start, start_tip, frozen, STANDOFF_M)
        if code != 0 or args.final_approach != 'true':
            return code

        require_rover_stationary(n, rover_initial)
        angle_before = n.main_switch_angle
        # Conserviamo la fase 1 originale e passiamo alla fase 2 SOLO se ha
        # pianificato correttamente (e, con --execute, e' stata eseguita).
        if args.execute:
            second_state = n.get_state()
            second_q = q_from_state(second_state)
            if max(abs(x-y) for x,y in zip(second_q, last_q)) > 0.035:
                raise RuntimeError('Stato dopo pre-approach non coincide con il piano: STOP seconda fase.')
            time.sleep(1.0)
            verify_q = q_from_state(n.get_state())
            if max(abs(x-y) for x,y in zip(second_q, verify_q)) > MAX_START_STILL_RAD:
                raise RuntimeError('Braccio ancora in movimento dopo la fase 1: STOP.')
            second_state = n.get_state()
            second_q = q_from_state(second_state)
        else:
            # In solo-plan si simula la configurazione d'arrivo della fase 1.
            # Non affermiamo di averla eseguita.
            second_state = put_joints(original, last_q)
            second_q = tuple(last_q)
            print('SOLO PLAN: fase 2 pianificata da stato previsto, non raggiunto fisicamente.', flush=True)

        require_rover_stationary(n, rover_initial)
        second_tip = n.fingertip_fk(second_state)
        expected_first_target = tuple(center0 + STANDOFF_M * normal0)
        if math.dist(second_tip, expected_first_target) > 0.025:
            raise RuntimeError('Punta dopo fase 1 troppo lontana dal pre-approach: STOP seconda fase.')
        n.check_state(second_state, 'Stato iniziale seconda fase')
        final_target = tuple(float(v) for v in (center0 + args.final_standoff * normal0))
        print(f'AVVICINAMENTO FINALE: {STANDOFF_M*100:.1f} -> '
              f'{args.final_standoff*100:.1f} cm davanti alla leva (NO contatto).', flush=True)
        print('TARGET FASE 2:', fmt(final_target), flush=True)
        if math.dist(second_tip, final_target) > 0.08:
            raise RuntimeError('Secondo passo piu lungo di 8 cm: STOP.')
        code2, _ = plan_and_maybe_execute(n, final_target, args, second_state,
                                          second_q, second_tip, frozen,
                                          args.final_standoff, is_second=True)
        if code2 == 0:
            print('FASE 2 PIANIFICATA' if not args.execute else 'FASE 2 ESEGUITA', flush=True)
            print('Nessun comando diretto a main_switch; contatto NON programmato.', flush=True)
            if args.execute:
                for _ in range(10):
                    rclpy.spin_once(n, timeout_sec=0.1)
                if n.main_switch_angle is not None and angle_before is not None:
                    print(f'Angolo main_switch prima/dopo: {angle_before:+.4f} -> '
                          f'{n.main_switch_angle:+.4f} rad; '
                          f'delta={n.main_switch_angle-angle_before:+.4f} rad', flush=True)
        if code2 == 0 and args.execute:
            ensure_lever_visible(n, args, depth, frozen,
                                 f'dopo avvicinamento finale a {args.final_standoff*100:.1f} cm')
        if code2 == 0 and args.attempt_contact == "true":
            return perform_contact_extension(n,args,frozen,rover_initial,center0,depth)
        return code2
    except Exception as e:
        print('STOP:', e, flush=True)
        print('Se un movimento era gia in corso, verificarne lo stato in Gazebo.', flush=True)
        return 2
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('Interrotto. Se era gia partito un comando, verificare lo stato Gazebo.', flush=True)
        sys.exit(130)
