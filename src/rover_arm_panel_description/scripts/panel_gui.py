#!/usr/bin/env python3

import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from builtin_interfaces.msg import Duration as DurationMsg
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rover_arm_panel_description.msg import PanelState
from rover_arm_panel_description.srv import SetSwitch

# ---------------------------------------------------------------------------
# Panel geometry data (user coordinate system: X=right, Y=up from bottom-left)
# ---------------------------------------------------------------------------

SWITCHES = {
    'main_switch': {
        'type': 'lever', 'label': 'MAIN', 'x': 85, 'y': 220,
        'size': (30, 12), 'joint': 'main_switch', 'limits': (-1.57, 1.57),
    },
    'lever_switch_1': {
        'type': 'lever', 'label': 'L1', 'x': 117, 'y': 220,
        'size': (25, 10), 'joint': 'lever_switch_1', 'limits': (-1.57, 1.57),
    },
    'lever_switch_2': {
        'type': 'lever', 'label': 'L2', 'x': 149, 'y': 220,
        'size': (25, 10), 'joint': 'lever_switch_2', 'limits': (-1.57, 1.57),
    },
    'lever_switch_3': {
        'type': 'lever', 'label': 'L3', 'x': 181, 'y': 220,
        'size': (25, 10), 'joint': 'lever_switch_3', 'limits': (-1.57, 1.57),
    },
    'lever_switch_4': {
        'type': 'lever', 'label': 'L4', 'x': 213, 'y': 220,
        'size': (25, 10), 'joint': 'lever_switch_4', 'limits': (-1.57, 1.57),
    },
    'rotary_switch_1': {
        'type': 'rotary', 'label': 'R1', 'x': 55, 'y': 300,
        'radius': 12, 'joint': 'rotary_switch_1',
    },
    'rotary_switch_2': {
        'type': 'rotary', 'label': 'R2', 'x': 86, 'y': 300,
        'radius': 12, 'joint': 'rotary_switch_2',
    },
    'rotary_switch_3': {
        'type': 'rotary', 'label': 'R3', 'x': 117, 'y': 300,
        'radius': 12, 'joint': 'rotary_switch_3',
    },
    'rotary_switch_4': {
        'type': 'rotary', 'label': 'R4', 'x': 148, 'y': 300,
        'radius': 12, 'joint': 'rotary_switch_4',
    },
    'rotary_switch_5': {
        'type': 'rotary', 'label': 'R5', 'x': 179, 'y': 300,
        'radius': 12, 'joint': 'rotary_switch_5',
    },
    'power_switch_1': {
        'type': 'rotary', 'label': 'PWR1', 'x': 210, 'y': 300,
        'radius': 15, 'joint': 'power_switch_1',
    },
    'power_switch_2': {
        'type': 'rotary', 'label': 'PWR2', 'x': 241, 'y': 300,
        'radius': 15, 'joint': 'power_switch_2',
    },
}

INDICATORS = [
    {'id': 'ind_1', 'label': 'PWR', 'x': 105, 'y': 390},
    {'id': 'ind_2', 'label': 'CABLE', 'x': 129, 'y': 390},
    {'id': 'ind_3', 'label': 'MAG', 'x': 153, 'y': 390},
    {'id': 'ind_4', 'label': 'SYS', 'x': 177, 'y': 390},
    {'id': 'ind_5', 'label': 'ERR', 'x': 201, 'y': 390},
]

SOCKETS = [
    {'id': 'socket_grey', 'label': 'SOCK A', 'x': 130, 'y': 135},
    {'id': 'socket_pink_1', 'label': 'CONN B', 'x': 200, 'y': 135},
    {'id': 'socket_pink_2', 'label': 'CONN C', 'x': 265, 'y': 135},
]

ELECTROMAGNET = {'x': 240, 'y': 50, 'radius': 35}

SCALE = 1.3
PANEL_W = 330
PANEL_H = 450
CANVAS_W = int(PANEL_W * SCALE) + 200
CANVAS_H = int(PANEL_H * SCALE) + 100
OFFSET_X = 60
OFFSET_Y = 60

TASK_STEPS = [
    '1. Set MAIN switch ON',
    '2. Set 4 lever switches ON',
    '3. Turn 3 rotary switches',
    '4. Insert plug into socket',
    '5. Turn Power Switch 1 ON',
    '6. Turn Power Switch 2 ON',
    '7. Place plate on magnet',
    '8. All 5 indicators lit',
]


class PanelGuiNode(Node):
    def __init__(self):
        super().__init__('panel_gui')

        # ── Publishers ─────────────────────────────
        self._traj_pub = self.create_publisher(
            JointTrajectory, '/panel_controller/joint_trajectory', 10)
        self._state_pub = self.create_publisher(
            PanelState, '/panel/state', 10)

        # ── Subscribers ────────────────────────────
        self._js_sub = self.create_subscription(
            JointState, '/panel/joint_states', self._on_joint_states, 10)

        # ── Services ────────────────────────────────
        self._srv = self.create_service(
            SetSwitch, '/panel/set_switch', self._on_set_switch)

        # ── State ───────────────────────────────────
        self._joint_positions = {name: 0.0 for name in SWITCHES}
        self._indicator_states = [False] * 5
        self._socket_states = [False] * 3
        self._electromagnet_on = False
        self._plate_attached = False
        self._elements_broken = {name: False for name in SWITCHES}
        self._damage_info = {name: '' for name in SWITCHES}
        self._plug_side = False  # which side has the plug: False=left, True=right

        # ── GUI will be set later ──────────────────
        self._gui = None

        # ── Timer for state publishing ─────────────
        self._state_timer = self.create_timer(0.1, self._publish_state)

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            if name in self._joint_positions:
                self._joint_positions[name] = msg.position[i] if i < len(msg.position) else 0.0

    def _on_set_switch(self, req, resp):
        if req.element_id in SWITCHES:
            self.send_joint_command(req.element_id, req.position)
            resp.success = True
            resp.message = f'{req.element_id} set to {req.position:.2f}'
        elif req.element_id.startswith('ind_'):
            idx = int(req.element_id.split('_')[1]) - 1
            if 0 <= idx < 5:
                self._indicator_states[idx] = req.state
                resp.success = True
                resp.message = f'{req.element_id} set to {req.state}'
        elif req.element_id.startswith('socket'):
            idx = {'socket_grey': 0, 'socket_pink_1': 1, 'socket_pink_2': 2}.get(req.element_id, -1)
            if idx >= 0:
                self._socket_states[idx] = req.state
                resp.success = True
                resp.message = f'{req.element_id} set to {req.state}'
        elif req.element_id == 'electromagnet':
            self._electromagnet_on = req.state
            if not req.state:
                self._plate_attached = False
            resp.success = True
            resp.message = f'Electromagnet set to {req.state}'
        elif req.element_id == 'metal_plate':
            self._plate_attached = req.state
            resp.success = True
            resp.message = f'Plate attached set to {req.state}'
        else:
            resp.success = False
            resp.message = f'Unknown element: {req.element_id}'
        return resp

    def set_gui(self, gui):
        self._gui = gui

    def send_joint_command(self, joint_name, position):
        if self._elements_broken.get(joint_name, False):
            self.get_logger().warn(f'{joint_name} is BROKEN — ignoring command')
            return

        msg = JointTrajectory()
        msg.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = DurationMsg(sec=0, nanosec=100000000)
        msg.points = [point]
        self._traj_pub.publish(msg)
        self.get_logger().info(f'Commanded {joint_name} → {position:.2f}')

    def toggle_switch(self, switch_id):
        sw = SWITCHES[switch_id]
        if self._elements_broken.get(switch_id, False):
            return
        current = self._joint_positions.get(switch_id, 0.0)
        if sw['type'] == 'lever':
            lo, hi = sw['limits']
            mid = (lo + hi) / 2.0
            target = lo if current > mid else hi
            self.send_joint_command(switch_id, target)
        elif sw['type'] == 'rotary':
            target = current + 0.5
            self.send_joint_command(switch_id, target)

    def toggle_rotary_back(self, switch_id):
        sw = SWITCHES[switch_id]
        if self._elements_broken.get(switch_id, False):
            return
        current = self._joint_positions.get(switch_id, 0.0)
        target = current - 0.5
        self.send_joint_command(switch_id, target)

    def toggle_socket(self, idx):
        self._socket_states[idx] = not self._socket_states[idx]

    def toggle_electromagnet(self):
        self._electromagnet_on = not self._electromagnet_on
        if not self._electromagnet_on:
            self._plate_attached = False

    def toggle_plate(self):
        if self._electromagnet_on:
            self._plate_attached = not self._plate_attached

    def _publish_state(self):
        msg = PanelState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.main_switch = self._joint_positions.get('main_switch', 0.0)
        msg.lever_switches = [
            self._joint_positions.get(f'lever_switch_{i}', 0.0) for i in range(1, 5)]
        msg.rotary_switches = [
            self._joint_positions.get(f'rotary_switch_{i}', 0.0) for i in range(1, 6)]
        msg.power_switches = [
            self._joint_positions.get('power_switch_1', 0.0),
            self._joint_positions.get('power_switch_2', 0.0),
        ]
        msg.indicators = self._indicator_states
        msg.sockets = self._socket_states
        msg.electromagnet_on = self._electromagnet_on
        msg.plate_attached = self._plate_attached

        levers = msg.lever_switches
        rotaries = msg.rotary_switches
        powers = msg.power_switches
        msg.task_main_switch_on = msg.main_switch > 1.0
        msg.task_lever_switches_set = all(v > 0.5 for v in levers)
        msg.task_rotary_switches_set = sum(1 for v in rotaries if abs(v) > 0.5) >= 3
        msg.task_plug_inserted = any(msg.sockets)
        msg.task_power_switch_1_on = powers[0] > 1.0 if len(powers) > 0 else False
        msg.task_power_switch_2_on = powers[1] > 1.0 if len(powers) > 1 else False
        msg.task_plate_placed = self._plate_attached
        msg.task_all_indicators_lit = all(self._indicator_states)

        msg.elements_broken = [self._elements_broken[k] for k in SWITCHES]
        msg.damage_info = [self._damage_info[k] for k in SWITCHES]

        self._state_pub.publish(msg)

        if self._gui:
            self._gui.update_task_checklist(msg)
            self._gui.update_damage_display(self._elements_broken)


# ---------------------------------------------------------------------------
# Tkinter GUI
# ---------------------------------------------------------------------------

class PanelGUI(tk.Tk):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.title('ERC 2026 — Maintenance Task Panel')
        self.geometry(f'{CANVAS_W + 280}x{CANVAS_H + 60}')

        self._main_frame = ttk.Frame(self)
        self._main_frame.pack(fill=tk.BOTH, expand=True)

        # ── Canvas ─────────────────────────────────
        self._canvas_frame = ttk.LabelFrame(self._main_frame, text='Panel')
        self._canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self._canvas = tk.Canvas(
            self._canvas_frame, width=CANVAS_W, height=CANVAS_H,
            bg='#2a2a2a', highlightthickness=0)
        self._canvas.pack(padx=5, pady=5)

        # ── Side panel ─────────────────────────────
        self._side_frame = ttk.Frame(self._main_frame, width=250)
        self._side_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        self._side_frame.pack_propagate(False)

        # Task checklist
        self._task_frame = ttk.LabelFrame(self._side_frame, text='Task Sequence')
        self._task_frame.pack(fill=tk.X, pady=(0, 10))

        self._task_vars = []
        for step in TASK_STEPS:
            var = tk.BooleanVar(value=False)
            cb = ttk.Checkbutton(self._task_frame, text=step, variable=var, state='disabled')
            cb.pack(anchor=tk.W, padx=5, pady=1)
            self._task_vars.append(var)

        # Damage display
        self._damage_frame = ttk.LabelFrame(self._side_frame, text='Damage Status')
        self._damage_frame.pack(fill=tk.BOTH, expand=True)

        self._damage_text = tk.Text(self._damage_frame, height=8, state='disabled')
        self._damage_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Status bar
        self._status_label = ttk.Label(self._side_frame, text='Ready', relief=tk.SUNKEN)
        self._status_label.pack(fill=tk.X, side=tk.BOTTOM, pady=(10, 0))

        # ── Draw elements ──────────────────────────
        self._canvas_items = {}
        self._draw_panel()

        # ── Bindings ───────────────────────────────
        self._canvas.bind('<Button-1>', self._on_click)
        self._canvas.bind('<Button-3>', self._on_right_click)

    # ── Drawing ────────────────────────────────────

    def _to_canvas(self, x, y):
        cx = OFFSET_X + x * SCALE
        cy = OFFSET_Y + (PANEL_H - y) * SCALE
        return cx, cy

    def _draw_panel(self):
        self._canvas.delete('all')

        # Panel face
        x1, y1 = self._to_canvas(0, 0)
        x2, y2 = self._to_canvas(PANEL_W, PANEL_H)
        self._canvas.create_rectangle(
            x1, y1, x2, y2, fill='#d0d0d0', outline='#888888', width=2)

        # Indicators
        for ind in INDICATORS:
            cx, cy = self._to_canvas(ind['x'], ind['y'])
            r = 6 * SCALE
            item = self._canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r,
                fill='#3a5a3a', outline='#888', width=1)
            self._canvas.create_text(cx, cy + r + 8, text=ind['label'],
                                     fill='white', font=('Arial', 7))
            self._canvas_items[ind['id']] = item

        # DIN rail background
        x1, y1 = self._to_canvas(55 - 110, 300 - 20)
        x2, y2 = self._to_canvas(55 + 110 + 220, 300 + 20)
        self._canvas.create_rectangle(
            x1, y1, x2, y2, fill='#555', outline='#777', width=1)

        # Switches
        for sid, sw in SWITCHES.items():
            cx, cy = self._to_canvas(sw['x'], sw['y'])
            if sw['type'] == 'lever':
                w, h = sw.get('size', [0.03, 0.02])[0] * SCALE, sw.get('size', [0.03, 0.02])[1] * SCALE
                item = self._canvas.create_rectangle(
                    cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2,
                    fill='#cc4444' if sid == 'main_switch' else '#cccc44',
                    outline='#aaa', width=1,
                    tags=('switch', sid))
                # Knob indicator line
                knob = self._canvas.create_line(
                    cx, cy - h / 2, cx, cy + h / 2,
                    fill='black', width=2, tags=('knob', sid))
                self._canvas_items[f'{sid}_knob'] = knob
            elif sw['type'] == 'rotary':
                r = sw.get('radius', 0.020) * SCALE
                item = self._canvas.create_oval(
                    cx - r, cy - r, cx + r, cy + r,
                    fill='#444', outline='#888', width=1,
                    tags=('switch', sid))
                # Indicator pointer
                pointer = self._canvas.create_line(
                    cx, cy, cx + r, cy,
                    fill='#ff4444', width=2, tags=('pointer', sid))
                self._canvas_items[f'{sid}_pointer'] = pointer

            self._canvas_items[sid] = item
            # Label below
            self._canvas.create_text(
                cx, cy + sw.get('size', [0.03, 0.02])[1] * SCALE + 10,
                text=sw['label'], fill='white', font=('Arial', 7))

        # Sockets
        for sock in SOCKETS:
            cx, cy = self._to_canvas(sock['x'], sock['y'])
            w, h = 25 * SCALE, 15 * SCALE
            item = self._canvas.create_rectangle(
                cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2,
                fill='#886644', outline='#aaa', width=1,
                tags=('socket', sock['id']))
            self._canvas_items[sock['id']] = item
            self._canvas.create_text(
                cx, cy + h / 2 + 10,
                text=sock['label'], fill='white', font=('Arial', 7))

        # Electromagnet
        cx, cy = self._to_canvas(ELECTROMAGNET['x'], ELECTROMAGNET['y'])
        r = ELECTROMAGNET['radius'] * SCALE
        item = self._canvas.create_oval(
            cx - r, cy - r, cx + r, cy + r,
            fill='#666', outline='#888', width=2,
            tags=('electromagnet',))
        self._canvas_items['electromagnet'] = item
        self._canvas.create_text(
            cx, cy + r + 10, text='MAGNET', fill='white', font=('Arial', 7))

        # Metal plate
        px, py = self._to_canvas(ELECTROMAGNET['x'], ELECTROMAGNET['y'] + 25)
        pw, ph = 70 * SCALE, 8 * SCALE
        item = self._canvas.create_rectangle(
            px - pw / 2, py - ph / 2, px + pw / 2, py + ph / 2,
            fill='#999999', outline='#ccc', width=1,
            tags=('plate',))
        self._canvas_items['metal_plate'] = item
        self._canvas.create_text(
            px, py + ph / 2 + 10, text='PLATE', fill='white', font=('Arial', 7))

        # Side handle
        hx1, hy1 = self._to_canvas(-15, 225 - 75)
        hx2, hy2 = self._to_canvas(-5, 225 + 75)
        self._canvas.create_rectangle(
            hx1, hy1, hx2, hy2, fill='#777', outline='#999', width=1)

        # Labels for panel sections
        for label, x, y in [('INDICATORS', 165, 420), ('LEVERS', 165, 250),
                            ('ROTARIES', 165, 330), ('SOCKETS', 165, 165)]:
            tx, ty = self._to_canvas(x, y)
            self._canvas.create_text(
                tx, ty, text=label, fill='#aaa', font=('Arial', 8, 'italic'))

    # ── Interaction ────────────────────────────────

    def _find_nearest(self, x, y, tags, max_dist=40):
        items = self._canvas.find_overlapping(x - max_dist, y - max_dist,
                                               x + max_dist, y + max_dist)
        best_item = None
        best_dist = max_dist
        for item in items:
            tags_list = self._canvas.gettags(item)
            if any(t.startswith(tag) for tag in tags):
                coords = self._canvas.coords(item)
                if coords:
                    cx = sum(coords[::2]) / len(coords[::2])
                    cy = sum(coords[1::2]) / len(coords[1::2])
                    dist = math.hypot(x - cx, y - cy)
                    if dist < best_dist:
                        best_dist = dist
                        best_item = item
        return best_item

    def _get_switch_id_from_item(self, item):
        tags = self._canvas.gettags(item)
        for tag in tags:
            if tag in SWITCHES:
                return tag
        return None

    def _on_click(self, event):
        x, y = event.x, event.y

        # Check switch clicks
        item = self._find_nearest(x, y, ('switch',), max_dist=30)
        if item:
            sid = self._get_switch_id_from_item(item)
            if sid:
                self.node.toggle_switch(sid)
                self._status_label.config(text=f'Actuated: {SWITCHES[sid]["label"]}')
                return

        # Check socket clicks
        item = self._find_nearest(x, y, ('socket',), max_dist=30)
        if item:
            tags = self._canvas.gettags(item)
            for tag in tags:
                if tag.startswith('socket_'):
                    idx = {'socket_grey': 0, 'socket_pink_1': 1, 'socket_pink_2': 2}.get(tag, 0)
                    self.node.toggle_socket(idx)
                    self._status_label.config(text=f'Toggled: {tag}')
                    self.update_socket_display(tag, self.node._socket_states[idx])
                    return

        # Check electromagnet
        item = self._find_nearest(x, y, ('electromagnet',), max_dist=30)
        if item:
            self.node.toggle_electromagnet()
            self._status_label.config(
                text=f'Electromagnet: {"ON" if self.node._electromagnet_on else "OFF"}')
            return

        # Check metal plate
        item = self._find_nearest(x, y, ('plate',), max_dist=30)
        if item:
            self.node.toggle_plate()
            self._status_label.config(
                text=f'Plate: {"ATTACHED" if self.node._plate_attached else "DETACHED"}')
            return

    def _on_right_click(self, event):
        x, y = event.x, event.y
        item = self._find_nearest(x, y, ('pointer', 'knob'), max_dist=30)
        if item:
            tags = self._canvas.gettags(item)
            for tag in tags:
                if tag in SWITCHES and SWITCHES[tag]['type'] == 'rotary':
                    self.node.toggle_rotary_back(tag)
                    self._status_label.config(text=f'Rotated back: {SWITCHES[tag]["label"]}')
                    return

    # ── Visual Updates ─────────────────────────────

    def update_indicator_display(self):
        colors = ['#33ff33' if s else '#2a4a2a' for s in self.node._indicator_states]
        for i, ind in enumerate(INDICATORS):
            item = self._canvas_items.get(ind['id'])
            if item:
                self._canvas.itemconfig(item, fill=colors[i])

    def update_socket_display(self, socket_id, plugged):
        item = self._canvas_items.get(socket_id)
        if item:
            color = '#44aa44' if plugged else '#886644'
            self._canvas.itemconfig(item, fill=color)

    def update_switch_positions(self):
        for sid, sw in SWITCHES.items():
            pos = self.node._joint_positions.get(sid, 0.0)
            cx, cy = self._to_canvas(sw['x'], sw['y'])

            if sw['type'] == 'lever':
                knob = self._canvas_items.get(f'{sid}_knob')
                if knob:
                    h = sw.get('size', [0.03, 0.02])[1] * SCALE
                    angle = pos  # rad
                    dx = math.sin(angle) * h * 1.5
                    dy = -math.cos(angle) * h * 1.5
                    self._canvas.coords(knob, cx, cy, cx + dx, cy + dy)

            elif sw['type'] == 'rotary':
                pointer = self._canvas_items.get(f'{sid}_pointer')
                if pointer:
                    r = sw.get('radius', 0.020) * SCALE * 0.8
                    ex = cx + r * math.cos(pos)
                    ey = cy - r * math.sin(pos)
                    self._canvas.coords(pointer, cx, cy, ex, ey)

    def update_electromagnet_display(self):
        item = self._canvas_items.get('electromagnet')
        if item:
            color = '#33ff33' if self.node._electromagnet_on else '#666666'
            self._canvas.itemconfig(item, fill=color, outline='#33ff33' if self.node._electromagnet_on else '#888')

        plate_item = self._canvas_items.get('metal_plate')
        if plate_item:
            if self.node._plate_attached:
                # Move plate up to electromagnet
                cx, cy = self._to_canvas(ELECTROMAGNET['x'], ELECTROMAGNET['y'] - 5)
                pw, ph = 70 * SCALE, 8 * SCALE
                self._canvas.coords(plate_item,
                                     cx - pw / 2, cy - ph / 2,
                                     cx + pw / 2, cy + ph / 2)
                self._canvas.itemconfig(plate_item, fill='#cccccc')
            else:
                cx, cy = self._to_canvas(ELECTROMAGNET['x'], ELECTROMAGNET['y'] + 25)
                pw, ph = 70 * SCALE, 8 * SCALE
                self._canvas.coords(plate_item,
                                     cx - pw / 2, cy - ph / 2,
                                     cx + pw / 2, cy + ph / 2)
                self._canvas.itemconfig(plate_item, fill='#999999')

    def update_task_checklist(self, msg):
        checks = [
            msg.task_main_switch_on,
            msg.task_lever_switches_set,
            msg.task_rotary_switches_set,
            msg.task_plug_inserted,
            msg.task_power_switch_1_on,
            msg.task_power_switch_2_on,
            msg.task_plate_placed,
            msg.task_all_indicators_lit,
        ]
        for i, var in enumerate(self._task_vars):
            if i < len(checks):
                var.set(checks[i])

    def update_damage_display(self, broken):
        self._damage_text.config(state='normal')
        self._damage_text.delete(1.0, tk.END)
        has_damage = False
        for sid, is_broken in broken.items():
            if is_broken:
                label = SWITCHES[sid]['label']
                self._damage_text.insert(tk.END, f'⚠  {label}: DAMAGED\n')
                has_damage = True
        if not has_damage:
            self._damage_text.insert(tk.END, 'All elements OK\n')
        self._damage_text.config(state='disabled')

    def update_all(self):
        self.update_indicator_display()
        self.update_switch_positions()
        self.update_electromagnet_display()

        for sock in SOCKETS:
            idx = {'socket_grey': 0, 'socket_pink_1': 1, 'socket_pink_2': 2}.get(sock['id'], 0)
            self.update_socket_display(sock['id'], self.node._socket_states[idx])

        self.after(100, self.update_all)


def main(args=None):
    rclpy.init(args=args)
    node = PanelGuiNode()

    app = PanelGUI(node)
    node.set_gui(app)

    # Periodic ROS2 spin in background
    def poll_ros():
        rclpy.spin_once(node, timeout_sec=0)
        app.after(20, poll_ros)
    app.after(20, poll_ros)

    app.update_all()
    app.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
