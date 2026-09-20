#!/usr/bin/env python3
"""
EtherCAt keyboard + PS4 Controller control
Can run on Linux, macOs and windows

Keyboard fallback:  w/s/a/d/z, space, e, q
Joystick (DS4) mapping:
    D-pad ↑↓←→       → forward / reverse / left / right
    L2/R2 = decel/accel.
    × = setup
    ○ = spin
    □ = idle 
    △ = quit
"""
import os
import re
import sys
import tty
import termios
import subprocess as sp
import time
import pygame
import ctypes

# Velocity presets -------------------------------------NB check again the values!!!!!!!!!!!!!!!!!
VEL_LOW  = 0x186A0          # 100,000   (right?)    0xFFFE7960  (left?)
VEL_MID  = 0x7A120          # 500.000   (left?)     0xFFF85EE0  (right?)
VEL_HIGH = 0X000C3500       # 800.000   0XFFF3CB00
VEL_HIGHER = 0x000F4240     # 1.000.000 0xFF0BDC0
VEL_SO_HIGHER = 0x0016E340  # 1.500.000 0xFFE91CC0 
velocity_order = [VEL_LOW, VEL_MID, VEL_HIGH, VEL_HIGHER, VEL_SO_HIGHER]

# Driver groups (edit if wiring differs) 
row1 = [3, 4, 5]
row2 = [0, 1, 2]
motors=(0, 1, 2, 3, 4, 5)

flag_action = "forward"
vel_index = 0

# Keyboard mapping 
KEYMAP = {
    "w": "forward",
    "s": "reverse",
    "a": "left",
    "d": "right",
    "z": "increase_speed",
    "x": "decrease_speed",
    "f": "spin",
    " ": "idle",
    "e": "setup",
    "q": "quit",
}


vel_state = {
    "forward": 0,
    "reverse": 0,
    "left": 0,
    "right": 0,
    "increase_speed": 0,
    "decrease_speed": 0,
}


def get_key():    # for macOs
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)               # raw 1-char input
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# Increase/decrease velocity
#def update_velocity_state(action):
  #   idx = vel_state[action]
   #  if idx > 0:                         # HIGH → MID → LOW
    #    vel_state[action] = idx - 1
    #return velocity_order[vel_state[action]]

def change_speed(action):
    global vel_index
    #idx = vel_state[action]

    if action == "increase_speed" and vel_index < len(velocity_order) - 1:
        vel_index += 1
	#vel_state["action"] = idx + 1
    elif action == "decrease_speed" and vel_index > 0:
        vel_index -= 1
	#vel_state[action] = idx - 1

    vel = velocity_order[vel_index]
    #vel = velocity_order[vel_state["forward"]]
    print(f" Speed level: {vel_index}  (value={vel:#x})")
    return vel

# Ethercat command for setting velocity
def ether_download(port, value):
    sp.run(
        ["sudo", "ethercat", "download",
         f"-p{port}","--type", "uint32", "0x60FF", "--", "0x00", str(value)],
        check=True,
    )
    # May need this version if getting type not specified
   # sp.run(
    #    ["sudo", "ethercat", "download",
     #    f"-p{port}", "--type", "int32", "0x60FF", "0x00", str(value)],
     #   check=True,
    #)

# Actual command
def send_velocity(v_row1, v_row2):         #----------------check again, how can this work????????
#   for p in row1:
#       ether_download(p, v_row1)
#       ether_download(p, v_row2)
    for p in row1:
    	ether_download(p, v_row1)
    for p in row2:
    	ether_download(p, v_row2)


# List of possible action --------------------------------------------------
def handle_action(action):
    # Quit
    if action == "quit":        
        print("Exiting.")
        sys.exit(0)
    
    # Setup Motors
    if action == "setup":

      print("Setting Profile Velocity Mode (0x03) on all motors")

      # Imposta profile velocity mode
      for i in motors:
            sp.run(
                ["sudo", "ethercat", "download", f"-p{i}", "--type", "int8", "0x6060", "0x00", "0x03"],
                check=True,
            )
            time.sleep(0.2)
      # Shutdown
      for i in motors:
            sp.run(
                ["sudo", "ethercat", "download", f"-p{i}", "--type", "uint16", "0x6040", "0x00", "0x06"],
                check=True,
            )
            time.sleep(0.2)
      # Switch ON
      for i in motors:
            sp.run(
                ["sudo", "ethercat", "download", f"-p{i}", "--type", "uint16", "0x6040", "0x00", "0x07"],
                check=True,
            )
            time.sleep(0.2)      
      # Enable Operation
      for i in motors:
            sp.run(
                ["sudo", "ethercat", "download", f"-p{i}", "--type", "uint16", "0x6040", "0x00", "0x0F"],
                check=True,
            )
            time.sleep(0.2)

    # Setup Motors
    if action == "idle":   

      print("Stopping all motors")
      for i in motors:
            sp.run(
                ["sudo", "ethercat", "download", f"-p{i}", "--type", "int32", "0x60FF", "0x00", "100"],
                check=True,
            )
      for i in motors:
            sp.run(
                ["sudo", "ethercat", "download", f"-p{i}", "--type", "int8", "0x6060", "0x00", "0x00"],
                check=True,
            )
                    
    if action not in vel_state:
        return

    vel = velocity_order[vel_index]
    #vel2 = ctypes.c_int32("100000").value
    #vel2 = "-100000"
    #vel = change_speed(action)
    print(f" {action} at {vel:#x}")

    # Commands
    if action == "forward":
        send_velocity(-vel, vel)
        flag_action = "forward"
    
    elif action == "reverse":
        send_velocity(vel, -vel)
        flag_action = "reverse"      
    
    elif action == "right":
        if flag_action == "forward":
            if vel_index < 4:           # right - forward
                vel2 = velocity_order[vel_index+1]
            else:
                vel = velocity_order[vel_index-1]
                vel2 = velocity_order[vel_index]
            send_velocity(-vel, vel2)
        else:                           # right - backwards
            if vel_index < 4:
                vel2 = velocity_order[vel_index+1]
            else:
                vel = velocity_order[vel_index-1]
                vel2 = velocity_order[vel_index]
            send_velocity(vel, -vel2)      
    
    elif action == "left":
        if flag_action == "reverse":
            if vel_index < 4:           # left - forward
                vel2 = velocity_order[vel_index+1]
            else:
                vel = velocity_order[vel_index-1]
                vel2 = velocity_order[vel_index]
            send_velocity(-vel2, vel)
        else:
            if vel_index < 4:
                vel2 = velocity_order[vel_index+1]
            else:
                vel = velocity_order[vel_index-1]
                vel2 = velocity_order[vel_index]
            send_velocity(vel2, -vel)
    
    elif action == "spin":
        send_velocity(vel, vel)     # non funziona?

# Check controller connection
def dualshock4_connected():
    if sys.platform == "darwin":  # macOS
        try:
            out = sp.check_output(
                ["ioreg", "-n", "Sony Interactive Entertainment Wireless Controller", "-r"]
            )
            return b"Wireless Controller" in out
        except Exception:
            return False
    elif sys.platform.startswith("linux"):  # Linux
        return os.path.exists("/dev/input/js0")
    elif sys.platform == "win32":  # Windows
        return True
    return False

# Map buttons to actions
BUTTON_ACTION_MAP = {       # working at least on Windows
    # D-pad directions
    11: "forward",     # D-Pad Up
    12: "reverse",     # D-Pad Down
    13: "left",        # D-Pad Left
    14: "right",       # D-Pad Right

    0: "setup",        # ✕
    1: "spin",         # cerchio
    2: "quit",         # triangolo
    3: "idle",         # quadrato
}

face_buttons_pressed = set()

# Controller Initialization
def init_ds4():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        return None
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"🎮  {js.get_name()} connected")
    print("📋  Button mapping:")
    print("    D-pad ↑↓←→ = forward/reverse/left/right")
    print("    × = setup  ○ = spin  □ = stop  △ = quit  L2 = increase speed   R2 = decrease speed")
    return js

# Translate joystick button into events
def ds4_event_to_action(ev):
    global face_buttons_pressed, dpad_buttons_pressed
    global L2_ACTIVE, R2_ACTIVE

    # Face buttons 
    if ev.type == pygame.JOYBUTTONDOWN:
        #print(f"[DEBUG] Button {ev.button} pressed")
        if ev.button in BUTTON_ACTION_MAP:
            print(f"🎮  {BUTTON_ACTION_MAP[ev.button]} pressed")
            face_buttons_pressed.add(ev.button)
            return BUTTON_ACTION_MAP.get(ev.button)

    elif ev.type == pygame.JOYBUTTONUP:
        #print(f"[DEBUG] Button {ev.button} released")
        if ev.button in BUTTON_ACTION_MAP:
            print(f"🎮  {BUTTON_ACTION_MAP[ev.button]} released")
            face_buttons_pressed.discard(ev.button)

    # D-pad
    elif ev.type == pygame.JOYHATMOTION:
        x, y = ev.value  # tuple (-1,0,1)
        print(f"[DEBUG] D-pad moved: x={x}, y={y}")
        if y == 1:
            return "forward"
        elif y == -1:
            return "reverse"
        elif x == -1:
            return "left"
        elif x == 1:
            return "right"

    # L2/R2
    elif ev.type == pygame.JOYAXISMOTION:
        val = ev.value
        if ev.axis == 2:    #L2
           if val == 1: 
	   #if val > 0.5:
                print(f"🎮  L2 pressed (val={val:.2f}) → decrease_speed")
                change_speed("decrease_speed")
                send_velocity(vel_index, vel_index)
           #elif val < 0.1:
            #   print(f"🎮  L2 released (val={val:.2f})")
        elif ev.axis == 5:  #R2
            if val == 1:
                print(f"🎮  R2 pressed (val={val:.2f}) → increase_speed")
                change_speed("increase_speed")
                send_velocity(vel_index, vel_index)
            #elif val < 0.1:
             #   print(f"🎮  R2 released (val={val:.2f})")

    return None


# Main Loop
js = init_ds4() if dualshock4_connected() else None

if js:
    # Joystick connected
    print("🎮  Joystick control active")
    try:
        while True:
            input_detected = False
            for event in pygame.event.get():
                action = ds4_event_to_action(event)
                if action:
                    handle_action(action)
                    input_detected = True
    except KeyboardInterrupt:
        print("\n Exiting DS4 control.")

else:
    # Keyboard fallback
    print("⌨️  Keyboard control active")
    print("Keys: w s a d z  | space=idle  e=setup  q=quit   z=accel   x=decel")
    while True:
        key = get_key()
        if key in KEYMAP:
            handle_action(KEYMAP[key])
