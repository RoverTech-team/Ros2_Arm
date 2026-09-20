#!/usr/bin/env python3
"""Generate a blank 10x10m occupancy map (all free) for Nav2 simulation."""
import os
import struct

MAP_DIR = os.path.join(
    os.path.dirname(__file__), "..", "config"
)
SIZE_M = 10.0
RESOLUTION = 0.05
PIXELS = int(SIZE_M / RESOLUTION)  # 200

pgm_path = os.path.join(MAP_DIR, "panel_workspace.pgm")
yaml_path = os.path.join(MAP_DIR, "panel_workspace.yaml")

# PGM: P5 binary grayscale, all 254 (free)
with open(pgm_path, "wb") as f:
    f.write(f"P5\n{PIXELS} {PIXELS}\n255\n".encode())
    # 254 = free (255 is unknown in some conventions, 0 = occupied)
    row = bytes([254] * PIXELS)
    for _ in range(PIXELS):
        f.write(row)

with open(yaml_path, "w") as f:
    f.write(f"""image: panel_workspace.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")

print(f"Generated {pgm_path}")
print(f"Generated {yaml_path}")
print(f"Map: {SIZE_M}x{SIZE_M}m, {PIXELS}x{PIXELS}px, {RESOLUTION}m/px, all free")
