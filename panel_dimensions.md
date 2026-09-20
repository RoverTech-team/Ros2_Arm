# ERC 2026 Maintenance Task Panel - Geometry Data for URDF

This document contains the estimated dimensions and coordinates for the ERC 2026 Maintenance Task panel to assist in generating a URDF (Unified Robot Description Format) file for simulation.

## 1. Overall Enclosure Dimensions
*Note: The official workspace panel face is defined as 330 mm x 450 mm. The rest of the enclosure volume is estimated.*

* **Workspace Panel Face (W x H):** 330 mm x 450 mm
* **Panel Incline Angle:** ~45° to 50° relative to the ground/base.
* **Total Enclosure Width (X):** ~360 mm (includes side walls)
* **Total Enclosure Depth (Y, base):** ~300 - 350 mm
* **Total Enclosure Height (Z, rear peak):** ~350 mm

## 2. Coordinate System Definition
To map elements onto the sloped face in a URDF `joint` or `link`:
* **Origin (0,0,0):** Bottom-left corner of the white 330x450mm sloped workspace panel.
* **X-axis:** Points to the right along the panel face (0 to 330 mm).
* **Y-axis:** Points upward along the sloped panel face (0 to 450 mm).
* **Z-axis:** Normal to the sloped panel face (pointing outwards toward the rover).

## 3. Element Coordinates & Bounding Boxes
*Note: X and Y coordinates are to the **center** of the component's bounding box. Z-depth is an estimate of how far the component protrudes outward from the panel face for collision modeling.*

| Component | X-Center (mm) | Y-Center (mm) | Est. Size W x H (mm) | Est. Z-Protrusion (mm) | Shape Type |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Top Left Black Square** | 50 | 390 | 45 x 45 | 0 (flush/inset) | Box |
| **Top Green Buttons (Group of 5)** | 165 | 390 | 120 x 20 | +5 | Box |
| **Top Right Black Square** | 280 | 390 | 45 x 45 | 0 (flush/inset) | Box |
| **DIN Rail with Switches** | 165 | 300 | 220 x 40 | +25 | Box |
| **Middle Grey Toggle Switches (5x)**| 165 | 220 | 160 x 25 | +20 | Box/Cylinders |
| **Lower Left Grey Socket** | 130 | 135 | 30 x 20 | +10 | Box |
| **Lower Pink Connector 1 (Left)** | 200 | 135 | 50 (diameter) | +35 | Cylinder |
| **Lower Pink Connector 2 (Right)** | 265 | 135 | 50 (diameter) | +35 | Cylinder |
| **Bottom Left Black Cutout** | 60 | 50 | 45 x 45 | 0 (flush/inset) | Box (Negative space) |
| **Bottom Right Metal Handle/Bar** | 240 | 50 | 150 x 20 | +30 | Box/Cylinder |

## 4. External Elements
| Component | X-Center (mm) | Y-Center (mm) | Est. Size W x H (mm) | Est. Z-Protrusion (mm) | Note |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Side Metal Handle (Left Wall)** | -15 | 225 | 20 x 150 | +30 (from side wall) | Mounted on the exterior left face |
