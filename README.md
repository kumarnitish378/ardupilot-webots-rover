# Webots UGV Rover

A [Webots](https://cyberbotics.com/) simulation of an autonomous four-wheel
UGV rover, built as a stepping stone toward an ArduPilot Rover SITL bridge.
The rover uses **front-wheel steering + rear-wheel drive** (with an Ackermann
kinematic differential), independent suspension on every wheel, and a full
sensor suite. It can be driven manually from the keyboard, run a reactive
autonomous obstacle-avoidance loop, or take commands from an external file.

Two worlds are included: a small test **arena** with rocks/trees/obstacles,
and a **real-world map** generated from OpenStreetMap.

> Developed and tested with **Webots R2025a** on Windows.

---

## Features

- **Full-size rover** — 1.83 × 0.80 × 0.61 m body (6 × 4 × 2 ft), 18 in (0.457 m)
  wheels, ~170 kg, 0.36 m ground clearance, softer long-travel suspension.
- **Three drive modes** — Manual (keyboard), Autonomous (reactive obstacle
  avoidance), Remote (command file).
- **Sensor suite** — front FPV camera, top-down "map" camera, GPS, IMU, gyro,
  accelerometer, two front distance sensors, four corner radars, and steering
  position sensors.
- **Steering PID auto-tune** — relay-feedback (Åström–Hägglund) tuning on a
  key press.
- **Runtime speed control**, full state logging to CSV, and colour-coded
  components for easy visual identification (green = front, red = rear).
- **Real-world map** — a 500 × 500 m OpenStreetMap area imported as a drivable
  world, with solid road curbs and a physical geofence boundary.

---

## Repository layout

```
worlds/
  ugv_rover.wbt            # test arena: rocks, trees, plants, dunes
  osm_rover.wbt            # real OSM map (Greater Noida) + geofence
  my_first_simulation.wbt  # original e-puck example world
controllers/
  ugv_teleop/             # main rover controller (C)
    ugv_teleop.c
  epuck_go_forward/       # original example controller
LICENSE
README.md
```

The rover model is defined inline in the two `*_rover.wbt` worlds (identical
Robot node in both).

---

## Getting started

### 1. Prerequisites
- Webots **R2025a** (other recent versions may work but are untested).

### 2. Clone
```bash
git clone https://github.com/kumarnitish378/ardupilot-webots-rover.git
```

### 3. Open a world
In Webots: **File → Open World…** and choose one of:
- `worlds/osm_rover.wbt` — the real-world map (recommended).
- `worlds/ugv_rover.wbt` — the obstacle test arena.

### 4. Build the controller (first run only)
Webots compiles the controller automatically the first time you press Play.
If it doesn't, open `controllers/ugv_teleop/ugv_teleop.c` in Webots' editor
and click **Build**. (A C compiler is required — Webots bundles one on
Windows.)

### 5. Run
Press **▶ Play**, then **click inside the 3D view** so it has keyboard focus.

### 6. Show the camera overlays
Use the **Overlays** menu in Webots' menu bar to show the `camera` (front FPV)
and `map_camera` (top-down) feeds if they aren't already visible.

---

## Controls

| Key | Action |
|-----|--------|
| **↑ / ↓** | Drive forward / reverse |
| **← / →** | Steer left / right |
| **M** | Manual mode |
| **A** | Autonomous mode (reactive obstacle avoidance) |
| **R** | Remote mode (reads `rover_command.txt`) |
| **T** | Auto-tune the steering PID |
| **PageUp / + / =** | Increase speed (up to 2.5×) |
| **PageDown / -** | Decrease speed (down to 0.2×) |

### Drive modes
- **Manual** — you drive with the arrow keys.
- **Autonomous** — the rover drives itself, steering away from obstacles seen
  by the front distance sensors and reversing out if it gets boxed in or stuck
  (stuck detection uses GPS ground speed, so it works in both the local-metre
  arena and the WGS84 map world).
- **Remote** — the controller reads `drive steer` values (each in `[-1, 1]`)
  from `controllers/ugv_teleop/rover_command.txt`; the last value written is
  held until replaced. Useful for scripted control.

### Telemetry / logging
Every step is logged to `controllers/ugv_teleop/ugv_teleop_log.csv`
(position, heading, all sensor readings, steering angles, wheel speeds) and a
summary line is printed to the console once per second.

---

## Rover at a glance

| | |
|---|---|
| Body | 1.83 × 0.80 × 0.61 m (6 × 4 × 2 ft) |
| Wheels | 0.457 m dia (18 in), front steer / rear drive |
| Mass | ~170 kg |
| Wheelbase / track | 1.20 m / 1.10 m |
| Ground clearance | 0.36 m |
| Suspension | independent per wheel (spring + damper, ±0.12 m travel) |

**Colour code:** green = front (bumper + wheel stripes), red = rear; cyan =
front sensor pod, blue = camera, yellow = distance sensors, magenta = radars,
orange = nav-sensor cluster.

---

## Regenerating the OSM map (optional / advanced)

`osm_rover.wbt` was produced from OpenStreetMap data with Webots' bundled
`osm_importer` (`<WEBOTS_HOME>/resources/osm_importer/importer.py`). To build a
different area:

1. Download an `.osm` extract for your bounding box, e.g.:
   ```
   curl "https://api.openstreetmap.org/api/0.6/map?bbox=<min_lon>,<min_lat>,<max_lon>,<max_lat>" -o map.osm
   ```
2. Install the importer's Python dependencies:
   `pip install pyproj shapely numpy webcolors lxml`
3. Run the importer, then paste the rover Robot node into the generated world.

---

## Known limitations

- The four corner **radars currently detect nothing** in these worlds: Webots'
  Radar only sees `Solid` nodes with `radarCrossSection > 0`, and the rocks,
  walls, buildings and curbs don't set it. Obstacle avoidance therefore relies
  on the two front **distance sensors**. Converting the radars to distance-
  sensor cones (or setting a cross-section on obstacles) is a planned fix.
- Physics values (masses, spring/damper rates, motor torques, steering PID)
  are reasonable engineering estimates, not lab-calibrated; expect to tune for
  new scenarios.

---

## Roadmap

- Replace the manual/reactive controller with an **ArduPilot Rover SITL**
  bridge (socket-based, à la `ardupilot/libraries/SITL/examples/Webots_Python`).
- All-around obstacle sensing (fix the radar cross-section issue).
- Software geofence / mission handling via ArduPilot.

---

## License

Released under the [MIT License](LICENSE).
