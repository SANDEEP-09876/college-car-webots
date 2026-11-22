# Webots Simulation — Autonomous 4WD Car (College Demo)

A complete, interactive Webots project of the Phase-1 autonomous car.
The robot model matches the final CAD: 260×180 mm plates, 65 mm wheels,
135 mm wheelbase, 200 mm track, 3× ToF (front / front-left / front-right),
Pi Camera V2 on a low front mount, MPU6050 IMU, wheel encoders, and the
6-DOF arm (photo-style, blue/white) on the turret.

100% free — Webots is open-source (Apache 2.0). No premium software needed.

## Install & run

1. Install Webots from **cyberbotics.com** (Windows / Linux / macOS), R2023b or newer.
2. In Webots: `File → Open World`, pick `worlds/college_demo.wbt` from this folder.
3. Press the **Play** button (or `Ctrl+Shift+P`).
4. **Click the 3D view once** so it has keyboard focus, then use the keys below.

## Keyboard controls

| Key | Action |
|---|---|
| `T` | Teleop mode — drive with WASD or arrow keys |
| `1` | Autonomous obstacle avoidance |
| `2` | Right-hand wall following |
| `3` | Waypoint navigation (4 markers) |
| `Space` | Stop |
| `R` | Reset robot to start + clear metrics |
| `P` | Print scoreboard (time, distance, collisions, odometry drift) |
| `H` | Print help |
| `4` | Arm: stowed pose (folded back over the rear deck) |
| `5` | Arm: demo pose (raised) |
| `6` | Arm: reach pose (forward, fingers open) |
| `7` | Arm: inspect pose (gripper high) |
| `8` | Arm: pick pose (gripper straight down, fingertips ~4 mm up) |
| `9` | Arm auto-demo: cycle the five poses every 6 s (press again to stop) |

## The 6-DOF arm (photo-style)

The arm from the photo — white base, blue turret and shoulder, white
forearm, white wrist, white palm, blue 2-segment fingers, **no ball** —
mounted on the pre-drilled turret circle at (45, 0). Link lengths
120 / 110 / 70 / 85 mm, max reach 385 mm. The camera mast is gone: it
stood in the arm's sweep, so the Pi Camera moved to the low front mount
(lens z ~150 mm). The arm loads in the **stowed** pose.

Joints are relative angles (the gripper pitch always equals the wrist
pitch — there is no extra pitch joint at the gripper):

| Joint | Motion | Range |
|---|---|---|
| J0 | base yaw | −170°…+170° |
| J1 | shoulder pitch | −20°…+115° |
| J2 | elbow (relative to upper arm) | −155°…+25° |
| J3 | wrist pitch (relative to forearm) | −120°…+120° |
| J4 | wrist roll (about the tool axis) | −180°…+180° |
| J5 | gripper fingers (0° open … 15° closed) | 0°…15° |

Practical limits found by the CAD joint sweep (all five poses are verified
0.00 mm³ interference): shoulder ≥ +14° when the arm points straight
forward (front ToF bracket, top 173 mm); low picks need the base yawed
≥ ~28° off-centre (same bracket); elbow −125°…+25° at the demo reference
(−150° only with the shoulder raised, as in the stowed pose); wrist
−80°…+120° at the demo reference. The full CAD package (per-link STLs in
joint frames for sim import, poses, sweep data) is `full_robot_photo_arm.zip`.

If the cloud session's keyboard does not reach the controller, set
`ARM_AUTO_DEMO = True` at the top of `college_car.py` and the arm cycles
the poses on its own.

The 3D camera follows the robot automatically. Open the robot's camera view via
the device toolbar to see what the Pi Camera sees.

## The three demo scenarios

**Zone A — Slalom (press `1`).** Five orange boxes in a staggered lane.
The car drives forward on the front ToF; under 40 cm it stops and turns toward
the clearer side (comparing left/right ToF), with gentle lane-centering
otherwise. Score: zero collisions.

**Zone B — Corridor (press `2`).** A 1.4 m wide, 6 m long corridor.
Right-hand wall following with a PD loop holding 35 cm from the right wall.
Drive the car to the corridor entrance with `T` first, then press `2`.

**Zone C — Waypoints (press `3`).** Four colored ground markers.
Proportional heading control drives to each in turn; the front ToF overrides
steering if something gets in the way. Reaching all four prints the scoreboard.

**Scoreboard (`P`)** reports run time, encoder distance, collision count,
waypoints reached, and **encoder-odometry drift vs ground truth** — a genuine
talking point: this drift is exactly why the real robot fuses the MPU6050.

## How it maps to the real hardware

| Simulation | Real robot |
|---|---|
| `drive(v, omega)` velocity commands | ESP32 wheel-speed PID setpoints |
| `tof_front/left/right` (metres) | VL53L0X readings |
| `enc_*` PositionSensors | N25 motor encoders |
| `imu` InertialUnit | MPU6050 |
| `camera` 640×480, 62.2° HFOV | Pi Camera Module V2 |
| `gps` | ground truth for the scoreboard drift number — but waypoint mode (key `3`) still steers from GPS for now; a GPS-free odometry + IMU version is drafted, pending the Test 0 yaw-sign check |

## Files

- `worlds/college_demo.wbt` — the arena (slalom + corridor + waypoints)
- `protos/CollegeCar.proto` — the robot (dimensions from the final CAD)
- `controllers/college_car/college_car.py` — all behaviours + scoring

## Suggested college demo flow (5 minutes)

1. `T` — drive manually, show the camera overlay and live sensor response.
2. `R`, then `1` — obstacle avoidance through the slalom, zero collisions.
3. `R`, drive to corridor with `T`, then `2` — wall following.
4. `R`, then `3` — waypoint run; end with `P` scoreboard.
5. Talking point: odometry drift number, and what sensor fusion would fix.

## ESP32 realism layer (in `college_car.py`, `ESP32_SIM = True`)

Webots alone is too perfect: motors reach speed instantly, encoders return
exact angles, ToF returns exact distances. The controller now mimics the
real ESP32 firmware between the behaviour modes and the devices:

- **Per-wheel velocity PID** — feedforward + PI (D = 0 on purpose, the
  derivative of a quantised encoder at 32 ms is noise), output as PWM duty
  in [-1, 1], wheel speed following through a 0.10 s first-order motor lag.
  Gains: Kp = 0.10, Ki = 0.20, Kd = 0. Tuned offline for the 32 ms Webots
  step; the real ESP32 loop is ~10 ms, so re-tune on hardware.
- **Encoder steps** — odometry and PID see only whole encoder counts
  (858 counts per wheel revolution, step 0.42°). **ASSUMED, not in the
  datasheet:** 11 counts/motor-rev × 78:1, single-edge counting. Measure
  it by rotating one wheel exactly one revolution and counting pulses,
  then replace `ENCODER_COUNTS_PER_WHEEL_REV` in the controller.
- **Sensor noise** — Gaussian ToF noise (8 mm + 1% of distance, 1%
  no-return dropouts) and Gaussian IMU yaw noise (0.01 rad), fixed seed
  42 so runs are repeatable. Set `ESP32_SIM = False` for the original
  ideal behaviour. Original controller kept as
  `college_car_pre_esp32.py.bak` in the controller folder.

## Notes

- Physics: 1.85 kg chassis (1.7 kg car + arm base + PDB), 70 rpm wheel limit;
  arm links ~0.85 kg total (turret 0.12, shoulder 0.25, forearm 0.20,
  wrist 0.12, gripper 0.10, fingers 0.03 each). Whole robot ~2.7 kg.
- The car-only PROTO is kept as `protos/CollegeCar_noarm.proto` (no arm,
  camera on the mast) — the controller runs on it too, with the arm keys
  doing nothing.
- ToF model: single-ray, 2 m range, linear response — an approximation of the
  VL53L0X; real-world crosstalk and surface dependence are not modelled.
- If a sensor seems to face the wrong way in your Webots version, check the
  `rotation` fields in `CollegeCar.proto` (Webots R2022+: sensors face +X).
