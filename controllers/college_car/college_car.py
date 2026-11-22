# college_car.py
# Controller for the CollegeCar PROTO (4WD autonomous car).
#
# Keyboard controls (click the 3D view first so it has focus):
#   T ......... teleop mode (WASD / arrow keys to drive)
#   1 ......... autonomous obstacle avoidance (slalom zone)
#   2 ......... right-hand wall following (corridor zone)
#   3 ......... waypoint navigation (waypoint markers)
#   Space ..... stop
#   R ......... reset robot to start + clear metrics
#   P ......... print scoreboard
#   H ......... print this help
#   4..8 ...... arm pose: 4 stowed, 5 demo, 6 reach, 7 inspect, 8 pick
#   9 ......... arm auto-demo: cycle the five poses every 6 s (press again
#               to stop it; Space stops the car, 9 stops the arm cycle)
#
# 6-DOF ARM (photo-style, blue/white; CollegeCar.proto, J0..J4 + 2 fingers):
#   Joints are RELATIVE angles (deg), link lengths 120/110/70/85 mm.
#   J0 yaw -170..+170, J1 shoulder -20..+115, J2 elbow -155..+25 (relative),
#   J3 wrist -120..+120 (relative), J4 roll -180..+180, fingers 0..15 deg
#   (0 = open, 15 = closed). Gripper pitch = wrist pitch (rigid roll
#   collar); the Webots J4 angle is -q4 and the finger motors are L = -phi,
#   R = +phi (see set_arm_pose). The arm loads in the STOWED pose.
#   ARM_AUTO_DEMO = True makes the arm cycle poses on its own (for cloud
#   sessions where key presses may not reach the controller).
#
# The controller mirrors the real hardware split:
#   velocity commands  -> what the ESP32 wheel-PID loop will receive
#   tof_front/left/right -> VL53L0X readings (metres)
#   enc_*             -> wheel encoder ticks (radians here)
#   imu               -> MPU6050
#   gps               -> ground truth, used ONLY for scoring/odometry-drift
#                       measurement, never for navigation.
#
# ESP32 REALISM LAYER (ESP32_SIM = True, below):
#   Webots on its own is too perfect: motors reach the commanded speed
#   instantly, PositionSensors return exact angles, and DistanceSensors
#   return exact distances. The real ESP32 does none of that. This layer
#   sits between the high-level modes and the Webots devices and mimics
#   what the ESP32 firmware would do:
#     1. Per-wheel velocity PID (feedforward + PI, D = 0 on purpose:
#        differentiating a quantised encoder at 32 ms is too noisy).
#        PID output is a PWM duty in [-1, 1]; wheel speed follows it
#        through a first-order motor lag (MOTOR_TAU).
#     2. Encoder quantisation: odometry and PID see only whole encoder
#        counts, never the exact Webots angle.
#     3. Sensor noise: Gaussian noise on every ToF reading, Gaussian
#        noise on the IMU yaw, and a small ToF dropout rate.
#   The PID runs once per Webots step (32 ms). The real ESP32 loop is
#   ~10 ms, so the sim PID is slower than the hardware loop; gains below
#   were tuned offline for the 32 ms step (see the tuning notes in the
#   project files) and will need re-tuning on hardware.
#   Set ESP32_SIM = False to restore the original ideal behaviour.
#
#   ASSUMPTION — ENCODER COUNTS: ENCODER_COUNTS_PER_WHEEL_REV = 858 is
#   ASSUMED (11 counts per motor revolution x 78:1 gearbox, single-edge
#   counting). It is NOT in the GM25-370CA datasheet and no encoder CPR
#   has been confirmed. Measure it: rotate one wheel exactly one
#   revolution by hand, count the encoder pulses, and replace this
#   constant with the measured counts per wheel revolution. Until then,
#   treat every odometry/drift number from this sim as approximate.

from controller import Supervisor, Keyboard
import math
import random

TIME_STEP = 32
WHEEL_R = 0.0325     # wheel radius, m (65 mm wheels)
TRACK = 0.20         # left-right wheel distance, m
MAX_W = 7.33         # rad/s (70 rpm no-load)
START_POS = [-3.5, 0.0, -2.5]
WAYPOINTS = [(4.5, 3.6), (2.5, 3.6), (2.5, 2.6), (4.5, 2.6)]
WP_TOL = 0.30        # waypoint reached radius, m

# --- ESP32 realism layer: constants (tune / measure as noted above) ---
ESP32_SIM = True
ENCODER_COUNTS_PER_WHEEL_REV = 858  # ASSUMED, see header. Measure it.
ENCODER_STEP_RAD = 2.0 * math.pi / ENCODER_COUNTS_PER_WHEEL_REV
PID_KP = 0.10        # per rad/s of speed error
PID_KI = 0.20        # per rad/s, integrated over seconds
PID_KD = 0.0         # deliberately 0: derivative of quantised encoder is noise
PID_INT_MAX = 1.5    # integral anti-windup clamp (PWM units)
MOTOR_TAU = 0.10     # s, first-order motor/driver lag
VEL_FILTER_ALPHA = 0.35  # low-pass on encoder-derived wheel speed
TOF_NOISE_BASE_M = 0.008   # m, Gaussian sigma floor
TOF_NOISE_PCT = 0.01       # sigma grows 1% of distance
TOF_DROPOUT_P = 0.01       # per reading: report max range (no return)
IMU_YAW_NOISE_RAD = 0.01   # rad, Gaussian sigma on IMU yaw
NOISE_SEED = 42            # fixed seed: same noise every run, for repeatable demos

# --- 6-DOF arm: pose targets (joint-space degrees, from the photo-arm CAD) ---
# q = (q0 yaw, q1 shoulder, q2 elbow-rel, q3 wrist-rel, q4 roll), phi = finger
# angle deg (0 open, 15 closed). All five poses are verified 0.00 mm^3
# interference against the car CAD; the sweep limits are in the PROTO.
ARM_AUTO_DEMO = False
ARM_POSES_DEG = {
    "stowed":  ((170, 85, -150, 65, 0), 15),
    "demo":    ((-20, 65, -85, 50, 0), 7),
    "reach":   ((0, 15, 0, 0, 0), 0),
    "inspect": ((0, 70, -50, 30, 0), 7),
    "pick":    ((35, 5, -21, -74, 0), 0),
}
ARM_LIMITS_DEG = {"J0": (-170, 170), "J1": (-20, 115), "J2": (-155, 25),
                  "J3": (-120, 120), "J4": (-180, 180), "J5": (0, 15)}
ARM_VEL = 1.0           # rad/s, arm joint speed
ARM_AUTO_PERIOD_S = 6.0  # s per pose in the auto-demo cycle


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def tof_to_m(v):
    # lookupTable [ 0 0 0,  0.02 1000 0,  2.0 0 0 ]
    if v >= 1000.0:
        return 0.02
    if v <= 0.0:
        return 2.0
    return 2.0 - (v / 1000.0) * 1.98


class _EncoderSim:
    """Quantise an exact Webots angle to whole encoder counts.

    The real ESP32 only ever sees integer counts. Counts increase by
    one whole step at a time; between steps the reported angle does not
    move at all. `base` is the true angle at the last reset.
    """

    def __init__(self):
        self.base = 0.0
        self.counts = 0

    def reset(self, true_angle=0.0):
        self.base = true_angle
        self.counts = 0

    def update(self, true_angle):
        self.counts = math.floor((true_angle - self.base) / ENCODER_STEP_RAD)
        return self.base + self.counts * ENCODER_STEP_RAD


class _WheelPID:
    """One wheel-speed PID, as the ESP32 firmware would run it.

    Output is a PWM duty in [-1, 1]: feedforward (setpoint / MAX_W)
    plus PI correction on the speed error. Integral is clamped so it
    cannot wind up while the output is saturated.
    """

    def __init__(self):
        self.integral = 0.0
        self.prev_err = None

    def reset(self):
        self.integral = 0.0
        self.prev_err = None

    def update(self, setpoint, measured, dt):
        err = setpoint - measured
        self.integral += err * dt
        self.integral = max(-PID_INT_MAX, min(PID_INT_MAX, self.integral))
        deriv = 0.0
        if self.prev_err is not None:
            deriv = (err - self.prev_err) / dt
        self.prev_err = err
        pwm = setpoint / MAX_W + PID_KP * err + PID_KI * self.integral \
            + PID_KD * deriv
        return max(-1.0, min(1.0, pwm))


class CollegeCar:
    def __init__(self):
        self.robot = Supervisor()   # Supervisor IS a Robot, plus getSelf() for the R reset
        self.kb = self.robot.getKeyboard()
        self.kb.enable(TIME_STEP)

        self.motors = {}
        for n in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
            m = self.robot.getDevice(n)
            m.setPosition(float("inf"))   # velocity-control mode
            m.setVelocity(0.0)
            self.motors[n] = m

        self.enc = {}
        for n in ("enc_fl", "enc_fr", "enc_rl", "enc_rr"):
            e = self.robot.getDevice(n)
            e.enable(TIME_STEP)
            self.enc[n] = e

        self.tof = {}
        for n in ("tof_front", "tof_left", "tof_right"):
            s = self.robot.getDevice(n)
            s.enable(TIME_STEP)
            self.tof[n] = s

        self.imu = self.robot.getDevice("imu")
        self.imu.enable(TIME_STEP)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(TIME_STEP)
        self.cam = self.robot.getDevice("camera")
        self.cam.enable(TIME_STEP)

        # --- 6-DOF arm (defensive: the car-only PROTO has no arm devices) ---
        self.arm_motors = {}
        self.arm_pos = {}
        for n in ("arm_j0", "arm_j1", "arm_j2", "arm_j3", "arm_j4",
                  "finger_L", "finger_R"):
            m = self.robot.getDevice(n)
            if m is not None:
                self.arm_motors[n] = m
        for n in ("arm_pos_j0", "arm_pos_j1", "arm_pos_j2", "arm_pos_j3",
                  "arm_pos_j4", "arm_pos_finger_L", "arm_pos_finger_R"):
            s = self.robot.getDevice(n)
            if s is not None:
                s.enable(TIME_STEP)
                self.arm_pos[n] = s
        self.arm_present = len(self.arm_motors) == 7
        self.arm_pose_name = "stowed"
        self.arm_auto = ARM_AUTO_DEMO
        self.arm_auto_next = 0.0
        self.arm_auto_idx = 0
        if self.arm_present:
            for name, m in self.arm_motors.items():
                m.setVelocity(2.0 if name.startswith("finger") else ARM_VEL)
            self.set_arm_pose("stowed")
            if self.arm_auto:
                print("[ARM] auto-demo ON: cycling poses every %.0f s" % ARM_AUTO_PERIOD_S)
        else:
            print("[ARM] no arm devices in this PROTO (car-only build)")

        # --- ESP32 layer state ---
        self.rng = random.Random(NOISE_SEED)
        self.encoders = {n: _EncoderSim() for n in self.enc}
        self.pids = {n: _WheelPID() for n in self.motors}
        self.vel_actual = {n: 0.0 for n in self.motors}  # simulated motor speed
        self.meas_vel = {n: 0.0 for n in self.motors}    # filtered encoder speed
        self.q_enc = None            # quantised encoder angles, this step
        self._prev_q_enc = None      # quantised encoder angles, previous step
        self.cmd_wl = 0.0            # left wheel speed setpoint, rad/s
        self.cmd_wr = 0.0            # right wheel speed setpoint, rad/s
        self.imu_yaw_noisy = 0.0

        self.mode = "STOP"
        self.keys = {}            # key code -> last-seen time (for held keys)
        self.prev_enc = None
        self.odom = [START_POS[0], START_POS[2], 0.0]  # x, z, theta (encoder odometry)
        self.prev_gps = None
        self.nav_xy = (START_POS[0], START_POS[2])
        self.heading = 0.0        # GPS-derived heading, rad
        self.wp_idx = 0
        self.collisions = 0
        self.in_collision = False
        self.dist_m = 0.0
        self.t0 = 0.0
        self.wall_err_prev = 0.0
        self.print_help()

    # ---------------- ESP32 layer ----------------
    def _read_encoders(self):
        """Read all encoders once per step, quantised if ESP32_SIM is on."""
        true = {n: self.enc[n].getValue() for n in self.enc}
        if ESP32_SIM:
            self.q_enc = {n: self.encoders[n].update(true[n]) for n in self.enc}
        else:
            self.q_enc = true
        return self.q_enc

    def esp32_step(self):
        """One ESP32 loop: PID + motor lag + noisy IMU yaw.

        Must run once per main-loop iteration, after drive() has set
        the speed setpoints. With ESP32_SIM off it does nothing:
        drive() already set the motor velocities directly.
        """
        if not ESP32_SIM:
            self.imu_yaw_noisy = self.imu.getRollPitchYaw()[2]
            return
        if self.q_enc is None:
            self._read_encoders()
        dt = TIME_STEP / 1000.0
        setpoints = {
            "wheel_fl": self.cmd_wl, "wheel_rl": self.cmd_wl,
            "wheel_fr": self.cmd_wr, "wheel_rr": self.cmd_wr,
        }
        enc_for_motor = {
            "wheel_fl": "enc_fl", "wheel_rl": "enc_rl",
            "wheel_fr": "enc_fr", "wheel_rr": "enc_rr",
        }
        for motor_name, pid in self.pids.items():
            enc_name = enc_for_motor[motor_name]
            if self._prev_q_enc is None:
                raw_vel = 0.0
            else:
                raw_vel = (self.q_enc[enc_name] - self._prev_q_enc[enc_name]) / dt
            self.meas_vel[motor_name] += VEL_FILTER_ALPHA * (
                raw_vel - self.meas_vel[motor_name])
            pwm = pid.update(setpoints[motor_name], self.meas_vel[motor_name], dt)
            # First-order motor/driver lag: speed approaches pwm * MAX_W.
            target = pwm * MAX_W
            self.vel_actual[motor_name] += (target - self.vel_actual[motor_name]) \
                * (dt / (MOTOR_TAU + dt))
            self.motors[motor_name].setVelocity(self.vel_actual[motor_name])
        self._prev_q_enc = dict(self.q_enc)
        raw_yaw = self.imu.getRollPitchYaw()[2]
        self.imu_yaw_noisy = raw_yaw + self.rng.gauss(0.0, IMU_YAW_NOISE_RAD)

    # ---------------- low-level ----------------
    def drive(self, v, omega):
        """Differential-drive inverse kinematics -> wheel speed setpoints.

        With ESP32_SIM on these are setpoints for the PID layer, applied
        by esp32_step(). With it off they are set on the motors directly,
        as in the original controller.
        """
        wl = (v - omega * TRACK / 2.0) / WHEEL_R
        wr = (v + omega * TRACK / 2.0) / WHEEL_R
        wl = max(-MAX_W, min(MAX_W, wl))
        wr = max(-MAX_W, min(MAX_W, wr))
        self.cmd_wl, self.cmd_wr = wl, wr
        if not ESP32_SIM:
            self.motors["wheel_fl"].setVelocity(wl)
            self.motors["wheel_rl"].setVelocity(wl)
            self.motors["wheel_fr"].setVelocity(wr)
            self.motors["wheel_rr"].setVelocity(wr)

    def sense(self):
        f = tof_to_m(self.tof["tof_front"].getValue())
        l = tof_to_m(self.tof["tof_left"].getValue())
        r = tof_to_m(self.tof["tof_right"].getValue())
        if ESP32_SIM:
            # VL53L0X-like noise: Gaussian, growing with distance, plus
            # occasional no-return dropouts reported as max range.
            noisy = []
            for d in (f, l, r):
                if self.rng.random() < TOF_DROPOUT_P:
                    noisy.append(2.0)
                else:
                    sigma = TOF_NOISE_BASE_M + TOF_NOISE_PCT * d
                    noisy.append(max(0.02, min(2.0, d + self.rng.gauss(0.0, sigma))))
            f, l, r = noisy
        return f, l, r

    def update_odometry(self):
        # Uses quantised encoder angles when ESP32_SIM is on (set by
        # _read_encoders()), exact angles when it is off.
        if self.q_enc is not None:
            cur = self.q_enc
        else:
            cur = {n: self.enc[n].getValue() for n in self.enc}
        if self.prev_enc is None:
            self.prev_enc = cur
            return
        dl = (cur["enc_fl"] - self.prev_enc["enc_fl"]
              + cur["enc_rl"] - self.prev_enc["enc_rl"]) / 2.0 * WHEEL_R
        dr = (cur["enc_fr"] - self.prev_enc["enc_fr"]
              + cur["enc_rr"] - self.prev_enc["enc_rr"]) / 2.0 * WHEEL_R
        self.prev_enc = cur
        d = (dl + dr) / 2.0
        dth = (dr - dl) / TRACK
        self.odom[2] = wrap_pi(self.odom[2] + dth)
        self.odom[0] += d * math.cos(self.odom[2])
        self.odom[1] += d * math.sin(self.odom[2])
        self.dist_m += abs(d)

    def update_nav(self):
        p = self.gps.getValues()
        if self.prev_gps is not None:
            dx = p[0] - self.prev_gps[0]
            dz = p[2] - self.prev_gps[2]
            if math.hypot(dx, dz) > 0.002:
                self.heading = math.atan2(dz, dx)
        self.prev_gps = p
        self.nav_xy = (p[0], p[2])

    def check_collision(self, f, l, r):
        m = min(f, l, r)
        if m < 0.09 and not self.in_collision:
            self.in_collision = True
            self.collisions += 1
            print("[COLLISION #%d] t=%.1fs" % (self.collisions, self.robot.getTime() - self.t0))
        elif m > 0.16:
            self.in_collision = False

    # ---------------- modes ----------------
    def set_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            print("[MODE] %s" % mode)
            if mode == "WAYPOINT" and self.wp_idx >= len(WAYPOINTS):
                self.wp_idx = 0

    def do_teleop(self):
        v, w = 0.0, 0.0
        if self.held(Keyboard.UP, ord("w"), ord("W")):
            v += 0.22
        if self.held(Keyboard.DOWN, ord("s"), ord("S")):
            v -= 0.22
        if self.held(Keyboard.LEFT, ord("a"), ord("A")):
            w += 1.0
        if self.held(Keyboard.RIGHT, ord("d"), ord("D")):
            w -= 1.0
        self.drive(v, w)

    def do_avoid(self, f, l, r):
        if f < 0.40:
            w = 1.0 if l > r else -1.0   # turn toward the clearer side
            self.drive(0.0, w)
        elif l < 0.30 and l < r:
            self.drive(0.15, -0.5)        # veer right
        elif r < 0.30 and r < l:
            self.drive(0.15, 0.5)         # veer left
        else:
            w = max(-0.6, min(0.6, 0.8 * (r - l)))  # gentle lane centering
            self.drive(0.20, w)

    def do_wall(self, f, l, r):
        if f < 0.45:
            self.drive(0.05, 1.0)         # obstacle ahead: turn left
            return
        err = r - 0.35                    # right-hand wall, 35 cm target
        dt = TIME_STEP / 1000.0
        derr = (err - self.wall_err_prev) / dt
        self.wall_err_prev = err
        w = -1.4 * err - 0.15 * derr
        w = max(-1.2, min(1.2, w))
        if r > 1.2:
            w = -0.6                      # lost the wall: arc right to find it
        self.drive(0.18, w)

    def do_waypoint(self, f, l, r):
        if self.wp_idx >= len(WAYPOINTS):
            self.drive(0.0, 0.0)
            return
        gx, gz = WAYPOINTS[self.wp_idx]
        x, z = self.nav_xy
        dx, dz = gx - x, gz - z
        if math.hypot(dx, dz) < WP_TOL:
            print("[WP] reached %d/%d" % (self.wp_idx + 1, len(WAYPOINTS)))
            self.wp_idx += 1
            if self.wp_idx >= len(WAYPOINTS):
                print("[WP] all waypoints reached!")
                self.scoreboard()
                self.set_mode("STOP")
            return
        if f < 0.35:                      # reactive safety override
            w = 1.0 if l > r else -1.0
            self.drive(0.05, w)
            return
        err = wrap_pi(math.atan2(dz, dx) - self.heading)
        w = max(-1.2, min(1.2, 1.6 * err))
        self.drive(0.18, w)

    # ---------------- 6-DOF arm ----------------
    def set_arm_pose(self, name):
        """Drive the arm to a named CAD pose (joint-space degrees).

        Sign conventions (PROTO axes): J0 Webots angle = q0 (axis 0,-1,0);
        J1/J2/J3 Webots angles = q1/q2/q3 (axis 0,0,1, relative joints);
        J4 Webots angle = -q4 (axis 1,0,0, roll); fingers L = -phi,
        R = +phi (phi in radians, 0 open .. 15 deg closed). Targets are
        clamped to the servo limits in ARM_LIMITS_DEG.
        """
        if not self.arm_present or name not in ARM_POSES_DEG:
            return
        q_deg, phi_deg = ARM_POSES_DEG[name]
        lim = [ARM_LIMITS_DEG["J%d" % i] for i in range(5)]
        q = [max(lo, min(hi, v)) for v, (lo, hi) in zip(q_deg, lim)]
        phi = max(ARM_LIMITS_DEG["J5"][0],
                  min(ARM_LIMITS_DEG["J5"][1], phi_deg))
        targets = {
            "arm_j0": math.radians(q[0]),
            "arm_j1": math.radians(q[1]),
            "arm_j2": math.radians(q[2]),
            "arm_j3": math.radians(q[3]),
            "arm_j4": -math.radians(q[4]),
            "finger_L": -math.radians(phi),
            "finger_R": math.radians(phi),
        }
        for n, v in targets.items():
            self.arm_motors[n].setPosition(v)
        self.arm_pose_name = name
        print("[ARM] -> %s (q=%s, fingers %d deg)" % (name, list(q), phi))

    def arm_step(self):
        """Auto-demo timing only; the arm motors are position-controlled."""
        if not (self.arm_present and self.arm_auto):
            return
        now = self.robot.getTime()
        if now >= self.arm_auto_next:
            order = list(ARM_POSES_DEG.keys())
            name = order[self.arm_auto_idx % len(order)]
            self.arm_auto_idx += 1
            self.set_arm_pose(name)
            self.arm_auto_next = now + ARM_AUTO_PERIOD_S

    def arm_angles_deg(self):
        """Actual joint angles from the position sensors (CAD convention)."""
        if not self.arm_present:
            return None
        v = {n: self.arm_pos[n].getValue() for n in self.arm_pos}
        return {
            "q0": math.degrees(v["arm_pos_j0"]),
            "q1": math.degrees(v["arm_pos_j1"]),
            "q2": math.degrees(v["arm_pos_j2"]),
            "q3": math.degrees(v["arm_pos_j3"]),
            "q4": -math.degrees(v["arm_pos_j4"]),
            "phi": -math.degrees(v["arm_pos_finger_L"]),
        }

    # ---------------- keyboard ----------------
    def held(self, *codes):
        now = self.robot.getTime()
        return any(now - self.keys.get(c, -1e9) < 0.25 for c in codes)

    def handle_keys(self):
        while True:
            k = self.kb.getKey()
            if k == -1:
                break
            self.keys[k] = self.robot.getTime()
            if k == ord("1"):
                self.set_mode("AVOID")
            elif k == ord("2"):
                self.set_mode("WALL")
            elif k == ord("3"):
                self.set_mode("WAYPOINT")
            elif k == ord("4"):
                self.set_arm_pose("stowed")
            elif k == ord("5"):
                self.set_arm_pose("demo")
            elif k == ord("6"):
                self.set_arm_pose("reach")
            elif k == ord("7"):
                self.set_arm_pose("inspect")
            elif k == ord("8"):
                self.set_arm_pose("pick")
            elif k == ord("9"):
                self.arm_auto = not self.arm_auto
                self.arm_auto_next = self.robot.getTime()
                self.arm_auto_idx = 0
                print("[ARM] auto-demo %s" % ("ON" if self.arm_auto else "OFF"))
            elif k in (ord("t"), ord("T")):
                self.set_mode("TELEOP")
            elif k == ord(" ") or k == ord("0"):
                self.set_mode("STOP")
            elif k in (ord("r"), ord("R")):
                self.reset_run()
            elif k in (ord("p"), ord("P")):
                self.scoreboard()
            elif k in (ord("h"), ord("H")):
                self.print_help()

    def reset_run(self):
        node = self.robot.getSelf()
        node.getField("translation").setSFVec3f(START_POS)
        node.getField("rotation").setSFRotation([0, 1, 0, 0])
        node.resetPhysics()
        self.prev_enc = None
        self.prev_gps = None
        self.odom = [START_POS[0], START_POS[2], 0.0]
        self.nav_xy = (START_POS[0], START_POS[2])
        self.heading = 0.0
        self.wp_idx = 0
        self.collisions = 0
        self.in_collision = False
        self.dist_m = 0.0
        self.t0 = self.robot.getTime()
        self.wall_err_prev = 0.0
        # ESP32 layer: clear PID integrals, motor speeds, encoder counts
        # and re-seed the noise so a reset run is repeatable.
        for pid in self.pids.values():
            pid.reset()
        for enc in self.encoders.values():
            enc.reset(0.0)
        self.vel_actual = {n: 0.0 for n in self.motors}
        self.meas_vel = {n: 0.0 for n in self.motors}
        self.q_enc = None
        self._prev_q_enc = None
        self.cmd_wl = 0.0
        self.cmd_wr = 0.0
        self.rng = random.Random(NOISE_SEED)
        self.set_mode("STOP")
        if self.arm_present:
            self.set_arm_pose("stowed")
        print("[RESET] robot at start, metrics cleared")

    # ---------------- reporting ----------------
    def scoreboard(self):
        t = self.robot.getTime() - self.t0
        p = self.gps.getValues()
        odom_err = math.hypot(p[0] - self.odom[0], p[2] - self.odom[1])
        print("=========== SCOREBOARD ===========")
        print("mode.........: %s" % self.mode)
        print("run time.....: %.1f s" % t)
        print("distance.....: %.2f m (encoders)" % self.dist_m)
        print("collisions...: %d" % self.collisions)
        print("waypoints....: %d/%d" % (self.wp_idx, len(WAYPOINTS)))
        print("odom drift...: %.1f cm vs GPS" % (odom_err * 100.0))
        print("imu yaw......: %.2f rad%s" % (self.imu_yaw_noisy,
              " (noisy)" if ESP32_SIM else " (raw)"))
        if self.arm_present:
            a = self.arm_angles_deg()
            print("arm pose.....: %s (target)" % self.arm_pose_name)
            print("arm joints...: q0=%+.0f q1=%+.0f q2=%+.0f q3=%+.0f q4=%+.0f deg, fingers %.0f deg" % (
                a["q0"], a["q1"], a["q2"], a["q3"], a["q4"], a["phi"]))
        if ESP32_SIM:
            print("esp32 sim....: PID Kp=%.2f Ki=%.2f Kd=%.2f | encoder %d counts/rev (ASSUMED)" % (
                PID_KP, PID_KI, PID_KD, ENCODER_COUNTS_PER_WHEEL_REV))
        print("==================================")

    def print_help(self):
        print("---- CollegeCar controller ----")
        print("T teleop (WASD/arrows) | 1 avoid | 2 wall follow | 3 waypoints")
        print("Space stop | R reset | P scoreboard | H help")
        print("Arm: 4 stowed | 5 demo | 6 reach | 7 inspect | 8 pick | 9 auto-demo")
        if ESP32_SIM:
            print("ESP32 sim ON: wheel PID + encoder steps + ToF/IMU noise")
        print("-------------------------------")

    # ---------------- main loop ----------------
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            self.handle_keys()
            self.arm_step()
            self._read_encoders()
            f, l, r = self.sense()
            self.update_odometry()
            self.update_nav()
            self.check_collision(f, l, r)
            if self.mode == "TELEOP":
                self.do_teleop()
            elif self.mode == "AVOID":
                self.do_avoid(f, l, r)
            elif self.mode == "WALL":
                self.do_wall(f, l, r)
            elif self.mode == "WAYPOINT":
                self.do_waypoint(f, l, r)
            else:
                self.drive(0.0, 0.0)
            self.esp32_step()


if __name__ == "__main__":
    CollegeCar().run()
