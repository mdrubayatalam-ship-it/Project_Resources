"""
Combined Drone Tracking & Turret Control System with Proportional PID Speed Control
- Variable-speed PID for 1/32 Microstepping NEMA 17 Stepper Motor.
- Smooth Pan/Tilt tracking with zero overshooting.
- UDP Communication sending step frequencies to ESP32.
"""

import sys
import time
import select
import termios
import tty
import threading
import socket
import cv2
from gpiozero import OutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from picamera2 import Picamera2
from ultralytics import YOLO

# ============================================================
# PID CONTROLLER CLASS
# ============================================================
class PIDController:
    """PID Controller returning continuous control velocity."""
    def __init__(self, kp: float, ki: float, kd: float, output_limits=(-6400.0, 6400.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out, self.max_out = output_limits

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def update(self, error: float, dt: float = None) -> float:
        now = time.time()
        if dt is None:
            dt = (now - self.last_time) if self.last_time else 0.03
        self.last_time = now

        if dt <= 0.0:
            dt = 0.01

        p = self.kp * error

        # Anti-windup clamped integral
        self.integral += error * dt
        i_limit = max(abs(self.min_out), abs(self.max_out))
        self.integral = max(-i_limit, min(i_limit, self.integral))
        i = self.ki * self.integral

        d = self.kd * (error - self.last_error) / dt
        self.last_error = error

        output = p + i + d
        return max(self.min_out, min(self.max_out, output))

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None


# ============================================================
# HARDWARE CONFIGURATION
# ============================================================
RELAY_PIN = 17
LASER_PIN = 27
RELAY_ACTIVE_HIGH = True
RELAY_FIRE_TIME = 0.5
AUTO_FIRE_COOLDOWN = 1.5

# ESP32 UDP Network Settings
ESP32_IP = None
ESP32_CMD_PORT = 5007
CTRL_SEND_INTERVAL = 0.033   # 30Hz update rate to ESP32
LINK_STALE_SEC = 3.0

esp_lock = threading.Lock()
esp_state = {"ip": ESP32_IP, "last_hello": 0.0}
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ============================================================
# 1/32 MICROSTEPPING & PID TUNING
# ============================================================
# 1/32 Microstepping = 200 * 32 = 6,400 microsteps / revolution
MAX_PAN_SPEED = 6400.0        # Max speed: 1 rev/sec (6400 steps/sec)
MANUAL_PAN_SPEED = 2400.0     # Manual override pan speed

SERVO_MIN_ANGLE = 40
SERVO_MAX_ANGLE = 120
servo_target_angle = 40.0
pan_speed_target = 0.0        # Signed microsteps/sec (+ = Right, - = Left)

# Pan PID (Pixel Error -> Microsteps/sec Output)
PAN_KP = 14.0
PAN_KI = 0.2
PAN_KD = 1.5
PAN_DEADZONE_PX = 6           # Zero out pan speed if error is within ±6 pixels

# Tilt PID (Pixel Error -> Servo Angle Step Output)
TILT_KP = 0.035
TILT_KI = 0.002
TILT_KD = 0.008

pid_pan = PIDController(kp=PAN_KP, ki=PAN_KI, kd=PAN_KD, output_limits=(-MAX_PAN_SPEED, MAX_PAN_SPEED))
pid_tilt = PIDController(kp=TILT_KP, ki=TILT_KI, kd=TILT_KD, output_limits=(-4.0, 4.0))

FIRE_DEADZONE_X = 30
FIRE_DEADZONE_Y = 30

# Radar Configuration
ZONE_DESCRIPTIONS = {1: "left", 2: "center", 3: "right"}
RADAR_UDP_PORT = 5005
RADAR_STALE_SEC = 2.0
radar_lock = threading.Lock()
radar_state = {1: {"near": False, "last": 0.0}, 2: {"near": False, "last": 0.0}, 3: {"near": False, "last": 0.0}}

# Vision Configuration
MODEL_PATH = "droneUpdate_ncnn_model"
FRAME_W, FRAME_H = 640, 480
CAMERA_CENTER_X = FRAME_W // 2
CAMERA_CENTER_Y = FRAME_H // 2
CONF_THRESHOLD = 0.40
INFERENCE_IMGSZ = 320
DETECT_EVERY_N_FRAMES = 2

TARGET_CLASSES = {"Drone", "drone", "quadcopter"}
SAFE_CLASSES = {"Aeroplane", "Birds", "Helicopter"}

auto_mode = True
auto_fire_enabled = True
relay_active = False
relay_start_time = 0.0
last_auto_fire_time = 0.0

running = True
tracking_lock = threading.Lock()
current_detection = None

factory = PiGPIOFactory()
relay = OutputDevice(RELAY_PIN, active_high=RELAY_ACTIVE_HIGH, initial_value=False, pin_factory=factory)
laser = OutputDevice(LASER_PIN, initial_value=False, pin_factory=factory)
laser.on()


def send_ctrl():
    """Send variable step velocity and tilt angle to ESP32."""
    with esp_lock:
        ip = esp_state["ip"]
    if ip is None:
        return
    msg = f"CTRL {int(pan_speed_target)} {int(servo_target_angle)}"
    try:
        cmd_sock.sendto(msg.encode(), (ip, ESP32_CMD_PORT))
    except OSError:
        pass


def esp_link_ok():
    with esp_lock:
        return (time.time() - esp_state["last_hello"]) < LINK_STALE_SEC


def trigger_relay():
    global relay_active, relay_start_time
    if not relay_active:
        relay_active = True
        relay_start_time = time.monotonic()
        relay.on()
        print("\n[!] TARGET LOCKED - RELAY FIRED [!]")


def handle_relay_timeout():
    global relay_active
    if relay_active and (time.monotonic() - relay_start_time) >= RELAY_FIRE_TIME:
        relay.off()
        relay_active = False


def radar_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", RADAR_UDP_PORT))
    sock.settimeout(0.5)

    while running:
        try:
            data, addr = sock.recvfrom(1024)
        except (socket.timeout, OSError):
            continue

        msg = data.decode(errors="ignore").strip()

        if msg.startswith("ESP32 HELLO") or msg.startswith("RADAR "):
            with esp_lock:
                if ESP32_IP is None and esp_state["ip"] != addr[0]:
                    esp_state["ip"] = addr[0]
                esp_state["last_hello"] = time.time()

        if not msg.startswith("RADAR "):
            continue

        try:
            rest = msg[len("RADAR "):]
            id_str, tail = rest.split(":", 1)
            rid = int(id_str.strip())
        except Exception:
            continue

        near = "clear" not in tail
        with radar_lock:
            if rid in radar_state:
                radar_state[rid]["near"] = near
                radar_state[rid]["last"] = time.time()


def camera_thread():
    global current_detection, pan_speed_target, servo_target_angle, last_auto_fire_time

    model = YOLO(MODEL_PATH)
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (FRAME_W, FRAME_H), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)

    frame_count = 0
    fps = 0.0
    fps_counter = 0
    fps_start_time = time.time()

    while running:
        frame = picam2.capture_array()
        frame_count += 1
        fps_counter += 1

        if frame_count % DETECT_EVERY_N_FRAMES == 0:
            results = model.predict(frame, conf=CONF_THRESHOLD, imgsz=INFERENCE_IMGSZ, verbose=False)
            boxes = results[0].boxes
            best_detection = None
            best_conf = 0.0

            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    conf = float(box.conf[0])
                    if conf < CONF_THRESHOLD:
                        continue

                    cls_id = int(box.cls[0])
                    label = model.names[cls_id]

                    if conf > best_conf:
                        best_conf = conf
                        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        err_x = cx - CAMERA_CENTER_X
                        err_y = cy - CAMERA_CENTER_Y

                        best_detection = {
                            "label": label, "conf": conf, "box": (x1, y1, x2, y2),
                            "center": (cx, cy), "error": (err_x, err_y)
                        }

            with tracking_lock:
                current_detection = best_detection

        with tracking_lock:
            det = current_detection

        annotated = frame.copy()

        # ----------------------------------------------------
        # PROPORTIONAL PID TRACKING
        # ----------------------------------------------------
        if det is not None:
            label, conf = det["label"], det["conf"]
            x1, y1, x2, y2 = det["box"]
            cx, cy = det["center"]
            err_x, err_y = det["error"]

            is_target = label in TARGET_CLASSES
            is_safe = label in SAFE_CLASSES

            if auto_mode and is_target:
                # Calculate proportional step speed
                if abs(err_x) <= PAN_DEADZONE_PX:
                    pan_speed_target = 0.0
                    pid_pan.reset()
                else:
                    pan_speed_target = pid_pan.update(err_x)

                # Tilt update
                u_tilt = pid_tilt.update(err_y)
                servo_target_angle -= u_tilt
                servo_target_angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, servo_target_angle))

                send_ctrl()

                # Auto Fire trigger
                now = time.time()
                is_centered = (abs(err_x) <= FIRE_DEADZONE_X) and (abs(err_y) <= FIRE_DEADZONE_Y)
                if auto_fire_enabled and is_centered and (now - last_auto_fire_time >= AUTO_FIRE_COOLDOWN):
                    trigger_relay()
                    last_auto_fire_time = now

            elif auto_mode and not is_target:
                pid_pan.reset()
                pid_tilt.reset()
                pan_speed_target = 0.0
                send_ctrl()

            box_color = (0, 0, 255) if is_target else ((0, 255, 0) if is_safe else (0, 165, 255))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), box_color, 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(annotated, (CAMERA_CENTER_X, CAMERA_CENTER_Y), (cx, cy), (255, 0, 0), 2)
            cv2.putText(annotated, f"Err X:{err_x} Y:{err_y} Spd:{int(pan_speed_target)}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        else:
            if auto_mode:
                pid_pan.reset()
                pid_tilt.reset()
                pan_speed_target = 0.0
                send_ctrl()
            cv2.putText(annotated, "SEARCHING...", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Overlays
        cv2.drawMarker(annotated, (CAMERA_CENTER_X, CAMERA_CENTER_Y), (0, 255, 255), cv2.MARKER_CROSS, 18, 1)
        cv2.rectangle(annotated, (CAMERA_CENTER_X - FIRE_DEADZONE_X, CAMERA_CENTER_Y - FIRE_DEADZONE_Y),
                      (CAMERA_CENTER_X + FIRE_DEADZONE_X, CAMERA_CENTER_Y + FIRE_DEADZONE_Y), (255, 255, 0), 1)

        now_time = time.time()
        if (now_time - fps_start_time) >= 1.0:
            fps = fps_counter / (now_time - fps_start_time)
            fps_counter = 0
            fps_start_time = now_time
        cv2.putText(annotated, f"FPS: {fps:.1f}", (FRAME_W - 110, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        cv2.imshow("Proportional 1/32 Microstep Tracking", annotated)
        cv2.waitKey(1)

    cv2.destroyAllWindows()
    picam2.stop()


def handle_keyboard_input(key):
    global servo_target_angle, pan_speed_target, auto_mode

    if key == 'm':
        auto_mode = not auto_mode
        pid_pan.reset()
        pid_tilt.reset()
        pan_speed_target = 0.0
        send_ctrl()

    if not auto_mode:
        if key == 'a':
            pan_speed_target = -MANUAL_PAN_SPEED
            send_ctrl()
        elif key == 'd':
            pan_speed_target = MANUAL_PAN_SPEED
            send_ctrl()
        elif key in (' ', 'x'):
            pan_speed_target = 0.0
            send_ctrl()
        elif key == 'w':
            servo_target_angle = min(SERVO_MAX_ANGLE, servo_target_angle + 3)
            send_ctrl()
        elif key == 's':
            servo_target_angle = max(SERVO_MIN_ANGLE, servo_target_angle - 3)
            send_ctrl()

    if key == 'f':
        trigger_relay()


def main():
    global running, pan_speed_target

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    threads = [
        threading.Thread(target=radar_listener, daemon=True),
        threading.Thread(target=camera_thread, daemon=True),
    ]
    for t in threads: t.start()

    last_ctrl_send = 0.0

    try:
        while running:
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    running = False
                    continue
                handle_keyboard_input(key)

            handle_relay_timeout()

            now = time.monotonic()
            if now - last_ctrl_send >= CTRL_SEND_INTERVAL:
                last_ctrl_send = now
                send_ctrl()

            time.sleep(0.005)

    finally:
        running = False
        pan_speed_target = 0.0
        for _ in range(3):
            send_ctrl()
            time.sleep(0.02)

        for t in threads: t.join(timeout=2.0)
        relay.off()
        laser.off()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()