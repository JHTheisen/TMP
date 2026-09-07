import math
import time
import pygame
import serial
import msvcrt

# ---- CONFIG ----
SERIAL_PORT = "COM9"   # CHANGE THIS if needed (check Device Manager)
BAUD_RATE = 115200

# Right thumbstick axes (your mapping)
YAW_AXIS = 1
PITCH_AXIS = 2

DEADZONE = 0.05
SEND_INTERVAL = 0.02  # seconds
PRINT_INTERVAL = 0.10  # seconds
MAX_SLEW_RATE = 900.0  # position units per second (smooth ramping)
MAX_ACCEL = 2600.0     # acceleration limit for smoother starts/stops

# Button mapping (common Xbox-style layout)
KEYFRAME_BUTTONS = (2, 0)   # X button commonly 2, sometimes 0
B_BUTTONS = (1,)            # B button commonly 1
ZERO_BUTTONS = (3,)         # Y button commonly 3
LEVEL_BUTTONS = (4,)        # LB button commonly 4
NORTH_BUTTONS = (5,)        # RB button commonly 5
CANCEL_BUTTONS = ()

# ---- FUNCTIONS ----
def apply_deadzone(val):
    if abs(val) < DEADZONE:
        return 0.0
    return val


def clamp(val, low, high):
    return max(low, min(high, val))


def ramp_to(target, current, dt):
    delta = target - current
    if abs(delta) < 0.0001:
        return current

    max_step = MAX_ACCEL * dt
    max_step = max(max_step, MAX_SLEW_RATE * dt)
    step = min(abs(delta), max_step)
    return current + math.copysign(step, delta)


def read_serial_messages(ser, rx_buffer):
    if ser.in_waiting <= 0:
        return rx_buffer, []

    rx_buffer += ser.read(ser.in_waiting).decode(errors="ignore")
    lines = []

    while "\n" in rx_buffer:
        line, rx_buffer = rx_buffer.split("\n", 1)
        line = line.strip()
        if line:
            lines.append(line)

    return rx_buffer, lines


def center_smoothed_sticks():
    global current_yaw, current_pitch
    current_yaw = 0.0
    current_pitch = 0.0


# ---- MAIN ----
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found.")
    raise SystemExit(1)

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Controller:", joystick.get_name())
print("Controls:")
print("  - Right thumbstick: manual control")
print("  - X button: capture an AS5600 keyframe")
print("  - B button: play AS5600 keyframes on the one measured axis")
print("  - Y button: request AS5600 return-to-zero mode")
print("  - LB button: level with BNO085")
print("  - T key: tare current BNO level angle")
print("  - RB button: find north with BNO085")
print("  - M key: north + level mode")
print("  - C key: cancel auto mode")

# Open serial to ESP32
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
time.sleep(2)  # allow ESP32 to reset

# State for smoothing and AS5600 telemetry
current_yaw = 0.0
current_pitch = 0.0
last_time = time.monotonic()
last_print_time = 0.0

serial_rx_buffer = ""
last_encoder_angle = None
last_pitch_angle = None
last_heading = None
last_bno_pitch = None
last_bno_roll = None
keyframe_count = 0
current_mode = "manual"

try:
    while True:
        pygame.event.pump()

        # Handle button and keyboard events first
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button in B_BUTTONS:
                    center_smoothed_sticks()
                    ser.write(b"B\n")
                    current_mode = "keyframes"
                    print("B button: AS5600 keyframe playback requested")
                elif event.button in KEYFRAME_BUTTONS:
                    ser.write(b"K\n")
                    print("X button: AS5600 keyframe record requested")
                elif event.button in ZERO_BUTTONS:
                    center_smoothed_sticks()
                    ser.write(b"Z\n")
                    current_mode = "zero"
                    print("Y button: return-to-zero requested")
                elif event.button in LEVEL_BUTTONS:
                    center_smoothed_sticks()
                    ser.write(b"L\n")
                    current_mode = "level"
                    print("LB button: level mode requested")
                elif event.button in NORTH_BUTTONS:
                    center_smoothed_sticks()
                    ser.write(b"N\n")
                    current_mode = "north"
                    print("RB button: north mode requested")
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_l:
                    center_smoothed_sticks()
                    ser.write(b"L\n")
                    current_mode = "level"
                    print("L key: level mode requested")
                elif event.key == pygame.K_t:
                    ser.write(b"T\n")
                    print("T key: level tare requested")
                elif event.key == pygame.K_n:
                    center_smoothed_sticks()
                    ser.write(b"N\n")
                    current_mode = "north"
                    print("N key: north mode requested")
                elif event.key == pygame.K_m:
                    center_smoothed_sticks()
                    ser.write(b"M\n")
                    current_mode = "north+level"
                    print("M key: north+level mode requested")
                elif event.key == pygame.K_c:
                    center_smoothed_sticks()
                    ser.write(b"C\n")
                    current_mode = "manual"
                    print("C key: cancel auto mode requested")

        dt = max(time.monotonic() - last_time, 0.001)
        last_time = time.monotonic()

        # Terminal keyboard commands for Windows
        if msvcrt.kbhit():
            key = msvcrt.getwch()
            if key in ("l", "L"):
                center_smoothed_sticks()
                ser.write(b"L\n")
                current_mode = "level"
                print("L: level mode requested")
            elif key in ("t", "T"):
                ser.write(b"T\n")
                print("T: level tare requested")
            elif key in ("n", "N"):
                center_smoothed_sticks()
                ser.write(b"N\n")
                current_mode = "north"
                print("N: north mode requested")
            elif key in ("m", "M"):
                center_smoothed_sticks()
                ser.write(b"M\n")
                current_mode = "north+level"
                print("M: north+level mode requested")
            elif key in ("c", "C"):
                center_smoothed_sticks()
                ser.write(b"C\n")
                current_mode = "manual"
                print("C: cancel auto mode requested")


        # Read manual input
        yaw_input = apply_deadzone(joystick.get_axis(YAW_AXIS))
        pitch_input = apply_deadzone(joystick.get_axis(PITCH_AXIS))
        manual_yaw = int(round(yaw_input * 1000))
        manual_pitch = int(round(pitch_input * 1000))

        target_yaw = manual_yaw
        target_pitch = manual_pitch

        current_yaw = ramp_to(target_yaw, current_yaw, dt)
        current_pitch = ramp_to(target_pitch, current_pitch, dt)

        yaw_cmd = int(round(current_yaw))
        pitch_cmd = int(round(current_pitch))

        # Keep to a valid command range
        yaw_cmd = clamp(yaw_cmd, -1000, 1000)
        pitch_cmd = clamp(pitch_cmd, -1000, 1000)

        message = f"{yaw_cmd},{pitch_cmd}\n"
        ser.write(message.encode())

        serial_rx_buffer, serial_lines = read_serial_messages(ser, serial_rx_buffer)
        for line in serial_lines:
            if line.startswith("A,"):
                try:
                    last_encoder_angle = float(line.split(",", 1)[1])
                except ValueError:
                    pass
            elif line.startswith("P,"):
                try:
                    last_pitch_angle = float(line.split(",", 1)[1])
                except ValueError:
                    pass
            elif line.startswith("H,"):
                try:
                    last_heading = float(line.split(",", 1)[1])
                except ValueError:
                    pass
            elif line.startswith("O,"):
                try:
                    last_bno_pitch = float(line.split(",", 1)[1])
                except ValueError:
                    pass
            elif line.startswith("R,"):
                try:
                    last_bno_roll = float(line.split(",", 1)[1])
                except ValueError:
                    pass
            elif line.startswith("Keyframe ") and " recorded:" in line:
                try:
                    keyframe_count = int(line.split()[1])
                except (IndexError, ValueError):
                    pass
                print(f"\nESP32: {line}")
            else:
                print(f"\nESP32: {line}")
                if line in (
                    "Zero reached",
                    "Level reached",
                    "North heading reached",
                    "North and level reached",
                    "Control routine canceled",
                    "Keyframe playback complete",
                ) or line.endswith("unavailable") or line.endswith("read failed"):
                    current_mode = "manual"

        if last_encoder_angle is None:
            encoder_display = "Yaw AS5600: ---.--"
        else:
            encoder_display = f"Yaw AS5600: {last_encoder_angle:7.2f}"

        if last_pitch_angle is None:
            pitch_display = "Pitch AS5600: ---.--"
        else:
            pitch_display = f"Pitch AS5600: {last_pitch_angle:7.2f}"

        heading_display = (
            f"Heading: {last_heading:7.2f}" if last_heading is not None else "Heading: ---.--"
        )
        bno_pitch_display = (
            f"BNO Pitch: {last_bno_pitch:7.2f}" if last_bno_pitch is not None else "BNO Pitch: ---.--"
        )
        bno_roll_display = (
            f"BNO Roll: {last_bno_roll:7.2f}" if last_bno_roll is not None else "BNO Roll: ---.--"
        )

        now_print = time.monotonic()
        if now_print - last_print_time >= PRINT_INTERVAL:
            last_print_time = now_print
            print(
                f"Yaw: {yaw_cmd:5d}  Pitch: {pitch_cmd:5d}  "
                f"{encoder_display}  {pitch_display}  "
                f"{heading_display}  {bno_pitch_display}  {bno_roll_display}  "
                f"Mode: {current_mode}  Keyframes: {keyframe_count}",
                end="\r"
            )

        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    try:
        ser.write(b"0,0\n")
    except Exception:
        pass
    ser.close()
    pygame.quit()
