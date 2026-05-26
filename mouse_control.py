import serial
import pyautogui
import threading
import queue
import time

PORT = "COM10"
BAUD = 9600
SENSITIVITY = 0.1
SCALE = 20.0

# Deadzone settings
DEADZONE_MIN = 1.5
DEADZONE_MAX = 15.0

pyautogui.FAILSAFE = False
pyautogui.PAUSE = 0

event_queue = queue.Queue()

def apply_deadzone(val):
    if abs(val) < DEADZONE_MIN:
        return 0.0
    if abs(val) > DEADZONE_MAX:
        return DEADZONE_MAX if val > 0 else -DEADZONE_MAX
    return val

def parse_line(line):
    """
    Parses line. Actions: ('move', dx, dy), ('Lclick',), ('Rclick',)
    """
    line = line.strip()
    if not line:
        return None

    # LEFT CLICK
    if line == "LCLICK":
        return ('Lclick',)
    
    # RIGHT CLICK
    if line == "RCLICK":
        return ('Rclick',)

    # Format: "roll,pitch,flex1,flex2"
    if "," in line and not line.startswith("R:"):
        try:
            parts = line.split(",")
            if len(parts) >= 2:
                x_val = float(parts[0])
                y_val = float(parts[1])

                x_val = apply_deadzone(x_val)
                y_val = apply_deadzone(y_val)

                dx = x_val * SENSITIVITY * SCALE
                dy = y_val * SENSITIVITY * SCALE

                return ('move', dx, dy)
        except ValueError:
            pass

    return None

def serial_worker():
    last_print_time = 0
    try:
        with serial.Serial(PORT, BAUD, timeout=1) as ser:
            print(f"Connected to {PORT} at {BAUD} baud.")
            while True:
                try:
                    line_bytes = ser.readline()
                    if not line_bytes:
                        continue

                    line = line_bytes.decode(errors="ignore")

                    # Debug print
                    if time.time() - last_print_time > 0.5:
                        print(f"[RX] {line.strip()}")
                        last_print_time = time.time()

                    action = parse_line(line)
                    if action:
                        event_queue.put(action)

                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    break
    except Exception as e:
        print(f"Could not open serial port: {e}")

def main():
    t = threading.Thread(target=serial_worker, daemon=True)
    t.start()

    print("Mouse control started . Press Ctrl+C to exit.")

    try:
        while True:
            if event_queue.empty():
                time.sleep(0.001)
                continue

            dx_total = 0.0
            dy_total = 0.0
            Lclicks = 0
            Rclicks = 0

            while not event_queue.empty():
                try:
                    action = event_queue.get_nowait()

                    if action[0] == 'move':
                        dx_total += action[1]
                        dy_total += action[2]

                    elif action[0] == 'Lclick':
                        Lclicks += 1

                    elif action[0] == 'Rclick':
                        Rclicks += 1

                except queue.Empty:
                    break

            # Move the mouse
            if abs(dx_total) > 0 or abs(dy_total) > 0:
                pyautogui.moveRel(dx_total, dy_total)

            # Left Click
            for _ in range(Lclicks):
                pyautogui.click()

            # Right Click
            for _ in range(Rclicks):
                pyautogui.click(button='right')

    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()
