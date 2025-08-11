import numpy as np
from ultralytics import YOLO
import time
import subprocess
import serial
import os
import sys
import logging

# ========== LOGGING SETUP ==========
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ========== ROBOT PARAMETERS ==========
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
SERIAL_TIMEOUT = 3
MODEL_PATH = "best.pt"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_THRESHOLD = 25
CONFIDENCE_THRESHOLD = 0.40

# --- Corrected Distance & Speed Parameters ---
PICKUP_DISTANCE_CM = 22
OBSTACLE_DISTANCE_CM = 5
TRACKING_SPEED = 90
SEARCH_SPEED = 160
MIN_SPEED = 90

# --- Image Capture ---
IMAGE_PATH = "image.jpeg"
CAPTURE_DELAY_MS = 1500

# --- MODIFIED: Search Strategy Parameters ---
SEARCH_STEP_ANGLE_DEG = 15    # Estimated angle of a single search turn. Tune if needed.
INITIAL_SWEEP_ANGLE = 45      # First sweep will be 45 degrees right.
SWEEP_ANGLE_INCREMENT = 30    # Each subsequent sweep will be 30 degrees wider.
MAX_SWEEP_ANGLE = 120         # Stop searching if sweep exceeds this angle.


# ========== GLOBAL VARIABLES ==========
ser = None
model = None
class_names = None
locked_target_class_id = None

# ========== CORE FUNCTIONS ==========

def setup():
    """Initializes the serial connection to the Arduino and loads the YOLO model."""
    global ser, model, class_names
    logger.info("--- Initializing Setup ---")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        time.sleep(2)
        start_time = time.time()
        while time.time() - start_time < 10:
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').strip()
                logger.info(f"Arduino says: {response}")
                if "READY" in response:
                    logger.info("✅ Arduino connection established and ready.")
                    break
            time.sleep(0.1)
        else:
            logger.error("❌ Timed out waiting for Arduino 'READY' signal.")
            return False
    except serial.SerialException as e:
        logger.error(f"❌ Failed to connect to Arduino on {SERIAL_PORT}: {e}")
        return False
    try:
        model = YOLO(MODEL_PATH)
        class_names = model.names
        logger.info(f"✅ YOLO model '{MODEL_PATH}' loaded successfully.")
    except Exception as e:
        logger.error(f"❌ Failed to load YOLO model: {e}")
        return False
    logger.info("--- Setup Complete ---")
    return True

def send_command(command):
    """Sends a command string to the Arduino."""
    try:
        ser.write((command + "\n").encode())
        ser.flush()
        logger.info(f"⬆️  Sent command: {command}")
        time.sleep(0.1)
        return True
    except Exception as e:
        logger.error(f"❌ Failed to send command '{command}': {e}")
        return False

def send_speed_command(speed):
    """Sends a standardized speed command."""
    speed = max(50, min(255, int(speed)))
    return send_command(f"SPEED:{speed}")

def read_arduino_response():
    """Reads a single line from the Arduino serial buffer."""
    try:
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            if response:
                logger.info(f"⬇️  Arduino response: {response}")
                return response
        return None
    except Exception as e:
        logger.error(f"❌ Error reading from Arduino: {e}")
        return None

def measure_distance():
    """Commands the Arduino to measure and return the distance."""
    try:
        ser.flushInput()
        if not send_command("DISTANCE"): return None
        start_time = time.time()
        while time.time() - start_time < 2:
            response = read_arduino_response()
            if response and response.startswith("DISTANCE:"):
                try:
                    distance = float(response.split(":")[1])
                    if 0 < distance <= 400:
                        logger.info(f"📏 Measured Distance: {distance:.2f} cm")
                        return distance
                except (ValueError, IndexError):
                    logger.error(f"Could not parse distance from response: {response}")
                    return None
            time.sleep(0.05)
        logger.warning("Timed out waiting for DISTANCE response from Arduino.")
        return None
    except Exception:
        return None

def capture_and_predict():
    """Captures an image, runs YOLO prediction, and returns the best detection."""
    global locked_target_class_id
    try:
        logger.info("📸 Capturing and analyzing image...")
        if os.path.exists(IMAGE_PATH): os.remove(IMAGE_PATH)
        subprocess.run(
            ["rpicam-still", "-t", str(CAPTURE_DELAY_MS), "-o", IMAGE_PATH, "--width", str(FRAME_WIDTH), "--height", str(FRAME_HEIGHT), "--nopreview"],
            check=True, capture_output=True, text=True, timeout=10
        )
        if not os.path.exists(IMAGE_PATH):
            logger.error("❌ Image capture failed, file not found.")
            return None

        results = model.predict(IMAGE_PATH, verbose=False)
        if not results or not results[0].boxes or not results[0].boxes.conf.nelement() > 0:
            logger.info("...No objects detected in image.")
            locked_target_class_id = None
            return None

        detections_in_frame = []
        for i in range(len(results[0].boxes)):
            confidence = results[0].boxes.conf[i].item()
            if confidence >= CONFIDENCE_THRESHOLD:
                detections_in_frame.append({
                    "box": results[0].boxes.xyxy[i].cpu().tolist(),
                    "class_id": int(results[0].boxes.cls[i].item()),
                    "confidence": confidence
                })

        if not detections_in_frame:
            logger.info(f"...No detections with confidence >= {CONFIDENCE_THRESHOLD}")
            locked_target_class_id = None
            return None

        best_detection = None
        if locked_target_class_id is not None:
            for det in detections_in_frame:
                if det["class_id"] == locked_target_class_id:
                    if best_detection is None or det["confidence"] > best_detection["confidence"]:
                        best_detection = det
            if best_detection:
                 logger.info(f"✅ Re-acquired locked target: {class_names[locked_target_class_id]}")

        if best_detection is None:
            best_detection = max(detections_in_frame, key=lambda x: x['confidence'])
            locked_target_class_id = best_detection["class_id"]
            logger.info(f"🎯 New target lock: {class_names[locked_target_class_id]} ({best_detection['confidence']:.2f})")

        x1, _, x2, _ = best_detection["box"]
        final_detection = {
            "x_center": (x1 + x2) / 2,
            "class_name": class_names[best_detection["class_id"]],
            "confidence": best_detection["confidence"]
        }
        return final_detection
    except Exception as e:
        logger.error(f"❌ Error during capture/prediction: {e}", exc_info=True)
        return None

def cleanup():
    """Stops the robot and closes the serial connection cleanly."""
    logger.info("Initiating cleanup...")
    if ser and ser.is_open:
        try:
            send_command("STOP")
            time.sleep(0.5)
            ser.close()
            logger.info("✅ Serial connection closed.")
        except Exception as e:
            logger.error(f"❌ Error during serial cleanup: {e}")
    logger.info("🏁 Program terminated.")

# ========== MAIN STATE MACHINE ==========

def main():
    if not setup():
        sys.exit(1)

    state = "INITIAL_SCAN"
    global locked_target_class_id

    # This trick helps reset search variables when re-entering the search state
    search_vars = {}

    try:
        while True:
            response = read_arduino_response()
            if response and "OBSTACLE" in response:
                logger.warning(f"🚨 ARDUINO EMERGENCY STOP: {response}. Overriding to AVOIDING.")
                state = "AVOIDING_OBSTACLE"

            if state == "INITIAL_SCAN":
                logger.info("--- State: INITIAL_SCAN ---")
                detection = capture_and_predict()
                state = "ALIGNING" if detection else "SEARCHING"
                continue
            
            # =================================================================
            #               MODIFIED: BIDIRECTIONAL SEARCHING STATE
            # =================================================================
            elif state == "SEARCHING":
                # --- This block runs only ONCE when the state first changes to SEARCHING ---
                if 'direction' not in search_vars:
                    logger.info("--- State: SEARCHING (Initializing Sweep Search) ---")
                    locked_target_class_id = None
                    send_speed_command(SEARCH_SPEED)
                    
                    # Initialize the state variables for the sweep search
                    search_vars['direction'] = "RIGHT"  # Start by sweeping right
                    search_vars['sweep_limit'] = INITIAL_SWEEP_ANGLE
                    search_vars['angle_swept'] = 0

                # --- Main search loop ---
                logger.info(f"Searching {search_vars['direction']}... (Swept: {search_vars['angle_swept']}° / Limit: {search_vars['sweep_limit']}°)")

                # 1. Execute one turn step in the current direction
                command_to_send = f"SEARCH_{search_vars['direction']}"
                send_command(command_to_send)
                time.sleep(0.5) # Wait for the timed turn on Arduino to complete

                # 2. Update how far we've swept in this direction
                search_vars['angle_swept'] += SEARCH_STEP_ANGLE_DEG

                # 3. Look for the target
                detection = capture_and_predict()
                if detection:
                    send_command("STOP")
                    logger.info(f"🎯 Target found during search! Transitioning to ALIGNING.")
                    search_vars.clear() # Reset for the next time we search
                    state = "ALIGNING"
                    continue

                # 4. If the sweep in the current direction is complete, reverse and expand
                if search_vars['angle_swept'] >= search_vars['sweep_limit']:
                    logger.info(f"--- Sweep limit reached. Reversing direction. ---")
                    # Reverse direction
                    search_vars['direction'] = "LEFT" if search_vars['direction'] == "RIGHT" else "RIGHT"
                    
                    # The new sweep is the old one plus the increment
                    search_vars['sweep_limit'] += SWEEP_ANGLE_INCREMENT
                    
                    # Reset the swept angle for the new sweep
                    search_vars['angle_swept'] = 0
                    
                    # 5. Check if the search has become too wide
                    if search_vars['sweep_limit'] > MAX_SWEEP_ANGLE:
                        logger.warning(f"🏁 Max search angle exceeded. No objects found. Stopping.")
                        send_command("STOP")
                        break # Exit the main while loop

            # =================================================================
            #               MODIFIED: ALIGNING STATE
            # =================================================================
            elif state == "ALIGNING":
                logger.info("--- State: ALIGNING ---")
                send_speed_command(TRACKING_SPEED)
                detection = capture_and_predict()
                if not detection:
                    logger.warning("Target lost during alignment. Reverting to SEARCHING.")
                    state = "SEARCHING"
                    continue
                error_x = detection['x_center'] - FRAME_WIDTH / 2
                if abs(error_x) <= CENTER_THRESHOLD:
                    send_command("STOP")
                    logger.info("✅ Alignment complete. Transitioning to APPROACHING.")
                    state = "APPROACHING"
                else:
                    send_command("LEFT" if error_x < 0 else "RIGHT")
                    # MODIFIED: Reduced sleep time to fix overshooting
                    time.sleep(0.01)
                    send_command("STOP")

            elif state == "APPROACHING":
                logger.info("--- State: APPROACHING ---")
                logger.info("Camera OFF. Moving forward based on distance sensor ONLY.")
                while True:
                    distance = measure_distance()
                    if distance is None:
                        logger.warning("Could not measure distance during approach. Stopping and retrying.")
                        send_command("STOP")
                        time.sleep(1)
                        continue
                    if distance <= PICKUP_DISTANCE_CM:
                        logger.info(f"🎯 Target in pickup range ({distance:.2f} cm). Stopping and PICKING.")
                        send_command("STOP")
                        state = "PICKING"
                        break
                    else:
                        logger.info(f"Approaching... Current Distance: {distance:.2f} cm")
                        speed = MIN_SPEED + ((distance / (OBSTACLE_DISTANCE_CM * 2)) * (TRACKING_SPEED - MIN_SPEED))
                        send_speed_command(speed)
                        send_command("FORWARD")
                        time.sleep(0.05)

            elif state == "PICKING":
                logger.info("--- State: PICKING ---")
                send_command("STOP")
                if locked_target_class_id is not None:
                    category = class_names[locked_target_class_id]
                    command = f"PICKUP:{category.lower()}"
                    send_command(command)
                    logger.info(f"🤖 Sent pick command: {command}")
                    pick_start_time = time.time()
                    while time.time() - pick_start_time < 30:
                        response = read_arduino_response()
                        if response and "PICKED" in response:
                            logger.info(f"✅ Arduino confirmed pick and place sequence complete: {response}")
                            break
                        time.sleep(0.5)
                    else:
                        logger.warning("⚠️ Timed out waiting for pick confirmation. Proceeding anyway.")
                state = "TARGET_REACHED"

            elif state == "AVOIDING_OBSTACLE":
                locked_target_class_id = None
                search_vars.clear() # Clear search state if we were searching
                send_command("STOP")
                send_speed_command(SEARCH_SPEED)
                send_command("TURN90")
                time.sleep(2.0)
                state = "SEARCHING"

            elif state == "TARGET_REACHED":
                logger.info("✅ SUCCESS: Target picked and placed. Resetting for next target.")
                locked_target_class_id = None # Clear the lock to find new objects
                search_vars.clear() # Ensure search state is reset
                send_command("STOP") # Ensure robot is stopped before next scan
                time.sleep(1) # Pause briefly before starting the next cycle
                state = "INITIAL_SCAN"

    except KeyboardInterrupt:
        logger.info("🛑 Keyboard interrupt received. Exiting.")
    except Exception as e:
        logger.error(f"❌ Unexpected error in main loop: {e}", exc_info=True)
    finally:
        cleanup()

if __name__ == "__main__":
    main()