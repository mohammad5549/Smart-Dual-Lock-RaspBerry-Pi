import RPi.GPIO as GPIO
import time
import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import pickle
import threading

# ========== GLOBAL STATE ==========
face_detected = False
pin_entered = False
entered_pin = ""

# ========== CONFIG ==========
CORRECT_PIN = "1234"
authorized_names = ["mohammad"]
cv_scaler = 4

# ========== LOAD ENCODINGS ==========
with open("encodings.pickle", "rb") as f:
    data = pickle.loads(f.read())
known_face_encodings = data["encodings"]
known_face_names = data["names"]

# ========== CAMERA ==========
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# ========== GPIO Setup ==========
# Keypad GPIO
L1, L2, L3, L4 = 16, 20, 21, 5
C1, C2, C3, C4 = 6, 13, 19, 26
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
for pin in [L1, L2, L3, L4]:
    GPIO.setup(pin, GPIO.OUT)
for pin in [C1, C2, C3, C4]:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Relay GPIOs (Two Motors)
RELAY_MOTOR_1 = 23
RELAY_MOTOR_2 = 24
GPIO.setup(RELAY_MOTOR_1, GPIO.OUT)
GPIO.setup(RELAY_MOTOR_2, GPIO.OUT)
GPIO.output(RELAY_MOTOR_1, GPIO.LOW)
GPIO.output(RELAY_MOTOR_2, GPIO.LOW)

# Keypad matrix
KEYPAD = [
    ["1", "2", "3", "A"],
    ["4", "5", "6", "B"],
    ["7", "8", "9", "C"],
    ["*", "0", "#", "D"]
]
row_pins = [L1, L2, L3, L4]
col_pins = [C1, C2, C3, C4]

# ========== FUNCTIONS ==========

def face_detection_loop():
    global face_detected
    while not face_detected:
        frame = picam2.capture_array()
        small_frame = cv2.resize(frame, (0, 0), fx=1/cv_scaler, fy=1/cv_scaler)
        rgb_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        encodings = face_recognition.face_encodings(rgb_frame)
        for encoding in encodings:
            matches = face_recognition.compare_faces(known_face_encodings, encoding)
            if True in matches:
                best_match = np.argmin(face_recognition.face_distance(known_face_encodings, encoding))
                name = known_face_names[best_match]
                if name in authorized_names:
                    face_detected = True
                    print(f"[✅] Authorized face detected: {name}")
                    return
        time.sleep(0.2)

def keypad_loop():
    global pin_entered, entered_pin
    pin = ""
    print("Enter 4-digit PIN:")
    while not pin_entered:
        for i, row in enumerate(KEYPAD):
            GPIO.output(row_pins[i], GPIO.HIGH)
            for j in range(4):
                if GPIO.input(col_pins[j]) == 1:
                    key = row[j]
                    if key.isdigit():
                        pin += key
                        print(f"Pressed: {key}")
                        time.sleep(0.3)
                    if len(pin) == 4:
                        if pin == CORRECT_PIN:
                            print("[✅] Correct PIN entered.")
                            pin_entered = True
                            entered_pin = pin
                            return
                        else:
                            print("[❌] Incorrect PIN.")
                            pin = ""
            GPIO.output(row_pins[i], GPIO.LOW)
        time.sleep(0.05)

def trigger_motors(duration=2):
    print("[⚙️] Activating both motors...")
    GPIO.output(RELAY_MOTOR_1, GPIO.HIGH)
    GPIO.output(RELAY_MOTOR_2, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(RELAY_MOTOR_1, GPIO.LOW)
    GPIO.output(RELAY_MOTOR_2, GPIO.LOW)
    print("[🛑] Motors stopped.")

# ========== MAIN ==========
try:
    while True:
        face_detected = False
        pin_entered = False
        entered_pin = ""

        t1 = threading.Thread(target=face_detection_loop)
        t2 = threading.Thread(target=keypad_loop)
        t1.start()
        t2.start()

        t1.join()
        t2.join()

        if face_detected and pin_entered:
            print("\n🔓 Unlocked!\n")
            trigger_motors(duration=3)
        else:
            print("\n🔒 Locked.\n")

        time.sleep(2)

except KeyboardInterrupt:
    print("\n[INFO] Program terminated.")
    GPIO.output(RELAY_MOTOR_1, GPIO.LOW)
    GPIO.output(RELAY_MOTOR_2, GPIO.LOW)
    GPIO.cleanup()
    picam2.stop()








'''
import RPi.GPIO as GPIO
import time
import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import pickle
import threading

# ========== GLOBAL STATE ==========
face_detected = False
pin_entered = False
entered_pin = ""

# ========== CONFIG ==========
CORRECT_PIN = "1234"
authorized_names = ["mohammad"]
cv_scaler = 4

# ========== LOAD ENCODINGS ==========
with open("encodings.pickle", "rb") as f:
    data = pickle.loads(f.read())
known_face_encodings = data["encodings"]
known_face_names = data["names"]

# ========== CAMERA ==========
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# ========== GPIO Setup ==========
# Keypad GPIO
L1, L2, L3, L4 = 16, 20, 21, 5
C1, C2, C3, C4 = 6, 13, 19, 26
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
for pin in [L1, L2, L3, L4]:
    GPIO.setup(pin, GPIO.OUT)
for pin in [C1, C2, C3, C4]:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Relay GPIO (One Motor)
RELAY_MOTOR = 23
GPIO.setup(RELAY_MOTOR, GPIO.OUT)
GPIO.output(RELAY_MOTOR, GPIO.LOW)

KEYPAD = [
    ["1", "2", "3", "A"],
    ["4", "5", "6", "B"],
    ["7", "8", "9", "C"],
    ["*", "0", "#", "D"]
]
row_pins = [L1, L2, L3, L4]
col_pins = [C1, C2, C3, C4]

# ========== FUNCTIONS ==========

def face_detection_loop():
    global face_detected
    while not face_detected:
        frame = picam2.capture_array()
        small_frame = cv2.resize(frame, (0, 0), fx=1/cv_scaler, fy=1/cv_scaler)
        rgb_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        encodings = face_recognition.face_encodings(rgb_frame)
        for encoding in encodings:
            matches = face_recognition.compare_faces(known_face_encodings, encoding)
            if True in matches:
                best_match = np.argmin(face_recognition.face_distance(known_face_encodings, encoding))
                name = known_face_names[best_match]
                if name in authorized_names:
                    face_detected = True
                    print(f"[✅] Authorized face detected: {name}")
                    return
        time.sleep(0.2)

def keypad_loop():
    global pin_entered, entered_pin
    pin = ""
    print("Enter 4-digit PIN:")
    while not pin_entered:
        for i, row in enumerate(KEYPAD):
            GPIO.output(row_pins[i], GPIO.HIGH)
            for j in range(4):
                if GPIO.input(col_pins[j]) == 1:
                    key = row[j]
                    if key.isdigit():
                        pin += key
                        print(f"Pressed: {key}")
                        time.sleep(0.3)
                    if len(pin) == 4:
                        if pin == CORRECT_PIN:
                            print("[✅] Correct PIN entered.")
                            pin_entered = True
                            entered_pin = pin
                            return
                        else:
                            print("[❌] Incorrect PIN.")
                            pin = ""
            GPIO.output(row_pins[i], GPIO.LOW)
        time.sleep(0.05)

def trigger_motor(duration=2):
    print("[⚙️] Activating motor...")
    GPIO.output(RELAY_MOTOR, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(RELAY_MOTOR, GPIO.LOW)
    print("[🛑] Motor stopped.")

# ========== MAIN ==========
try:
    while True:
        face_detected = False
        pin_entered = False
        entered_pin = ""

        t1 = threading.Thread(target=face_detection_loop)
        t2 = threading.Thread(target=keypad_loop)
        t1.start()
        t2.start()

        t1.join()
        t2.join()

        if face_detected and pin_entered:
            print("\n🔓 Unlocked!\n")
            trigger_motor(duration=3)
        else:
            print("\n🔒 Locked.\n")

        time.sleep(2)

except KeyboardInterrupt:
    print("\n[INFO] Program terminated.")
    GPIO.output(RELAY_MOTOR, GPIO.LOW)
    GPIO.cleanup()
    picam2.stop()
'''



'''
import RPi.GPIO as GPIO
import time
import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import pickle
import threading

# ========== GLOBAL STATE ==========
face_detected = False
pin_entered = False
entered_pin = ""

# ========== CONFIG ==========
CORRECT_PIN = "1234"
authorized_names = ["mohammad"]
cv_scaler = 4

# ========== LOAD ENCODINGS ==========
with open("encodings.pickle", "rb") as f:
    data = pickle.loads(f.read())
known_face_encodings = data["encodings"]
known_face_names = data["names"]

# ========== CAMERA ==========
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# ========== GPIO for Keypad ==========
L1, L2, L3, L4 = 16, 20, 21, 5
C1, C2, C3, C4 = 6, 13, 19, 26
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
for pin in [L1, L2, L3, L4]:
    GPIO.setup(pin, GPIO.OUT)
for pin in [C1, C2, C3, C4]:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

KEYPAD = [
    ["1", "2", "3", "A"],
    ["4", "5", "6", "B"],
    ["7", "8", "9", "C"],
    ["*", "0", "#", "D"]
]
row_pins = [L1, L2, L3, L4]
col_pins = [C1, C2, C3, C4]

# ========== FUNCTIONS ==========

def face_detection_loop():
    global face_detected
    while not face_detected:
        frame = picam2.capture_array()
        small_frame = cv2.resize(frame, (0, 0), fx=1/cv_scaler, fy=1/cv_scaler)
        rgb_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        encodings = face_recognition.face_encodings(rgb_frame)
        for encoding in encodings:
            matches = face_recognition.compare_faces(known_face_encodings, encoding)
            if True in matches:
                best_match = np.argmin(face_recognition.face_distance(known_face_encodings, encoding))
                name = known_face_names[best_match]
                if name in authorized_names:
                    face_detected = True
                    print(f"[✅] Authorized face detected: {name}")
                    return
        time.sleep(0.2)

def keypad_loop():
    global pin_entered, entered_pin
    pin = ""
    print("Enter 4-digit PIN:")
    while not pin_entered:
        for i, row in enumerate(KEYPAD):
            GPIO.output(row_pins[i], GPIO.HIGH)
            for j in range(4):
                if GPIO.input(col_pins[j]) == 1:
                    key = row[j]
                    if key.isdigit():
                        pin += key
                        print(f"Pressed: {key}")
                        time.sleep(0.3)
                    if len(pin) == 4:
                        if pin == CORRECT_PIN:
                            print("[✅] Correct PIN entered.")
                            pin_entered = True
                            entered_pin = pin
                            return
                        else:
                            print("[❌] Incorrect PIN.")
                            pin = ""
            GPIO.output(row_pins[i], GPIO.LOW)
        time.sleep(0.05)

# ========== MAIN ==========
try:
    while True:
        face_detected = False
        pin_entered = False
        entered_pin = ""

        # Start both threads
        t1 = threading.Thread(target=face_detection_loop)
        t2 = threading.Thread(target=keypad_loop)
        t1.start()
        t2.start()

        # Wait for both to complete
        t1.join()
        t2.join()

        if face_detected and pin_entered:
            print("\n🔓 Unlocked!\n")
        else:
            print("\n🔒 Locked.\n")

        time.sleep(2)  # slight pause before restarting

except KeyboardInterrupt:
    print("\n[INFO] Program terminated.")
    GPIO.cleanup()
    picam2.stop()
'''







'''
import RPi.GPIO as GPIO
import time
import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import pickle

# ========== SETUP SECTION ==========

# Face recognition setup
print("[INFO] Loading face encodings...")
with open("encodings.pickle", "rb") as f:
    data = pickle.loads(f.read())
known_face_encodings = data["encodings"]
known_face_names = data["names"]
authorized_names = ["mohammad"]  # Case-sensitive

# Camera setup
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()
cv_scaler = 4

# Correct PIN
CORRECT_PIN = "1234"  # Change this to your desired PIN

# Keypad GPIO setup
L1, L2, L3, L4 = 16, 20, 21, 5
C1, C2, C3, C4 = 6, 13, 19, 26
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
for pin in [L1, L2, L3, L4]:
    GPIO.setup(pin, GPIO.OUT)
for pin in [C1, C2, C3, C4]:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

KEYPAD = [
    ["1", "2", "3", "A"],
    ["4", "5", "6", "B"],
    ["7", "8", "9", "C"],
    ["*", "0", "#", "D"]
]
row_pins = [L1, L2, L3, L4]
col_pins = [C1, C2, C3, C4]

# ========== FACE RECOGNITION FUNCTION ==========

def detect_authorized_face(timeout=10):
    start_time = time.time()
    while time.time() - start_time < timeout:
        frame = picam2.capture_array()
        small_frame = cv2.resize(frame, (0, 0), fx=1/cv_scaler, fy=1/cv_scaler)
        rgb_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        encodings = face_recognition.face_encodings(rgb_frame)
        for encoding in encodings:
            matches = face_recognition.compare_faces(known_face_encodings, encoding)
            if True in matches:
                best_match = np.argmin(face_recognition.face_distance(known_face_encodings, encoding))
                name = known_face_names[best_match]
                if name in authorized_names:
                    print(f"[INFO] Authorized face detected: {name}")
                    return True
        print("[INFO] Face not authorized or not detected. Retrying...")
        time.sleep(1)
    return False

# ========== KEYPAD PIN ENTRY FUNCTION ==========

def read_keypad_input():
    pin = ""
    print("Enter 4-digit PIN:")
    while len(pin) < 4:
        for i, row in enumerate(KEYPAD):
            GPIO.output(row_pins[i], GPIO.HIGH)
            for j in range(4):
                if GPIO.input(col_pins[j]) == 1:
                    key = row[j]
                    if key.isdigit():
                        print(f"Pressed: {key}")
                        pin += key
                        time.sleep(0.3)
            GPIO.output(row_pins[i], GPIO.LOW)
    return pin

# ========== MAIN PROGRAM ==========

try:
    while True:
        print("\n[INFO] Waiting for face recognition...")
        if detect_authorized_face():
            pin_entered = read_keypad_input()
            if pin_entered == CORRECT_PIN:
                print("✅ Unlocked!")
            else:
                print("❌ Incorrect PIN. Locked.")
        else:
            print("❌ Face not recognized. Locked.")
        time.sleep(1)

except KeyboardInterrupt:
    print("\n[INFO] Program stopped.")
    GPIO.cleanup()
    picam2.stop()





'''

'''
import RPi.GPIO as GPIO
import time

# Define pins for rows and columns
L1 = 16
L2 = 20
L3 = 21
L4 = 5

C1 = 6
C2 = 13
C3 = 19
C4 = 26

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Set row pins as output
GPIO.setup(L1, GPIO.OUT)
GPIO.setup(L2, GPIO.OUT)
GPIO.setup(L3, GPIO.OUT)
GPIO.setup(L4, GPIO.OUT)

# Set column pins as input with pull-down resistors
GPIO.setup(C1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Function to read a line (row)
def readLine(line, characters):
    GPIO.output(line, GPIO.HIGH)
    if GPIO.input(C1) == 1:
        print(characters[0])
    if GPIO.input(C2) == 1:
        print(characters[1])
    if GPIO.input(C3) == 1:
        print(characters[2])
    if GPIO.input(C4) == 1:
        print(characters[3])
    GPIO.output(line, GPIO.LOW)

# Main loop
try:
    while True:
        readLine(L1, ["1", "2", "3", "A"])
        readLine(L2, ["4", "5", "6", "B"])
        readLine(L3, ["7", "8", "9", "C"])
        readLine(L4, ["*", "0", "#", "D"])
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nProgram is stopped")
    GPIO.cleanup()
'''




'''
from pad4pi import rpi_gpio
import time

# Define keypad layout (standard 4x4)
KEYPAD = [
    ["1","2","3","A"],
    ["4","5","6","B"],
    ["7","8","9","C"],
    ["*","0","#","D"]
]

# Define GPIO pins connected to the keypad
COL_PINS = [5, 6, 13, 19]     # BCM numbering
ROW_PINS = [12, 16, 20, 21]

# Setup the keypad
factory = rpi_gpio.KeypadFactory()
keypad = factory.create_keypad(keypad=KEYPAD, row_pins=ROW_PINS, col_pins=COL_PINS)

# This function is called when a key is pressed
def print_key(key):
    print(f"Key Pressed: {key}")

# Register the handler
keypad.registerKeyPressHandler(print_key)

# Keep the script running
print("Press keys on the keypad (CTRL+C to stop)...")
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nExiting...")
    keypad.cleanup()
'''