# keypad_motor_unlock.py
from pad4pi import rpi_gpio
import RPi.GPIO as GPIO
import time
import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import pickle

# ----- Setup -----
# Load face encodings
print("[INFO] loading face encodings...")
with open("encodings.pickle", "rb") as f:
    data = pickle.loads(f.read())
known_face_encodings = data["encodings"]
known_face_names = data["names"]
authorized_names = ["john", "alice", "bob"]  # case-sensitive

# Setup PiCamera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()
time.sleep(2)

# Keypad setup
KEYPAD = [
    ["1","2","3","A"],
    ["4","5","6","B"],
    ["7","8","9","C"],
    ["*","0","#","D"]
]
ROW_PINS = [5, 6, 13, 19]
COL_PINS = [12, 16, 20, 21]

factory = rpi_gpio.KeypadFactory()
keypad = factory.create_keypad(keypad=KEYPAD, row_pins=ROW_PINS, col_pins=COL_PINS)
entered_code = []

# ----- Logic Functions -----
def facial_recognition_check():
    frame = picam2.capture_array()
    resized = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    encodings = face_recognition.face_encodings(rgb)

    for encoding in encodings:
        matches = face_recognition.compare_faces(known_face_encodings, encoding)
        face_distances = face_recognition.face_distance(known_face_encodings, encoding)
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = known_face_names[best_match_index]
            if name in authorized_names:
                print(f"Face matched: {name}")
                return True
    print("No authorized face detected.")
    return False

def on_key_press(key):
    global entered_code
    print(f"Key pressed: {key}")
    if key == "#":
        pin = "".join(entered_code)
        print("Entered PIN:", pin)
        if pin == "1234":
            print("PIN correct. Scanning face...")
            if facial_recognition_check():
                print("Unlocked")
            else:
                print("Locked")
        else:
            print("Incorrect PIN. Locked")
        entered_code = []
    elif key == "*":
        entered_code = []
    else:
        entered_code.append(key)

# Register keypad handler
keypad.registerKeyPressHandler(on_key_press)

# ----- Main Loop -----
print("System ready. Enter PIN:")
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")
    picam2.stop()
    GPIO.cleanup()