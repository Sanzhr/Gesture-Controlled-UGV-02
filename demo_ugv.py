import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import threading

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

SERIAL_PORT = 'put here your USB port'
BAUDRATE = 115200

def normalize_depth(z, min_z=-0.4, max_z=0.1):
    z = np.clip(z, min_z, max_z)
    return int(((max_z - z) / (max_z - min_z)) * 100)

def detect_gesture(hand_landmarks, handedness):
    landmarks = hand_landmarks.landmark

    index_tip = landmarks[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_mcp = landmarks[mp_hands.HandLandmark.INDEX_FINGER_MCP]

    dx = index_tip.x - index_mcp.x
    dy = index_tip.y - index_mcp.y

    if abs(dx) > abs(dy):  
        if dx > 0:
            return "Right" if handedness == "Right" else "Left"
        else:
            return "Left" if handedness == "Right" else "Right"
    
    return None  

def read_serial():
    while ser.is_open:
        data = ser.readline().decode('utf-8').strip()
        if data:
            print(f"Received: {data}")

ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, dsrdtr=None)
ser.setRTS(False)
ser.setDTR(False)

serial_recv_thread = threading.Thread(target=read_serial)
serial_recv_thread.daemon = True
serial_recv_thread.start()

cap = cv2.VideoCapture(0)
with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

    last_command_time = 0
    command_interval = 0.1  
    last_command = None
    hand_detected = False  

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        image = cv2.flip(image, 1)
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        depth_text = "Depth: N/A"
        gesture_text = ""
        status_text = "Hand: Not detected"
        current_command = None
        proximity = 0
        hand_detected = False  

        if results.multi_hand_landmarks and results.multi_handedness:
            hand_detected = True
            status_text = "Hand: Detected"
            
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS)

                index_z = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].z
                proximity = normalize_depth(index_z)
                depth_text = f"Depth: {proximity}"

                gesture = detect_gesture(hand_landmarks, handedness.classification[0].label)
                if gesture:
                    gesture_text = gesture
                    current_command = gesture

        if hand_detected:
            if proximity <= 40:
                move_command = "Backward"
                speed = 0.5
            elif proximity >= 61:
                move_command = "Forward"
                speed = 0.5
            else:
                move_command = "Stop"
                speed = 0.0
        else:
            move_command = "Stop"
            speed = 0.0
            proximity = 0

        current_time = time.time()
        if current_time - last_command_time > command_interval:
            if not hand_detected:
                command = '{"T":1,"L":0.0,"R":0.0}'
            elif current_command:
                if current_command == "Left":
                    command = '{"T":1,"L":-0.3,"R":0.3}'
                elif current_command == "Right":
                    command = '{"T":1,"L":0.3,"R":-0.3}'
            else:
                if move_command == "Forward":
                    command = f'{{"T":1,"L":{speed},"R":{speed}}}'
                elif move_command == "Backward":
                    command = f'{{"T":1,"L":-{speed},"R":-{speed}}}'
                else:
                    command = '{"T":1,"L":0.0,"R":0.0}'

            try:
                ser.write(command.encode() + b'\n')
                print(f"Sent: {command}")
                last_command_time = current_time
                last_command = current_command if current_command else move_command
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                break

        cv2.putText(image, status_text, (50, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(image, depth_text, (50, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(image, f"Move: {move_command}", (50, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(image, f"Gesture: {gesture_text}", (50, 200),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('MediaPipe Hands', image)

        key = cv2.waitKey(5) & 0xFF
        if key == 27 or key == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
ser.close()