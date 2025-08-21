import cv2
import numpy as np
import time
import requests
import threading
from queue import Queue

# -------- CONFIG ----------
ESP_IP = "http://192.168.230.80"
CAM_URL = "http://192.168.230.254:8080/video"
MODEL_PROTO = "models/deploy.prototxt"
MODEL_WEIGHTS = "models/mobilenet_iter_73000.caffemodel"
USE_MOBILENET = True
CONF_THRESHOLD = 0.5
FRAME_W, FRAME_H = 320, 240
BASE_SPEED = 150
MAX_SPEED = 255
Kp, Ki, Kd = 1.0, 0.02, 0.25
# --------------------------

# PID state
integral = 0.0
prev_error = 0.0
prev_time = time.time()

def pid_control(error, dt):
    global integral, prev_error
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0.0
    out = Kp*error + Ki*integral + Kd*derivative
    prev_error = error
    return out

def send_speeds(left, right):
    left = int(max(-MAX_SPEED, min(MAX_SPEED, left)))
    right = int(max(-MAX_SPEED, min(MAX_SPEED, right)))
    url = f"{ESP_IP}/move?left={left}&right={right}"
    try:
        requests.get(url, timeout=0.2)
    except Exception as e:
        print("HTTP send error:", e)

# Load models
if USE_MOBILENET:
    net = cv2.dnn.readNetFromCaffe(MODEL_PROTO, MODEL_WEIGHTS)
else:
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Thread-safe queue
frame_queue = Queue(maxsize=2)

# Camera thread: constantly read frames
def camera_thread():
    cap = cv2.VideoCapture(CAM_URL)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue
        if not frame_queue.full():
            frame_queue.put(frame)
    cap.release()

# Processing thread: handle detection + PID + motor control
def processing_thread():
    global prev_time
    while True:
        frame = frame_queue.get()
        h, w = frame.shape[:2]
        person_found = False
        cx = w / 2

        if USE_MOBILENET:
            blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
            net.setInput(blob)
            detections = net.forward()
            best_conf = 0
            best_box = None
            for i in range(detections.shape[2]):
                conf = float(detections[0, 0, i, 2])
                class_id = int(detections[0, 0, i, 1])
                if class_id == 15 and conf > CONF_THRESHOLD and conf > best_conf:
                    best_conf = conf
                    box = (detections[0, 0, i, 3:7] * np.array([w, h, w, h])).astype(int)
                    best_box = box
            if best_box is not None:
                startX, startY, endX, endY = best_box
                cx = (startX + endX) / 2.0
                person_found = True
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.putText(frame, f"person {best_conf:.2f}", (startX, startY - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 1)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            if len(faces) > 0:
                (x, y, wf, hf) = faces[0]
                cx = x + wf / 2.0
                person_found = True
                cv2.rectangle(frame, (x, y), (x + wf, y + hf), (255, 0, 0), 2)

        if person_found:
            error = (cx - w / 2.0) / (w / 2.0)
            tnow = time.time()
            dt = tnow - prev_time if prev_time else 0.05
            prev_time = tnow
            control = pid_control(error, dt)

            left_speed = BASE_SPEED - control * BASE_SPEED
            right_speed = BASE_SPEED + control * BASE_SPEED
            send_speeds(left_speed, right_speed)

            cv2.putText(frame, f"err {error:.2f} L{int(left_speed)} R{int(right_speed)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            send_speeds(0, 0)
            cv2.putText(frame, "No person", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("Human Follow", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

# Start threads
print("Starting threaded human-follow bot. Press 'q' to quit.")
t1 = threading.Thread(target=camera_thread, daemon=True)
t2 = threading.Thread(target=processing_thread, daemon=True)
t1.start()
t2.start()

t1.join()
t2.join()