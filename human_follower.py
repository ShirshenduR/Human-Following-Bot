# human_follow_laptop.py
import cv2
import numpy as np
import time
import requests

# -------- CONFIG ----------
ESP_IP = "http://10.48.204.80"    # replace with the IP printed in ESP serial
CAM_URL = "http://100.79.162.7:8080/video"  # phone IP webcam stream (change)
MODEL_PROTO = "models/deploy.prototxt"
MODEL_WEIGHTS = "models/mobilenet_iter_73000.caffemodel"
USE_MOBILENET = True   # if False, script uses Haar face detection fallback
CONF_THRESHOLD = 0.5
FRAME_W, FRAME_H = 320, 240
BASE_SPEED = 150       # tune for your motors (0..255)
MAX_SPEED = 255
# PID params (tune these)
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
    # clamp and int
    left = int(max(-MAX_SPEED, min(MAX_SPEED, left)))
    right = int(max(-MAX_SPEED, min(MAX_SPEED, right)))
    url = f"{ESP_IP}/move?left={left}&right={right}"
    try:
        r = requests.get(url, timeout=0.5)
        # print("sent", left, right, r.status_code)
    except Exception as e:
        print("HTTP send error:", e)

# Load models
if USE_MOBILENET:
    net = cv2.dnn.readNetFromCaffe(MODEL_PROTO, MODEL_WEIGHTS)
else:
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Open camera (phone IP Webcam or local webcam)
cap = cv2.VideoCapture(CAM_URL)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

print("Starting. Press Ctrl+C to stop.")
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.2)
            continue
        h, w = frame.shape[:2]

        person_found = False
        cx = w/2

        if USE_MOBILENET:
            blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300,300)), 0.007843, (300,300), 127.5)
            net.setInput(blob)
            detections = net.forward()
            best_conf = 0
            best_box = None
            for i in range(detections.shape[2]):
                conf = float(detections[0,0,i,2])
                class_id = int(detections[0,0,i,1])
                # class 15 is 'person' in common SSD mapping
                if class_id == 15 and conf > CONF_THRESHOLD and conf > best_conf:
                    best_conf = conf
                    box = (detections[0,0,i,3:7] * np.array([w,h,w,h])).astype(int)
                    best_box = box
            if best_box is not None:
                startX, startY, endX, endY = best_box
                cx = (startX + endX) / 2.0
                person_found = True
                cv2.rectangle(frame, (startX,startY), (endX,endY), (0,255,0), 2)
                cv2.putText(frame, f"person {best_conf:.2f}", (startX, startY-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            if len(faces) > 0:
                (x,y,wf,hf) = faces[0]
                cx = x + wf/2.0
                person_found = True
                cv2.rectangle(frame, (x,y), (x+wf,y+hf), (255,0,0), 2)

        if person_found:
            # normalized error (-1 .. +1)
            error = (cx - w/2.0) / (w/2.0)
            tnow = time.time()
            dt = tnow - prev_time if 'prev_time' in globals() else 0.05
            prev_time = tnow
            control = pid_control(error, dt)  # negative -> steer left

            # Map control to wheel differential
            steer = control  # tune scale by Kp/Kd rather than multiply here
            left_speed = BASE_SPEED - steer * BASE_SPEED
            right_speed = BASE_SPEED + steer * BASE_SPEED

            # Optionally adjust forward speed by distance (box width)
            # For now keep forward constant. If too close, reduce speeds.
            send_speeds(left_speed, right_speed)
            cv2.putText(frame, f"err {error:.2f} L{int(left_speed)} R{int(right_speed)}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),2)
        else:
            send_speeds(0, 0)
            cv2.putText(frame, "No person", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255),2)

        cv2.imshow("Human Follow", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
