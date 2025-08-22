import cv2 as cv
import numpy as np
import time
from threading import Thread
import requests
import queue

class webCamStream:
    def __init__(self, stream_id):
        self.stream_id = stream_id
        self.vCap = cv.VideoCapture(self.stream_id)
        
        if not self.vCap.isOpened():
            print("[Exiting]: Error accessing webcam stream.")
            exit(0)
        
        fps_input_stream = int(self.vCap.get(5))
        print("FPS of input stream: {}".format(fps_input_stream))

        self.grabbed, self.frame = self.vCap.read()
        
        if not self.grabbed:
            print("[Exiting] No more frames to read")
            exit(0)
        
        self.stopped = True
        self.t = Thread(target=self.update, args=())
        self.t.daemon = True 

    def start(self):
        self.stopped = False
        self.t.start()

    def update(self):
        while True:
            if self.stopped:
                break

            self.grabbed, self.frame = self.vCap.read()
            if not self.grabbed or self.frame is None:
                print("No more frames to read or camera disconnected.")
                self.stopped = True
                break
        
        self.vCap.release()

    def read(self):
        return self.frame
    
    def stop(self):
        self.stopped = True

class ESPCommunication:
    def __init__(self, node_url):
        self.node_url = node_url
        self.command_queue = queue.Queue()
        self.last_command = None
        self.thread = Thread(target=self.process_commands, daemon=True)
        self.running = True
        self.thread.start()

    def send_command(self, command):
        if command != self.last_command:  # Avoid sending the same command repeatedly
            self.command_queue.put(command)
            self.last_command = command

    def process_commands(self):
        while self.running:
            try:
                command = self.command_queue.get(timeout=1)  
                requests.post(self.node_url + command, timeout=5)
                print(f"Sent command: {command}")
            except queue.Empty:
                continue
            except requests.exceptions.RequestException as e:
                print(f"Request failed: {e}")

    def stop(self):
        self.running = False
        self.thread.join()

COLOR_BOUNDS = [
    {"name": "Yellow", "lower": [20, 100, 100], "upper": [30, 255, 255]},
    {"name": "Green", "lower": [45, 100, 20], "upper": [75, 255, 255]},
    {"name": "Blue", "lower": [110, 50, 50], "upper": [130, 255, 255]},
    {"name": "Red", "lower": [0, 100, 100], "upper": [10, 255, 255]},
    {"name": "Black", "lower": [0, 0, 0], "upper": [180, 255, 30]},
    {"name": "White", "lower": [0, 0, 200], "upper": [180, 255, 255]},
]

def nothing(x):
    pass

node_url = "http://192.168.4.1/?State="
kernel = np.ones((5, 5), np.uint8)

webcam_stream = webCamStream("http://100.67.206.33:8080/video")  #IP address of webcam 
webcam_stream.start()

esp_comm = ESPCommunication(node_url)  # Initialize ESP communication thread

cv.namedWindow("Trackbars")
cv.createTrackbar("Color", "Trackbars", 0, len(COLOR_BOUNDS) - 1, nothing)

while True:
    frame = webcam_stream.read()
    if frame is None:
        print("Frame read failed. Exiting.")
        break

    frame = cv.resize(frame, (640, 480))  
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    cv.line(frame, (320, 0), (320, 480), (255, 0, 0), 2) 
    cv.line(frame, (0, 350), (640, 350), (255, 0, 0), 2)  

    color_index = cv.getTrackbarPos("Color", "Trackbars")
    selected_color = COLOR_BOUNDS[color_index]
    lower_bound = np.array(selected_color["lower"])
    upper_bound = np.array(selected_color["upper"])

    mask = cv.inRange(hsv, lower_bound, upper_bound)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

    segmented_img = cv.bitwise_and(frame, frame, mask=mask)
    contours, _ = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    current_command = None

    for contour in contours:
        area = cv.contourArea(contour)
        if area > 80:
            x, y, w, h = cv.boundingRect(contour)
            getX, getY = (x + (x + w)) / 2, (y + (y + h)) / 2

            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.circle(frame, (int(getX), int(getY)), 5, (0, 255, 255), -1)

            if getX < 180:
                print("LEFT")
                current_command = "R"
            elif getX > 440:
                print("RIGHT")
                current_command = "L"
            elif 180 < getX < 440 and getY < 350:
                print("FORWARD")
                current_command = "F"
            elif 180 < getX < 440 and getY > 350:
                print("BACKWARD")
                current_command = "B"
            elif 180 < getX < 440 and getY == 350:
                print("STOP")
                current_command = "S"

            if current_command:
                esp_comm.send_command(current_command)
            break  

    cv.putText(frame, f"Color: {selected_color['name']}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv.imshow('Frame', frame)
    cv.imshow('Mask', segmented_img)

    key = cv.waitKey(1)
    if key == ord('q'):
        esp_comm.send_command("S")  # Stop before exiting
        break

webcam_stream.stop()
esp_comm.stop()
cv.destroyAllWindows()