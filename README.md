# 🤖 Human-Following Robot

A Python-based human-following robot that uses computer vision, PID control, and an ESP8266 motor driver to follow a person in real-time. This project leverages OpenCV, MobileNet (or Haar cascades), and a simple web-controlled robot to create an autonomous following system.

---

## ⚡ Features

* **Real-Time Human Detection**
  Detects humans using MobileNet-SSD or faces using Haar cascades.

* **PID-Based Motor Control**
  Smoothly adjusts left and right motor speeds to follow the target.

* **ESP8266 Motor Integration**
  Sends HTTP commands to motor driver for precise movement.

* **Threaded Architecture**
  Separate threads for video capture and processing for low-latency operation.

* **Adjustable Parameters**
  Modify frame size, confidence threshold, PID gains, and speed easily.

* **Live Video Feed**
  Visualize the robot's target and control feedback with OpenCV window.

---

## 🛠 Hardware Requirements

* ESP8266 (NodeMCU or Wemos D1 Mini)
* Motor driver (compatible with ESP8266)
* 2–4 DC motors
* USB power supply for ESP8266 and motors
* Webcam or IP camera for video feed

---

## 💻 Software Requirements

* Python 3.10+
* OpenCV (`opencv-python`)
* NumPy (`numpy`)
* Requests (`requests`)
* Threading (built-in)

Optional (for face detection fallback):

* Haar cascades included with OpenCV

---

## 📦 Installation

1. **Clone this repository:**

```bash
git clone https://github.com/yourusername/human-follow-bot.git
cd human-follow-bot
```

2. **Create a virtual environment:**

```bash
python -m venv env
source env/bin/activate  # macOS/Linux
env\Scripts\activate     # Windows
```

3. **Install dependencies:**

```bash
pip install -r requirements.txt
```

4. **Update configuration in `bot.py`:**

```python
ESP_IP = "http://192.168.x.xxx"  # your ESP IP
CAM_URL = "http://your_camera_ip/video"
```

5. **Download MobileNet-SSD model (if using MobileNet):**

* `deploy.prototxt`
* `mobilenet_iter_73000.caffemodel`

Place them in the `models/` folder.

---

## 🏃 Usage

Run the main script:

```bash
python bot.py
```

**Controls:**

* Press **`q`** to quit the video feed and stop the robot.
* PID gains and speed can be tuned in the config section for smoother tracking.

---

## ⚙️ Configuration

```python
FRAME_W, FRAME_H = 320, 240     # Frame resolution
BASE_SPEED = 150                # Base motor speed
MAX_SPEED = 255                 # Maximum motor speed
CONF_THRESHOLD = 0.5            # Detection confidence
Kp, Ki, Kd = 1.0, 0.02, 0.25    # PID constants
```

---

## 🔧 How It Works

1. The camera captures video frames continuously.
2. Each frame is sent to the detector (MobileNet or Haar cascade).
3. The robot calculates the error between the target’s center and the frame center.
4. A PID controller computes speed adjustments for left and right motors.
5. Commands are sent via HTTP to ESP8266 to adjust motor speeds.
6. The robot follows the detected person smoothly in real-time.

---

## 🤝 Contribution

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Open a Pull Request

---