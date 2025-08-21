// ESP8266_L298N_Controller.ino
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "Mi 10i";
const char* password = "tmkb2627";

ESP8266WebServer server(80);

// Motor pin mapping (avoid boot-pin conflicts)
const uint8_t IN1 = D1;   // left dir A
const uint8_t IN2 = D2;   // left dir B
const uint8_t ENA = D5;   // left PWM (ENA)
const uint8_t IN3 = D0;   // right dir A
const uint8_t IN4 = D7;   // right dir B
const uint8_t ENB = D6;   // right PWM (ENB)

// LED (onboard blue LED)
const uint8_t LEDPIN = LED_BUILTIN;  // NodeMCU: active LOW

unsigned long lastBlink = 0;
bool ledState = false;

void setMotorPWM(int leftPWM, int rightPWM);
void forward_simple();
void left_simple();
void right_simple();
void stop_motors();

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH); // LED off (active LOW on NodeMCU)

  // motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  analogWriteRange(255);
  analogWriteFreq(1000);

  Serial.printf("Connecting to WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);

  // Blink LED while connecting
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LEDPIN, LOW); delay(250);
    digitalWrite(LEDPIN, HIGH); delay(250);
    Serial.print(".");
  }
  // connected
  digitalWrite(LEDPIN, LOW); // solid on
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // HTTP endpoint
  server.on("/move", []() {
    String leftParam = server.arg("left");
    String rightParam = server.arg("right");
    String dir = server.arg("dir");

    if (leftParam.length() > 0 && rightParam.length() > 0) {
      int left = leftParam.toInt();
      int right = rightParam.toInt();
      setMotorPWM(left, right);
      server.send(200, "text/plain", "OK");
      Serial.printf("Set speeds L=%d R=%d\n", left, right);
      return;
    }

    if (dir.length() > 0) {
      if (dir == "forward") forward_simple();
      else if (dir == "left") left_simple();
      else if (dir == "right") right_simple();
      else if (dir == "stop") stop_motors();
      server.send(200, "text/plain", "OK");
      Serial.println("Dir: " + dir);
      return;
    }

    server.send(400, "text/plain", "bad request");
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  // if disconnected, try to reconnect and blink LED
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - lastBlink > 300) {
      lastBlink = now;
      ledState = !ledState;
      digitalWrite(LEDPIN, ledState ? LOW : HIGH); // blink
      Serial.print("WiFi disconnected, attempting reconnect...\n");
    }
    WiFi.reconnect();
  } else {
    digitalWrite(LEDPIN, LOW); // keep LED solid when connected
  }
}

void setMotorPWM(int leftPWM, int rightPWM) {
  // clamp 0..255
  if (leftPWM > 255) leftPWM = 255;
  if (leftPWM < -255) leftPWM = -255;
  if (rightPWM > 255) rightPWM = 255;
  if (rightPWM < -255) rightPWM = -255;

  // Left motor direction
  if (leftPWM >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, leftPWM);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftPWM);
  }

  // Right motor direction
  if (rightPWM >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, rightPWM);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightPWM);
  }
}

void forward_simple() {
  setMotorPWM(180, 180);
}
void left_simple() {
  setMotorPWM(80, 180);
}
void right_simple() {
  setMotorPWM(180, 80);
}
void stop_motors() {
  setMotorPWM(0, 0);
}
