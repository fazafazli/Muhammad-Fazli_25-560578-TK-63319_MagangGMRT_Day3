#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

MPU6050 mpu;
Servo servo1, servo2, servo3, servo4, servo5;

unsigned long lastTime = 0;
unsigned long yawStopTime = 0, rollStopTime = 0, pitchStopTime = 0;
bool yawStopped = false, yawMoving = false;
bool rollStopped = false, rollMoving = false;
bool pitchStopped = false, pitchMoving = false;
bool pirState = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  
  servo1.attach(13);
  servo2.attach(12);
  servo3.attach(14);
  servo4.attach(27);
  servo5.attach(26);
  
  // posisi awal
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
  
  pinMode(25, INPUT);
  
  lastTime = millis();
  Serial.println("Ready!");
}

void loop() {
  // PIR Detection
  if (digitalRead(25) == HIGH && !pirState) {
    servo1.write(45); servo2.write(135); servo3.write(60); servo4.write(120); servo5.write(180);
    delay(1000);
    servo1.write(90); servo2.write(90); servo3.write(90); servo4.write(90); servo5.write(90);
    pirState = true;
  } else if (digitalRead(25) == LOW) pirState = false;
  
  // gyroscope data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // konversi ke degrees per second
  float rollRate = gx / 131.0;   // Roll (rotation around X-axis)
  float pitchRate = gy / 131.0;  // Pitch (rotation around Y-axis)
  float yawRate = gz / 131.0;    // Yaw (rotation around Z-axis)
  
  // Roll (berlawanan arah)
  if (abs(rollRate) > 5) {  // threshold in degrees/second
    int pos1 = constrain(servo1.read() - int(rollRate * 0.5), 0, 180);
    int pos2 = constrain(servo2.read() - int(rollRate * 0.5), 0, 180);
    servo1.write(pos1);
    servo2.write(pos2);
    rollMoving = true;
    rollStopped = false;
  } else if (rollMoving && !rollStopped) {
    rollStopTime = millis();
    rollStopped = true;
    rollMoving = false;
  }
  
  // Pitch (searah) - responds to rotation
  if (abs(pitchRate) > 5) {  // threshold in degrees/second
    int pos3 = constrain(servo3.read() + int(pitchRate * 0.5), 0, 180);
    int pos4 = constrain(servo4.read() + int(pitchRate * 0.5), 0, 180);
    servo3.write(pos3);
    servo4.write(pos4);
    pitchMoving = true;
    pitchStopped = false;
  } else if (pitchMoving && !pitchStopped) {
    pitchStopTime = millis();
    pitchStopped = true;
    pitchMoving = false;
  }
  
  // Yaw (mengikuti)
  if (abs(yawRate) > 5) {
    int pos5 = constrain(servo5.read() + int(yawRate * 0.5), 0, 180);
    servo5.write(pos5);
    yawMoving = true;
    yawStopped = false;
  } else if (yawMoving && !yawStopped) {
    yawStopTime = millis();
    yawStopped = true;
    yawMoving = false;
  }
  
  if (yawStopped && millis() - yawStopTime <= 1000) {
    servo5.write(90);
    yawStopped = false;
  }
  
  delay(50);
}