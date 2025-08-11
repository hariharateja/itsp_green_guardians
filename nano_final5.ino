#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ======================= Servo Control Setup =======================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  75
#define SERVOMAX  525

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// ======================= Motor & Sensor Pin Definitions =======================
#define ENA 9
#define IN1 8
#define IN2 7
#define ENB 3
#define IN3 5
#define IN4 4
#define TRIG_PIN 12
#define ECHO_PIN 11

// ======================= Behavior Constants =======================
#define SEARCH_TURN_DURATION_MS 350
#define OBSTACLE_AVOID_TURN_DURATION_MS 1050
// CORRECTED: Definition is placed here, before it is used.
#define MEDIAN_SAMPLES 5 // Number of samples for the stable distance sensor.

// Default speed, can be overwritten by a SPEED command from Python
int motorSpeed = 150;

// ======================= Arm Control Functions =======================
/*
 *  CORRECTED SERVO MAPPING:
 *  - Servo 11: Claw / Gripper
 *  - Servo 12: Arm Extension / Wrist
 *  - Servos 13 & 14: Main Arm Lift (paired for torque)
 *  - Servo 15: Base Rotation
*/

// MODIFIED: Sequence to pick up an object with direct arm movements
void pickUp() {
  // Start with the claw open
  pwm.setPWM(11, 0, angleToPulse(90));
  delay(1000);

  // MODIFIED: Lower the main arm directly
  pwm.setPWM(13, 0, angleToPulse(15));
  pwm.setPWM(14, 0, angleToPulse(196 - 15)); // Mirrored movement
  delay(1500); // Wait for movement and let the arm settle

  // Close the claw to grab the object
  pwm.setPWM(11, 0, angleToPulse(155));
  delay(1000);

  // MODIFIED: Lift the main arm up with the object directly
  pwm.setPWM(13, 0, angleToPulse(100));
  pwm.setPWM(14, 0, angleToPulse(196 - 100)); // Mirrored movement
  delay(1000); // Wait for the physical movement to complete
}

// Sequence to place a "Metal" object
void place1() {
  pwm.setPWM(15, 0, angleToPulse(145)); // Rotate base to metal position
  delay(1000);
  pwm.setPWM(13, 0, angleToPulse(100)); // Set arm lift to a neutral angle
  pwm.setPWM(14, 0, angleToPulse(196-100));
  pwm.setPWM(12, 0, angleToPulse(0)); // Extend arm forward
  delay(1000);
  // pwm.setPWM(11, 0, angleToPulse(120)); // Adjust claw angle
  // delay(1000);
  
  // Open claw to release the object
  pwm.setPWM(11, 0, angleToPulse(90)); 
  delay(1000);
  
  // Return to home position
  pwm.setPWM(11, 0, angleToPulse(155)); // Close claw
  delay(1000);
  pwm.setPWM(12, 0, angleToPulse(80)); // Retract arm
  delay(1000);
  pwm.setPWM(13, 0, angleToPulse(100)); // Set arm lift to home
  pwm.setPWM(14, 0, angleToPulse(196-100));
  delay(1000);
  pwm.setPWM(15, 0, angleToPulse(20)); // Rotate base to home
  delay(1000);
  Serial.println("DONE: PLACED METAL");
}

// Sequence to place a "Plastic" object
void place2() {
  pwm.setPWM(15, 0, angleToPulse(180)); // Rotate base to plastic position
  delay(1000);
  pwm.setPWM(12, 0, angleToPulse(0)); // Extend arm
  delay(1000);
  pwm.setPWM(13, 0, angleToPulse(110)); // Adjust arm lift
  pwm.setPWM(14, 0, angleToPulse(196-110));
  delay(1000);
  // pwm.setPWM(11, 0, angleToPulse(120)); // Adjust claw angle
  // delay(1000);

  // Open claw to release the object
  pwm.setPWM(11, 0, angleToPulse(90));
  delay(1000);

  // Return to home position
  pwm.setPWM(11, 0, angleToPulse(155)); // Close claw
  delay(1000);
  pwm.setPWM(12, 0, angleToPulse(80)); // Retract arm
  delay(1000);
  pwm.setPWM(13, 0, angleToPulse(100)); // Set arm lift to home
  pwm.setPWM(14, 0, angleToPulse(196-100));
  delay(1000);
  pwm.setPWM(15, 0, angleToPulse(20)); // Rotate base to home
  delay(1000);
  Serial.println("DONE: PLACED PLASTIC");
}

// Sequence to place a "Paper" object
void place3() {
  pwm.setPWM(15, 0, angleToPulse(55)); // Rotate base to paper position
  delay(1000);
  pwm.setPWM(12, 0, angleToPulse(180)); // Extend arm
  delay(1000);
  pwm.setPWM(13, 0, angleToPulse(105)); // Adjust arm lift
  pwm.setPWM(14, 0, angleToPulse(196-105));
  // pwm.setPWM(11, 0, angleToPulse(120)); // Adjust claw angle
  // delay(1000);

  // Open claw to release the object
  pwm.setPWM(11, 0, angleToPulse(90));
  delay(1000);
  
  // Return to home position
  pwm.setPWM(11, 0, angleToPulse(155)); // Close claw
  delay(1000);
  pwm.setPWM(12, 0, angleToPulse(80)); // Retract arm
  delay(1000);
  pwm.setPWM(13, 0, angleToPulse(100)); // Set arm lift to home
  pwm.setPWM(14, 0, angleToPulse(196-100));
  delay(1000);
  pwm.setPWM(15, 0, angleToPulse(20)); // Rotate base to home
  delay(1000);
  Serial.println("DONE: PLACED PAPER");
}

// ======================= Setup =======================
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  stopMotors();

  // Set arm to initial safe/home position on startup
  pwm.setPWM(11, 0, angleToPulse(155)); // Claw closed
  pwm.setPWM(12, 0, angleToPulse(80));  // Arm retracted
  pwm.setPWM(13, 0, angleToPulse(100)); // Arm lifted to neutral
  pwm.setPWM(14, 0, angleToPulse(196-100));
  pwm.setPWM(15, 0, angleToPulse(20));  // Base at home position

  Serial.println("READY");
}

// ======================= Main Loop =======================
void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("PICKUP:")) {
      String category = command.substring(7);
      pickUp();
      delay(500);

      if (category == "metal") {
        place3();
      } else if (category == "plastic") {
        place2();
      } else if (category == "paper") {
        place1();
      } else {
        Serial.println("ERROR: Invalid category");
      }
      Serial.println("PICKED");
    }
    else if (command == "FORWARD") {
      motorSpeed = 90;
      setMotors(motorSpeed, -motorSpeed);
    }
    else if (command == "LEFT") {
      motorSpeed = 255;
      setMotors(motorSpeed, motorSpeed);
    }
    else if (command == "RIGHT") {
      motorSpeed = 255;
      setMotors(-motorSpeed, -motorSpeed);
    }
    else if (command == "SEARCH_RIGHT") {
      motorSpeed = 255;
      timedTurn(-motorSpeed, -motorSpeed, SEARCH_TURN_DURATION_MS);
    }
    else if (command == "SEARCH_LEFT") {
      motorSpeed = 255;
      timedTurn(motorSpeed, motorSpeed, SEARCH_TURN_DURATION_MS);
    }
    else if (command == "TURN90") {
      motorSpeed = 255;
      timedTurn(-motorSpeed, -motorSpeed, OBSTACLE_AVOID_TURN_DURATION_MS);
    }
    else if (command == "STOP") {
      stopMotors();
    }
    else if (command.startsWith("SPEED:")) {
      int speedValue = command.substring(6).toInt();
      if (speedValue >= 0 && speedValue <= 255) {
        motorSpeed = speedValue;
      }
    }
    else if (command == "DISTANCE") {
      float distance = getDistanceCm_improved();
      Serial.print("DISTANCE:");
      Serial.println(distance);
    }
  }

  // --- INDEPENDENT OBSTACLE DETECTION ---
  static unsigned long lastDistanceCheck = 0;
  if (millis() - lastDistanceCheck > 250) {
    float distance = getDistanceCm_improved();
    
    if (distance > 0 && distance < 5) {
      stopMotors();
      Serial.print("OBSTACLE:");
      Serial.println(distance);
    }
    lastDistanceCheck = millis();
  }
}

// ======================= Motor Control Functions =======================
void setMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (leftSpeed < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
  analogWrite(ENA, abs(leftSpeed));

  if (rightSpeed > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (rightSpeed < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
  analogWrite(ENB, abs(rightSpeed));
}

void stopMotors() {
  setMotors(0, 0);
}

void timedTurn(int leftSpeed, int rightSpeed, int duration) {
  setMotors(leftSpeed, rightSpeed);
  delay(duration);
  stopMotors();
}


// ======================= Sensor Functions =======================

// A much more stable function to measure distance using a median filter.
float getDistanceCm_improved() {
  float samples[MEDIAN_SAMPLES];
  
  // 1. Take a burst of readings
  for (int i = 0; i < MEDIAN_SAMPLES; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration_us = pulseIn(ECHO_PIN, HIGH, 25000);
    samples[i] = (float)duration_us * 0.0343 / 2.0;
    
    delay(10); // Small delay between pings to prevent interference
  }

  // 2. Sort the readings (simple bubble sort is fine for a small array)
  for (int i = 0; i < MEDIAN_SAMPLES - 1; i++) {
    for (int j = 0; j < MEDIAN_SAMPLES - i - 1; j++) {
      if (samples[j] > samples[j+1]) {
        float temp = samples[j];
        samples[j] = samples[j+1];
        samples[j+1] = temp;
      }
    }
  }

  // 3. Return the middle value (the median)
  return samples[MEDIAN_SAMPLES / 2];
}

// OLD version is kept here for reference but is no longer used.
float getDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration_us = pulseIn(ECHO_PIN, HIGH, 25000);
  return (float)duration_us * 0.0343 / 2.0;
}