#include <Arduino.h>
#include <Steering.h>
#include <Gyro.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

const int BUTTON_PIN = 9;
const int BACK_DISTANCE = 250;
const int FAR_DISTANCE = -1;
const int MAX_LEFT_ANGLE = 0;
const int MAX_RIGHT_ANGLE = 180;
const int MOTOR_SPEED = 180;

// Engine configuration
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor = AFMS.getMotor(1);
Servo servo;

void setup() {
  Serial.begin(9600);

  // Start sensors
  start_sensors();

  // Start engine
  AFMS.begin();
  motor->run(RELEASE);

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Servo
  servo.attach(10);

  // waiting for start button press
  while (true) {
    int state = digitalRead(BUTTON_PIN);
    if (state == 0)
      break;

    delay(10);
  }

  set_reference_frame();
}

bool frozen = false;
bool turning_right = false;
unsigned long lastToggle = 0;

void loop() {  
  int state = digitalRead(BUTTON_PIN);
  long now = millis();
  if (state == 0 && (now - lastToggle) > 200) {
    set_reference_frame();
    frozen = !frozen;
    lastToggle = now;

    if (frozen) {
      motor->run(RELEASE);
      servo.write(STRAIGHT_ANGLE);
    }
  }

  read_sensor_data();

  if (frozen) {
    return;
  };

  float angle = constrain(turning_angle, MAX_LEFT_ANGLE, MAX_RIGHT_ANGLE);

  if (
(    (front > 600 || front == FAR_DISTANCE)
    && (left + right) < 1100
    && left != FAR_DISTANCE
    && right != FAR_DISTANCE) 
    || abs(angle) == 30 // also check if has a target then it should go around the pillar

  ) {
    motor->run(FORWARD);
    motor->setSpeed(MOTOR_SPEED);
    Serial.println("PID");  
    servo.write(angle);
    return;
  }

  motor->setSpeed(MOTOR_SPEED - 30);

  Serial.println("turning");
  turning_right = (right > left && left != FAR_DISTANCE) || right == -1;
  servo.write(turning_right ? MAX_RIGHT_ANGLE : MAX_LEFT_ANGLE);

  if (front < BACK_DISTANCE) {
    servo.write(turning_right ? MAX_LEFT_ANGLE : MAX_RIGHT_ANGLE);

    while (front < (BACK_DISTANCE + 200) && front != FAR_DISTANCE) {
      read_sensor_data();
      motor->run(BACKWARD);
      delay(10);
    }

    delay(50);
    motor->run(FORWARD);
  } else {
    motor->run(FORWARD);
  }

}