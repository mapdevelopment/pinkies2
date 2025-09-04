#include <Steering.h>
#include <Gyro.h>
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <StateManager.h>
#include <Sorting.h>

const int SENSOR_WDITH = 95;
const uint8_t XSHUT_PINS[3] = { 6, 5, 8 };
const uint8_t ADDR[3] = { 0x30, 0x31, 0x32 };
VL53L1X tof[3];

float front = 0, 
    left = 0, 
    right = 0,
    last_error = 0;

bool serial_active = false;
void start_sensors() {
    Wire.begin();

    // start time of flight sensors
    for (uint8_t i = 0; i < 3; i++) {
        pinMode(XSHUT_PINS[i], OUTPUT);
        digitalWrite(XSHUT_PINS[i], LOW);
        digitalWrite(XSHUT_PINS[i], HIGH);

        delay(10);

        if (!tof[i].init()) {
            Serial.print("Nepavyko paleisti sensoriu ");
            Serial.println(XSHUT_PINS[i]);
            while (true) {}
        }

        tof[i].setTimeout(500);
        tof[i].setDistanceMode(VL53L1X::Medium);
        tof[i].setMeasurementTimingBudget(50000);
        tof[i].setAddress(ADDR[i]);
        delay(50);
    }

    for (uint8_t i = 0; i < 3; i++) {
        tof[i].startContinuous(50);
    }

    // Start IMU gyroscope
    start_gyroscope();

    // Start Serial2
    Serial2.begin(9600);
}

const size_t buffer_size = 30;
unsigned long last_time = millis();
const float Kp = 0.2;
const float Kd = 0.2;
float turning_angle = STRAIGHT_ANGLE;
float track_buffer[buffer_size] = { 0 };
int track_tracker = 0;
bool track_ready = false;
const int DISTANCE_FROM_PILLAR = 300;

int16_t mm[3];

float custom_target;
bool set_custom_target = false;

void read_sensor_data() {
    float angle = get_gyro_angle();
    
    // Front - 0
    // Left - 2
    // Right - 1

    set_sensor_state(SENSOR_STATE_INDICATOR, LOW);

    for (uint8_t i = 0; i < 3; i++) {
        if (tof[i].dataReady()) {
            tof[i].read();    
        }

        if (tof[i].ranging_data.range_status == VL53L1X::RangeValid) 
            mm[i] = tof[i].ranging_data.range_mm;
        else mm[i] = -1;

        delay(50);
    }

    set_sensor_state(SENSOR_STATE_INDICATOR, HIGH);

    front = mm[0];
    left = mm[2];
    right = mm[1];

    float rad_angle = radians(angle);
    float width = (left + right) * cos(rad_angle);
    if (track_tracker == (buffer_size - 1)) {
        track_tracker = 0;
    } else {
        track_tracker++;
    }

    track_buffer[track_tracker] = width;
    const float track = get_dominant_cluster_average(
        buffer_size, track_buffer, 20
    );

    float distance = right * cos(rad_angle);

    float error = track / 2 - distance; 
    if (set_custom_target) {
        error = custom_target - distance;
    }

    if (!last_error) {
        last_error = error;
    }

    unsigned long now = millis();
    float delta_t = (now - last_time) / 1000.0f;
    last_time = now;


    const float derivative_delta = (error - last_error) / delta_t;
    last_error = error;

    turning_angle = STRAIGHT_ANGLE 
        - Kp * error
        - Kd * derivative_delta;

    
    Serial.print(custom_target);
    Serial.print(" ");
    Serial.print(distance);
    Serial.print(" ");
    Serial.println(error);


    if (Serial2.available()) {
        String line = Serial2.readStringUntil('\n');
        int commaIndex = line.indexOf(',');
        bool isGreen = line.substring(0, commaIndex).equalsIgnoreCase("true");
        float toRight = line.substring(commaIndex + 1).toFloat() * 10; // convert cm -> mm
        //Serial.print("Zalias: ");
        //Serial.println(isGreen);
        Serial.print("Atstumas x:");
        Serial.println(toRight);
        set_custom_target = true;
        /*
        custom_target = constrain(
            distance  + toRight * cos(rad_angle) + DISTANCE_FROM_PILLAR * (isGreen ? 1 : -1), 
            100, track - 10
        );*/

        if (isGreen) {
            custom_target = 800;
        } else {
            custom_target = 300;
        }
    }
}
