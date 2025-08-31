#include <math.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
unsigned long lastMs = 0;
float yaw = 0.0;
float straight = 0;

void start_gyroscope() {
    Wire.begin();

    if (!mpu.begin()) {
        Serial.println("Nepavyko paleisti MPU");
        while (true) {};
    }

    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    lastMs = millis();
}

float initial_straight = 0;
void set_reference_frame(int angle) {
    straight = yaw;
    if (initial_straight == 0) {
        initial_straight = straight;
    }
}

int N = 0;
unsigned long last_millis = 0;
float get_gyro_angle() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    yaw += g.gyro.z * dt * 180.0 / PI;
    yaw = fmod(yaw + 360.0f, 360.0f);

    unsigned long now_millis = millis();
    if ((yaw - initial_straight) < 15 && (last_millis - now_millis) > 15000) {
        N++;
        last_millis = now;
        Serial.print("ratas apvaziuotas ");
        Serial.println(N);
    }

    float diff = yaw - straight;
    if (diff > 180) 
        diff -= 360;
    
    if (diff < -180) 
        diff += 360;

    // When it gets back to PD mode it should rotate the symetry axis
    int multiplier = round(diff / 90);
    straight += 90 * multiplier;

    return diff;
}