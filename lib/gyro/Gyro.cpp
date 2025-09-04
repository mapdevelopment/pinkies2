#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Steering.h>

Adafruit_MPU6050 mpu;
unsigned long lastMs = 0;
float yaw = 0.0;
float straight = 0;
bool clockwise = NAN;

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
    initial_straight = yaw;
    straight = yaw;
}

int N = 0;
int last_sector = 0;

float get_gyro_angle() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    yaw += g.gyro.z * dt * 180.0 / PI;
    yaw = fmod(yaw + 360.0f, 360.0f);

    float dif = yaw - initial_straight;
    if (dif < 0) dif *= -1;

    float sector = round(dif / 90);
    if (sector == 4) 
        sector = 0;

    if (clockwise && sector > last_sector) {
        last_sector = sector;
        Serial.println("countinam dideja");
        if (sector == 0) {
            N++;
        }
    } else if (!clockwise && last_sector > sector) {
        last_sector = sector;
        Serial.println("countinam mazeja");
        if (sector == 0) {
            N++;
        }
    }
    
    Serial.print("clockwise - ");
    Serial.println(clockwise);

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