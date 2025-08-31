#ifndef STEERING_H
#define STEERING_H

void start_sensors();
void read_sensor_data();

const float STRAIGHT_ANGLE = 90.0f;

extern float front;
extern float left;
extern float right;

extern float turning_angle;

#endif // STEERING_H
