#ifndef GYRO_H
#define GYRO_H

extern bool clockwise;
extern int N;
void start_gyroscope();
void set_reference_frame(int angle = 0);
float get_gyro_angle();

#endif
