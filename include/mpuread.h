#ifndef _MPUREAD_H
#define _MPUREAD_H

#include "MPU6050_6Axis_MotionApps20.h"

extern MPU6050 mpu; // Make MPU6050 instance globally availabe
extern TaskHandle_t MPUTask;

int mpu_init(void);
void mpu_loop(void *pvParameters);

#endif