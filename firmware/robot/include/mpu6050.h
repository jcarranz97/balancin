#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"


bool mpu6050_is_present(void);
void mpu6050_init(void);
void mpu6050_reset(void);
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

#endif  //__MPU6050_H__

