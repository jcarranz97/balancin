#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"

void mpu6050_init(void);

uint8_t mpu6050_read_byte(uint8_t reg);

#endif  //__MPU6050_H__

