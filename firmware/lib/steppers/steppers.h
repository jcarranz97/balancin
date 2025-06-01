#ifndef _STEPPERS_H_
#define _STEPPERS_H_


#include <stdio.h>
#include "pico/stdlib.h"
// #include "FreeRTOS.h"
// #include "queue.h"
// #include "task.h"

// Stepper motor pins
#define LEFT_MOTOR_STEP_PIN   14
#define LEFT_MOTOR_DIR_PIN    15
#define LEFT_MOTOR_ENABLE_PIN  10
#define RIGHT_MOTOR_STEP_PIN  12
#define RIGHT_MOTOR_DIR_PIN   13
#define RIGHT_MOTOR_ENABLE_PIN 11

#define MICROSTEPPING 16
#define STEPS_PER_REV 200
#define PWM_DIVIDER 8.0f  // PWM divider for 125 MHz clock

unsigned long get_step_delay_us(double target_rpm, int steps_per_rev);
void steppers_set_speed(int16_t rpm);
void stepper_motor_task(__unused void *params);
// void steppers_start(void);
void disable_steppers(void);
void enable_steppers(void);

#endif /* _STEPPERS_H_ */
