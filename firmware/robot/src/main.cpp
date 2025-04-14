/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "FreeRTOS.h"
#include "task.h"
#include "hardware/i2c.h"
#include "MPU6050.h"

// Priorities of our threads - higher numbers are higher priority
#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 2UL )
#define BLINK_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1UL )

// Stack sizes of our threads in words (4 bytes)
#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define BLINK_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

MPU6050 mpu;
int16_t ax, ay, az; // Accelerometer data
int16_t gx, gy, gz; // Gyroscope data

void robot_task(__unused void *params) {
    printf("Robot Task is starting!!!\n");
    i2c_init(i2c_default, 400*1000);

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    printf("MPU6050 initializing...\n");
    mpu.initialize();
    printf("MPU6050 initialized\n");
    if(mpu.testConnection()) printf("MPU6050 connection successful\n");
    else {printf("MPU6050 connection failed\n");}

    // int16_t acceleration[3], gyro[3], temp;

    while (true) {
        printf("Reading sensor data!\n");
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        printf("Acc. X = %d, Y = %d, Z = %d\n", ax, ay, az);
        printf("Gyro. X = %d, Y = %d, Z = %d\n", gx, gy, gz);
        vTaskDelay(500);
    }
}


void main_task(__unused void *params) {
    // start the led blinking
    // xTaskCreate(blink_task, "BlinkThread", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
    xTaskCreate(robot_task, "JuanThread", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
    // int count = 0;
    while(true) {
        // printf("Hello from main task count=%u\n", count++);
        // vTaskDelay(1000);
    }
}

void vLaunch( void) {
    TaskHandle_t task;
    xTaskCreate(main_task, "MainThread", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &task);

#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    // we must bind the main task to one core (well at least while the init is called)
    vTaskCoreAffinitySet(task, 1);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main( void )
{
    stdio_init_all();

    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;
    rtos_name = "FreeRTOS SMP";
    printf("Starting %s on both cores:\n", rtos_name);
    vLaunch();
    return 0;
}
