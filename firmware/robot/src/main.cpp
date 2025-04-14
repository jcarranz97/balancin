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
// #include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

// Priorities of our threads - higher numbers are higher priority
#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 2UL )
#define BLINK_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1UL )

// Stack sizes of our threads in words (4 bytes)
#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define BLINK_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
                        //
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
                                        //
void robot_task(__unused void *params) {
    printf("Robot Task is starting!!!\n");
    i2c_init(i2c_default, 400*1000);

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    if(mpu.testConnection()) printf("MPU6050 connection successful\n");
    else {printf("MPU6050 connection failed\n");}

    // ================================================================
    // ===                      INITIAL SETUP                       ===
    // ================================================================

    printf("MPU6050 initializing...\n");
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    printf("MPU6050 initialized\n");

    /* --- if you have calibration data then set the sensor offsets here --- */
    mpu.setXAccelOffset(-2723);
    mpu.setYAccelOffset(-1519);
    mpu.setZAccelOffset(1201);
    mpu.setXGyroOffset(182);
    mpu.setYGyroOffset(34);
    mpu.setZGyroOffset(-103);

    /* --- alternatively you can try this (6 loops should be enough) --- */
    // mpu.CalibrateAccel(6);
    // mpu.CalibrateGyro(6);

    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);                // turn on the DMP, now that it's ready
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;                        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        packetSize = mpu.dmpGetFIFOPacketSize();      // get expected DMP packet size for later comparison
    }
    else
    {                                          // ERROR!        1 = initial memory load failed         2 = DMP configuration updates failed        (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)", devStatus);
        vTaskDelay(2000);
    }
    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;

    while (true) {
        if (!dmpReady);                                                    // if programming failed, don't try to do anything
        mpuInterrupt = true;
        fifoCount = mpu.getFIFOCount();                                           // get current FIFO count
        if ((mpuIntStatus & 0x10) || fifoCount == 1024)                           // check for overflow (this should never happen unless our code is too inefficient)
        {
            mpu.resetFIFO();                                                      // reset so we can continue cleanly
            printf("FIFO overflow!");
        }
        else if (mpuIntStatus & 0x01)                                             // otherwise, check for DMP data ready interrupt (this should happen frequently)
        {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();        // wait for correct available data length, should be a VERY short wait
            mpu.getFIFOBytes(fifoBuffer, packetSize);                             // read a packet from FIFO
            fifoCount -= packetSize;                                              // track FIFO count here in case there is > 1 packet available
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[0] * 180 / PI;
            pitch = ypr[1] * 180 / PI;
            roll = ypr[2] * 180 / PI;
            printf("ypr: %f,\t %f,\t %f\n", yaw, pitch, roll);
        }
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
