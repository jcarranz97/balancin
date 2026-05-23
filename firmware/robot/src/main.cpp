/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
// #include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "steppers.h"

// Priorities of our threads - higher numbers are higher priority
#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 2UL )
#define BLINK_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1UL )

// Stack sizes of our threads in words (4 bytes)
#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define BLINK_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

// Buttons
#define BUTTON_A 16
#define LED1_PIN 25


// UART Configuration
#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

MPU6050 mpu;


#define MAX_SAMPLES 1000  // How many samples to store (adjust if needed)

typedef struct {
    float timestamp;
    float target;
    float current;
    float error;
    float dT;
    float iTerm;
    float dTerm;
    float KpValue;
    float KiValue;
    float KdValue;
    float output;
} PIDLog;

PIDLog pidLogs[MAX_SAMPLES];
volatile int pidLogIndex = 0;

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

// Declare variables
// float Kp = 7;          // (P)roportional Tuning Parameter
// float Ki = 6;          // (I)ntegral Tuning Parameter
// float Kd = 3;          // (D)erivative Tuning Parameter
float target = 3.0;
float Kp = 300.00;          // (P)roportional Tuning Parameter
float Ki = 51.30;          // (I)ntegral Tuning Parameter
float Kd = 212.00;          // (D)erivative Tuning Parameter
float iTerm = 0;       // Used to accumulate error (integral)
float maxPTerm = 1000; // The maximum value that can be output
float maxITerm = 100; // The maximum value that can be output
float maxDTerm = 1000; // The maximum value that can be output
float lastTime = 0;    // Records the time the function was last called
float maxPID = 1000;    // The maximum value that can be output
float oldValue = 0;    // The last sensor value
bool running = false; // Whether the PID controller is running or not

uint32_t millis() {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}


QueueHandle_t button_event_queue;  // Queue handle


void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio == BUTTON_A) {
        // Send an event to the queue (from ISR)
        uint32_t event = events;
        xQueueSendFromISR(button_event_queue, &event, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void clear_pid_logs() {
    pidLogIndex = 0;
}


void button_task(void *pvParameters) {
    uint32_t event;
    absolute_time_t last_event_time = get_absolute_time();

    while (1) {
        if (xQueueReceive(button_event_queue, &event, portMAX_DELAY)) {
            // Basic debounce: wait a little time after event
            absolute_time_t now = get_absolute_time();
            int64_t diff_ms = absolute_time_diff_us(last_event_time, now) / 1000;

            if (diff_ms < 50) {
                // Too soon = likely bounce, ignore
                continue;
            }
            last_event_time = now;

            // Confirm button state after bounce time
            vTaskDelay(pdMS_TO_TICKS(10));  // Short delay to settle

            bool pressed = (gpio_get(BUTTON_A) == 0);  // 0 = pressed

            if (pressed) {
                // printf("Button A pressed\n");
                // motors.setSpeed(0, 0); or set your control flag
                // Toggle the running state variable
                // If controller is running, stop it
                if (running) {
                    // Reset PID parameters
                    iTerm = 0;
                    // Last time is set to 0 to avoid large jumps
                    lastTime = 0;
                    oldValue = 0;
                    printf("PID stopped\n");
                    // Disable the motors
                    disable_steppers();
                }
                // If controller is not running, start it
                else {
                    printf("PID started\n");
                    iTerm = 0;
                    // Last time is set to current time to avoid large jumps
                    // in the Integral term
                    lastTime = millis();
                    clear_pid_logs();
                    // Enable the motors
                    enable_steppers();
                }
                // Toggle the running state
                running = !running;
                gpio_put(LED1_PIN, running);  // Turn on LED if running
            } else {
                // printf("Button A released\n");
                // motors.setSpeed(1000, 1000); or clear your control flag
            }
        }
    }
}
/**
 * PID Controller
 * @param  (target)  The target position/value we are aiming for
 * @param  (current) The current value, as recorded by the sensor
 * @return The output of the controller
 */
float pid(float target, float current) {
	// Calculate the time since function was last called
	float thisTime = millis();
    // printf("This time: %f\n", thisTime);
	float dT = thisTime - lastTime;
	lastTime = thisTime;

	// Calculate error between target and current values
	float error = target - current;

	// Calculate the integral term
	iTerm += error * dT;

	// Calculate the derivative term (using the simplification)
	float dTerm = (oldValue - current) / dT;

	// Set old variable to equal new ones
	oldValue = current;

    float KpValue = (error * (Kp / 10.00));
    float KiValue = (iTerm * Ki / 100.00);
    float KdValue = (dTerm * Kd);

    // Limit the PID values to maximum values
    if (KpValue > maxPTerm) KpValue = maxPTerm;
    else if (KpValue < -maxPTerm) KpValue = -maxPTerm;

    if (KiValue > maxITerm) KiValue = maxITerm;
    else if (KiValue < -maxITerm) KiValue = -maxITerm;

    if (KdValue > maxDTerm) KdValue = maxDTerm;
    else if (KdValue < -maxDTerm) KdValue = -maxDTerm;
	// Multiply each term by its constant, and add it all up
    float result = KpValue + KiValue + KdValue;

	// Limit PID value to maximum values
	if (result > maxPID) result = maxPID;
	else if (result < -maxPID) result = -maxPID;
    // printf("Result: %f\n", result);
    // === LOG DATA HERE ===
    if (pidLogIndex < MAX_SAMPLES) {
        pidLogs[pidLogIndex].timestamp = thisTime;
        pidLogs[pidLogIndex].target = target;
        pidLogs[pidLogIndex].current = current;
        pidLogs[pidLogIndex].error = error;
        pidLogs[pidLogIndex].dT = dT;
        pidLogs[pidLogIndex].iTerm = iTerm;
        pidLogs[pidLogIndex].dTerm = dTerm;
        pidLogs[pidLogIndex].KpValue = KpValue;
        pidLogs[pidLogIndex].KiValue = KiValue;
        pidLogs[pidLogIndex].KdValue = KdValue;
        pidLogs[pidLogIndex].output = result;
        pidLogIndex++;
    }

	return result;
}


void dump_pid_logs() {
    // Print PID tunning values
    printf("Kp: %.4f\n", Kp);
    printf("Ki: %.4f\n", Ki);
    printf("Kd: %.4f\n", Kd);
    printf("timestamp,target,current,error,dT,iTerm,dTerm,KpValue,KiValue,KdValue,output\n");
    for (int i = 0; i < pidLogIndex; i++) {
        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            pidLogs[i].timestamp,
            pidLogs[i].target,
            pidLogs[i].current,
            pidLogs[i].error,
            pidLogs[i].dT,
            pidLogs[i].iTerm,
            pidLogs[i].dTerm,
            pidLogs[i].KpValue,
            pidLogs[i].KiValue,
            pidLogs[i].KdValue,
            pidLogs[i].output
        );
        // Add a small delay to avoid flooding the console
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("Total logs: %d\n", pidLogIndex);
}


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
    mpu.setXAccelOffset(-2737);
    mpu.setYAccelOffset(-1444);
    mpu.setZAccelOffset(1205);
    mpu.setXGyroOffset(174);
    mpu.setYGyroOffset(32);
    mpu.setZGyroOffset(-101);

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
    printf("Motors initialized\n");
    vTaskDelay(2000);
    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;
    int16_t speed = 0;
    int16_t max_rpm_speed = 500;

    while (true) {
        if (running) {
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
                //printf("ypr: %f,\t %f,\t %f\n", yaw, pitch, roll);
                // printf("roll: %f\n", roll);
                // Cast float to int16_t
                speed = (int16_t)pid(target, roll);
                // Limit speed to a maximum value
                if (speed > max_rpm_speed) speed = max_rpm_speed;
                else if (speed < -max_rpm_speed) speed = -max_rpm_speed;

                // Update motor speed
                steppers_set_speed(speed);
            }
        } else {
            steppers_set_speed(0); // Stop motors if not running
        }
        vTaskDelay(pdMS_TO_TICKS(2));  // Short delay to settle
    }
}

void process_line(const char *line) {
    // Check for "show" command
    if (strcmp(line, "s") == 0) {
        printf("\n--- Current PID values ---\n");
        printf("Kp: %.4f\n", Kp);
        printf("Ki: %.4f\n", Ki);
        printf("Kd: %.4f\n", Kd);
        printf("--------------------------\n");
        return;
    }

    // Check for "reset" command
    if (strcmp(line, "r") == 0) {
        Kp = 0.0f;
        Ki = 0.0f;
        Kd = 0.0f;
        printf("PID values reset to zero.\n");
        return;
    }

    // Check for "dump" command
    if (strcmp(line, "d") == 0) {
        // 500 ms delay
        vTaskDelay(pdMS_TO_TICKS(500));
        dump_pid_logs();
        return;
    }

    // Check for "clear" command
    if (strcmp(line, "c") == 0) {
        clear_pid_logs();
        printf("PID logs cleared.\n");
        return;
    }

    // Otherwise, assume it's a "P:", "I:", or "D:" command
    char param;
    float value;

    if (sscanf(line, "%c: %f", &param, &value) == 2) {
        switch (param) {
            case 'T': // Set target
                target = value;
                printf("Target updated to %.4f\n", target);
                break;
            case 'P':
                Kp = value;
                printf("Kp updated to %.4f\n", Kp);
                break;
            case 'I':
                Ki = value;
                printf("Ki updated to %.4f\n", Ki);
                break;
            case 'D':
                Kd = value;
                printf("Kd updated to %.4f\n", Kd);
                break;
            default:
                printf("Unknown parameter: %c\n", param);
                break;
        }
    } else {
        printf("Invalid command format: %s\n", line);
    }
}

void serial_scan(__unused void *params) {
    // Scan serial inputs. When 0 turn of leds, when 1 turn on led
    printf("Serial Scan Task is starting!!!\n");

    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART
    vTaskDelay(1000);
    char line[64];
    int pos = 0;
    while (true) {
        // Only read when data is available
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);

            // Echo the character back to the UART
            printf("%c", c);

            if (c == '\n' || c == '\r') {
                line[pos] = '\0';
                printf("Received: %s\n", line);
                process_line(line);
                pos = 0;
            } else if (pos < sizeof(line) - 1) {
                line[pos++] = c;
            }
        }
        // printf("Hello, UART! %d\n", pos);
        vTaskDelay(50);
    }
}


uint freq_to_wrap(uint frequency, float clk_div) {
    uint sys_clk = 125000000; // 125 MHz default for RP2040
    uint wrap = (uint)((sys_clk / clk_div) / frequency) - 1;
    // Ensure wrap is within valid range, if not, print an error
    // and stop the program
    if (wrap > 65535) {
        printf("Error: Frequency too low, wrap exceeds 16-bit limit.\n");
        wrap = 65535; // Cap at maximum wrap value
    }
    // printf("Wrap value for frequency %d Hz: %d\n", frequency, wrap);
    return wrap;
}

void set_pwm_step_frequency(uint slice, uint channel, uint frequency) {
    uint wrap = freq_to_wrap(frequency, PWM_DIVIDER);
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, channel, wrap / 2);  // 50% duty cycle
    pwm_set_enabled(slice, true);
}


void set_motor_speed(float rpm, uint slice, uint channel) {
    // Convert RPM to frequency
    uint steps_per_rev = STEPS_PER_REV * MICROSTEPPING;
    float revs_per_sec = rpm / 60.0f;
    uint step_freq = (uint)(steps_per_rev * revs_per_sec);  // Steps per second

    // Set the PWM frequency for the left motor step pin
    set_pwm_step_frequency(slice, channel, step_freq);
}


void stepper_task(__unused void *params) {
    // Configure the DIR and enable pins as outputs and set initial states
    gpio_init(LEFT_MOTOR_DIR_PIN);
    gpio_set_dir(LEFT_MOTOR_DIR_PIN, GPIO_OUT);
    gpio_init(LEFT_MOTOR_ENABLE_PIN);
    gpio_set_dir(LEFT_MOTOR_ENABLE_PIN, GPIO_OUT);
    gpio_put(LEFT_MOTOR_ENABLE_PIN, 0);  // Enable the motor
    gpio_put(LEFT_MOTOR_DIR_PIN, 0);     // Set direction

    // Configure Right motor pins similarly and disable motor
    gpio_init(RIGHT_MOTOR_DIR_PIN);
    gpio_set_dir(RIGHT_MOTOR_DIR_PIN, GPIO_OUT);
    gpio_init(RIGHT_MOTOR_ENABLE_PIN);
    gpio_set_dir(RIGHT_MOTOR_ENABLE_PIN, GPIO_OUT);
    gpio_put(RIGHT_MOTOR_ENABLE_PIN, 0);  // Enable the motor
    gpio_put(RIGHT_MOTOR_DIR_PIN, 0);     // Set direction

    // Initialize the stepper motor pins
    gpio_set_function(LEFT_MOTOR_STEP_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_MOTOR_STEP_PIN, GPIO_FUNC_PWM);

    // Get Slice and Channel for the left motor step pin
    uint left_motor_slice = pwm_gpio_to_slice_num(LEFT_MOTOR_STEP_PIN);
    uint left_motor_channel = pwm_gpio_to_channel(LEFT_MOTOR_STEP_PIN);
    uint right_motor_slice = pwm_gpio_to_slice_num(RIGHT_MOTOR_STEP_PIN);
    uint right_motor_channel = pwm_gpio_to_channel(RIGHT_MOTOR_STEP_PIN);

    // Divide the clock for the PWM slice, so that we can use a slower PWM
    // frequency for the stepper motor
    pwm_set_clkdiv(left_motor_slice, PWM_DIVIDER);  // Set slower PWM clock
    pwm_set_clkdiv(right_motor_slice, PWM_DIVIDER);  // Set slower PWM clock

    while (true) {
        set_motor_speed(5.0, left_motor_slice, left_motor_channel);  // Set left motor speed to 5.0 RPM
        set_motor_speed(10.0, right_motor_slice, right_motor_channel);  // Set right motor speed to 5.0 RPM
        vTaskDelay(pdMS_TO_TICKS(1000));  // idle or monitor here
        set_motor_speed(100.0, left_motor_slice, left_motor_channel);  // Set left motor speed to 100.0 RPM
        set_motor_speed(200.0, right_motor_slice, right_motor_channel);  // Set right motor speed to 100.0 RPM
        vTaskDelay(pdMS_TO_TICKS(1000));  // idle or monitor here
    }
}

void main_task(__unused void *params) {
    // start the led blinking
    // xTaskCreate(blink_task, "BlinkThread", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
    // xTaskCreate(robot_task, "JuanThread", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
    //xTaskCreate(motor_task, "MotorTask", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
    // xTaskCreate(serial_scan, "SerialScan", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
    // Initialize the LED pin
    // xTaskCreate(stepper_motor_task, "StepperMotorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(stepper_task, "stepper_task", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
    // gpio_init(LED1_PIN);
    // gpio_set_dir(LED1_PIN, GPIO_OUT);
    // int count = 0;
    // Initialize Button
    gpio_init(BUTTON_A);
    // Enable pull-up resistor, and watch for the button to be pressed
    gpio_pull_up(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    // Create queue for button events
    // button_event_queue = xQueueCreate(10, sizeof(uint32_t));

    // Register the IRQ
    // gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // Create the button handling task
    // xTaskCreate(button_task, "Button Task", 1024, NULL, 2, NULL);

    while(true) {
        // Toggle the LED state
        //led_state = !led_state;
        //gpio_put(LED1_PIN, led_state);
        // wait
        //vTaskDelay(500);
        vTaskDelay(20); // Delay for 20 ms
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
