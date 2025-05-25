#include "steppers.h"
// #include "FreeRTOS.h"
// #include "queue.h"
// #include "task.h"

static int16_t speed_rpm = 0;

static inline int16_t abs16(int16_t x) {
    return x < 0 ? -x : x;
}


// Returns the delay in microseconds between each step
unsigned long get_step_delay_us(double target_rpm, int steps_per_rev) {
    if (target_rpm <= 0) return 0; // Avoid invalid input
    return (unsigned long)(60000000.0 / (target_rpm * steps_per_rev));
}

void steppers_set_speed(int16_t rpm) {
    speed_rpm = rpm;
}

void stepper_motor_task(__unused void *params) {
    // Set Stepper motor pins
    gpio_init(LEFT_MOTOR_STEP_PIN);
    gpio_set_dir(LEFT_MOTOR_STEP_PIN, GPIO_OUT);
    gpio_init(LEFT_MOTOR_DIR_PIN);
    gpio_set_dir(LEFT_MOTOR_DIR_PIN, GPIO_OUT);
    gpio_init(RIGHT_MOTOR_STEP_PIN);
    gpio_set_dir(RIGHT_MOTOR_STEP_PIN, GPIO_OUT);
    gpio_init(RIGHT_MOTOR_DIR_PIN);
    gpio_set_dir(RIGHT_MOTOR_DIR_PIN, GPIO_OUT);
    // Configure the enable pins as outputs
    gpio_init(LEFT_MOTOR_ENABLE_PIN);
    gpio_set_dir(LEFT_MOTOR_ENABLE_PIN, GPIO_OUT);
    gpio_init(RIGHT_MOTOR_ENABLE_PIN);
    gpio_set_dir(RIGHT_MOTOR_ENABLE_PIN, GPIO_OUT);

    // NUMBER_OF_STEPS * DIVIDER_FOR_FULL_ROTATION
    //    MS1   MS2   MS3     Microstep Resolution
    //    0     0     0       Full Step
    //    1     0     0       Half Step
    //    0     1     0       Quarter Step
    //    1     1     0       Eighth Step
    //    1     1     1       Sixteenth Step
    int number_of_steps = 200 * 16; // Number of steps to take
    bool pin_state = false; // Pin state

    // Disable the motors initially
    gpio_put(LEFT_MOTOR_ENABLE_PIN, 1);
    gpio_put(RIGHT_MOTOR_ENABLE_PIN, 1);
    while (true) {
        // If speed is beetween -10 and 10, stop the motor
        // if (abs16(speed_rpm) < 5) {
        // if (speed_rpm == 0) {
        //     gpio_put(LEFT_MOTOR_ENABLE_PIN, 1);
        //     gpio_put(RIGHT_MOTOR_ENABLE_PIN, 1);
        // } else {
        //     // Enable the motor
        //     gpio_put(LEFT_MOTOR_ENABLE_PIN, 0);
        //     gpio_put(RIGHT_MOTOR_ENABLE_PIN, 0);
        // }
        // Set the direction of the motor according to the speed_rpm
        if (speed_rpm < 0) {
            gpio_put(LEFT_MOTOR_DIR_PIN, 1);
            gpio_put(RIGHT_MOTOR_DIR_PIN, 1);
        } else {
            gpio_put(LEFT_MOTOR_DIR_PIN, 0);
            gpio_put(RIGHT_MOTOR_DIR_PIN, 0);
        }
        // Move the stepper motor one step
        gpio_put(LEFT_MOTOR_STEP_PIN, pin_state);
        gpio_put(RIGHT_MOTOR_STEP_PIN, pin_state);
        sleep_us(get_step_delay_us(abs16(speed_rpm), number_of_steps));
        pin_state = !pin_state;
    }
}

// void steppers_start(void) {
//     xTaskCreate(stepper_motor_task, "StepperMotorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
// }


void disable_steppers(void) {
    // Disable the motors
    gpio_put(LEFT_MOTOR_ENABLE_PIN, 1);
    gpio_put(RIGHT_MOTOR_ENABLE_PIN, 1);
}


void enable_steppers(void) {
    // Enable the motors
    gpio_put(LEFT_MOTOR_ENABLE_PIN, 0);
    gpio_put(RIGHT_MOTOR_ENABLE_PIN, 0);
}
