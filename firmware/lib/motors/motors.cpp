#include "motors.h"


dualMotorController::dualMotorController(uint leftMotorPinA, uint leftMotorPinB, uint leftMotorPwmPin,
                                         uint rightMotorPinA, uint rightMotorPinB, uint rightMotorPwmPin)
    : leftMotorPinA(leftMotorPinA), leftMotorPinB(leftMotorPinB), leftMotorPwmPin(leftMotorPwmPin),
      rightMotorPinA(rightMotorPinA), rightMotorPinB(rightMotorPinB), rightMotorPwmPin(rightMotorPwmPin)
{
}


void dualMotorController::init(){
    // Initialize motor pins
    // Set the pin function to PWM
    gpio_set_function(leftMotorPwmPin, GPIO_FUNC_PWM);
    gpio_set_function(rightMotorPwmPin, GPIO_FUNC_PWM);

    // Get the slice number for the chosen pin
    sliceNumLeft = pwm_gpio_to_slice_num(leftMotorPwmPin);
    sliceNumRight = pwm_gpio_to_slice_num(rightMotorPwmPin);

    // Configure PWM
    pwm_config config = pwm_get_default_config();

    // Set PWM frequency to 10 kHz
    // sys_clk is 125 MHz by default, so:
    // 125 MHz / clkdiv / top = 10 kHz
    // Example: clkdiv = 1.0, top = 12500 --> 125MHz / 1 / 12500 = 10kHz
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, 12500);

    pwm_init(sliceNumLeft, &config, true);
    pwm_init(sliceNumRight, &config, true);

    // GPIO pin A and B are set to output
    gpio_init(leftMotorPinA);
    gpio_init(leftMotorPinB);
    gpio_set_dir(leftMotorPinA, GPIO_OUT);
    gpio_set_dir(leftMotorPinB, GPIO_OUT);

    gpio_init(rightMotorPinA);
    gpio_init(rightMotorPinB);
    gpio_set_dir(rightMotorPinA, GPIO_OUT);
    gpio_set_dir(rightMotorPinB, GPIO_OUT);
}


void dualMotorController::setMotorSpeed(uint pwmPin, uint pinA, uint pinB, int16_t speed) {
    // Ensure speed is within the range of -1000 to 1000
    if (speed < -1000)
        speed = -1000;
    else if (speed > 1000){
        speed = 1000;
    }

    // Set the direction of the motor, based on the speed
    // If speed is positive or zero, set pinA high and pinB low
    // If speed is negative, set pinA low and pinB high
    if (speed >= 0) {
        gpio_put(pinA, 1);
        gpio_put(pinB, 0);
    } else {
        gpio_put(pinA, 0);
        gpio_put(pinB, 1);
    }

    // Calculate the duty cycle level (which is absolute value of speed)
    uint level = (uint)(map(abs16(speed), 0, 1000, 0, 12500)); // 0 to 12500
                                                               // for 10kHz
                                                               // frequency

    // Set the PWM level
    pwm_set_gpio_level(pwmPin, level);
}

void dualMotorController::setLeftMotorSpeed(int16_t speed) {
    setMotorSpeed(leftMotorPwmPin, leftMotorPinA, leftMotorPinB, speed);
}


void dualMotorController::setRightMotorSpeed(int16_t speed) {
    setMotorSpeed(rightMotorPwmPin, rightMotorPinA, rightMotorPinB, speed);
}


void dualMotorController::setSpeed(int16_t leftSpeed, int16_t rightSpeed) {
    setLeftMotorSpeed(leftSpeed);
    setRightMotorSpeed(rightSpeed);
}
