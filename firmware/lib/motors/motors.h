#ifndef _MOTORS_H_
#define _MOTORS_H_


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"


class dualMotorController {
public:
    dualMotorController(uint leftMotorPinA, uint leftMotorPinB, uint leftMotorPwmPin,
			uint rightMotorPinA, uint rightMotorPinB, uint rightMotorPwmPin);
    void init();
    void setLeftMotorSpeed(int16_t speed);  // Speed range from -1000 to 1000
    void setRightMotorSpeed(int16_t speed); // Speed range from -1000 to 1000
    void setSpeed(int16_t leftSpeed, int16_t rightSpeed); // Speed range from
                                                          // -1000 to 1000

private:
    uint leftMotorPinA;
    uint leftMotorPinB;
    uint leftMotorPwmPin;
    uint rightMotorPinA;
    uint rightMotorPinB;
    uint rightMotorPwmPin;

    uint sliceNumLeft;
    uint sliceNumRight;

    void setMotorSpeed(uint pwmPin, uint pinA, uint pinB, int16_t speed);

    static int16_t abs16(int16_t x) {
        return x < 0 ? -x : x;
    }

    int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

#endif /* _MOTORS_H_ */
