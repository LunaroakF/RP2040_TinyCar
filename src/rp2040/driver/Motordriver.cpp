#include "Motordriver.h"

MotorDriver::MotorDriver(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                          uint32_t pwmFreq)
    : _in1(in1), _in2(in2), _in3(in3), _in4(in4), _pwmFreq(pwmFreq) {}

void MotorDriver::begin() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_in3, OUTPUT);
    pinMode(_in4, OUTPUT);

    // arduino-pico core: 全局设置 PWM 频率，避免可闻噪音
    analogWriteFreq(_pwmFreq);
    // 默认分辨率是 8bit (0~255)，如需更细可以 analogWriteResolution(x)

    stop();
}

void MotorDriver::driveOneMotor(uint8_t pinFwd, uint8_t pinBwd, int speed) {
    speed = constrain(speed, -100, 100);

    if (speed == 0) {
        analogWrite(pinFwd, 0);
        analogWrite(pinBwd, 0);
        return;
    }

    int pwmVal = map(abs(speed), 0, 100, 0, PWM_MAX);

    if (speed > 0) {
        analogWrite(pinFwd, pwmVal);
        analogWrite(pinBwd, 0);
    } else {
        analogWrite(pinFwd, 0);
        analogWrite(pinBwd, pwmVal);
    }
}

void MotorDriver::setLeftSpeed(int speed) {
    driveOneMotor(_in1, _in2, speed);
}

void MotorDriver::setRightSpeed(int speed) {
    driveOneMotor(_in3, _in4, speed);
}

void MotorDriver::setSpeed(int leftSpeed, int rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
}

void MotorDriver::stop() {
    analogWrite(_in1, 0);
    analogWrite(_in2, 0);
    analogWrite(_in3, 0);
    analogWrite(_in4, 0);
}

void MotorDriver::brake() {
    // 两脚同时拉满，形成抱死刹车（注意：会有短暂较大电流，L298N 一般能承受）
    analogWrite(_in1, PWM_MAX);
    analogWrite(_in2, PWM_MAX);
    analogWrite(_in3, PWM_MAX);
    analogWrite(_in4, PWM_MAX);
}