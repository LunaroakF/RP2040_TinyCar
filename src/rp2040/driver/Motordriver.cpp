#include "Motordriver.h"

MotorDriver::MotorDriver(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                          uint8_t enA, uint8_t enB,
                          uint32_t pwmFreq)
    : _in1(in1), _in2(in2), _in3(in3), _in4(in4),
      _enA(enA), _enB(enB), _pwmFreq(pwmFreq) {}

void MotorDriver::begin() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_in3, OUTPUT);
    pinMode(_in4, OUTPUT);
    pinMode(_enA, OUTPUT);
    pinMode(_enB, OUTPUT);

    // arduino-pico core: 全局设置 PWM 频率，避免可闻噪音
    analogWriteFreq(_pwmFreq);
    // 默认分辨率是 8bit (0~255)，如需更细可以 analogWriteResolution(x)

    stop();
}

void MotorDriver::driveOneMotor(uint8_t pinFwd, uint8_t pinBwd, uint8_t enPin, int speed) {
    speed = constrain(speed, -100, 100);

    if (speed == 0) {
        digitalWrite(pinFwd, LOW);
        digitalWrite(pinBwd, LOW);
        analogWrite(enPin, 0);
        return;
    }

    int pwmVal = map(abs(speed), 0, 100, 0, PWM_MAX);

    if (speed > 0) {
        digitalWrite(pinFwd, HIGH);
        digitalWrite(pinBwd, LOW);
    } else {
        digitalWrite(pinFwd, LOW);
        digitalWrite(pinBwd, HIGH);
    }
    analogWrite(enPin, pwmVal);
}

void MotorDriver::setLeftSpeed(int speed) {
    driveOneMotor(_in1, _in2, _enA, speed);
}

void MotorDriver::setRightSpeed(int speed) {
    driveOneMotor(_in3, _in4, _enB, speed);
}

void MotorDriver::setSpeed(int leftSpeed, int rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
}

void MotorDriver::stop() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    digitalWrite(_in3, LOW);
    digitalWrite(_in4, LOW);
    analogWrite(_enA, 0);
    analogWrite(_enB, 0);
}

void MotorDriver::brake() {
    // IN 同时拉高 + EN 拉满，形成抱死刹车（注意：会有短暂较大电流，L298N 一般能承受）
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, HIGH);
    digitalWrite(_in3, HIGH);
    digitalWrite(_in4, HIGH);
    analogWrite(_enA, PWM_MAX);
    analogWrite(_enB, PWM_MAX);
}