#pragma once
#include <Arduino.h>

// L298N 双路电机驱动（无独立 ENA/ENB，直接用 IN 引脚做 PWM 调速）
// speed 范围: -100 ~ 100
//   正数 = 正转（假设方向，如实际相反可在 setLeftSpeed/setRightSpeed 里交换 IN1/IN2 逻辑或改接线）
//   负数 = 反转
//   0    = 刹车（两脚同时拉低，自由停转；如需抱死刹车可自行改成两脚同时拉高）

class MotorDriver {
public:
    // in1/in2 控制左轮，in3/in4 控制右轮
    MotorDriver(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                uint32_t pwmFreq = 20000);

    void begin();

    void setLeftSpeed(int speed);   // -100 ~ 100
    void setRightSpeed(int speed);  // -100 ~ 100

    void setSpeed(int leftSpeed, int rightSpeed); // 同时设置两轮

    void stop();                    // 两轮都停（自由停转）
    void brake();                   // 两轮都刹车（抱死，两脚同时高电平）

private:
    uint8_t _in1, _in2, _in3, _in4;
    uint32_t _pwmFreq;

    static constexpr int PWM_MAX = 255; // arduino-pico 默认 8bit 分辨率

    void driveOneMotor(uint8_t pinFwd, uint8_t pinBwd, int speed);
};