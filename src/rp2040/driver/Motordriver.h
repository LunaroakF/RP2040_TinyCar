#pragma once
#include <Arduino.h>

// L298N 双路电机驱动（独立 ENA/ENB 做 PWM 调速，IN1~IN4 只做方向控制）
// 飞线说明：ENA 接 GPIO18，ENB 接 GPIO19（可在 GPIO18~28 范围内任选两根，
//           构造函数里传对应引脚号即可，不需要改代码逻辑）
// speed 范围: -100 ~ 100
//   正数 = 正转（假设方向，如实际相反可在 setLeftSpeed/setRightSpeed 里交换 IN1/IN2 逻辑或改接线）
//   负数 = 反转
//   0    = 停转（IN1/IN2 或 IN3/IN4 同时拉低，EN 脚 PWM 输出 0，自由停转）

class MotorDriver {
public:
    // in1/in2 控制左轮方向，in3/in4 控制右轮方向
    // enA 控制左轮转速（PWM），enB 控制右轮转速（PWM）
    MotorDriver(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                uint8_t enA, uint8_t enB,
                uint32_t pwmFreq = 20000);

    void begin();

    void setLeftSpeed(int speed);   // -100 ~ 100
    void setRightSpeed(int speed);  // -100 ~ 100

    void setSpeed(int leftSpeed, int rightSpeed); // 同时设置两轮

    void stop();                    // 两轮都停（自由停转，EN 拉低）
    void brake();                   // 两轮都刹车（抱死：IN 同时拉高 + EN 拉满）

private:
    uint8_t _in1, _in2, _in3, _in4;
    uint8_t _enA, _enB;
    uint32_t _pwmFreq;

    static constexpr int PWM_MAX = 255; // arduino-pico 默认 8bit 分辨率

    // pinFwd/pinBwd: 方向控制脚（数字电平）；enPin: 调速用 PWM 脚
    void driveOneMotor(uint8_t pinFwd, uint8_t pinBwd, uint8_t enPin, int speed);
};