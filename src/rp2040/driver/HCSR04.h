// HCSR04 sensor(2, 3); // Trig=GPIO2, Echo=GPIO3
// float d = sensor.readDistanceCM();

#pragma once
#include <Arduino.h>

class HCSR04 {
private:
    int trigPin;
    int echoPin;

    const float SOUND_SPEED = 0.0343f; // cm/us（声速）

    unsigned long timeout = 30000; // 30ms超时 ≈ 5m

public:
    HCSR04(int trig, int echo) {
        trigPin = trig;
        echoPin = echo;
    }

    void begin() {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);

        digitalWrite(trigPin, LOW);
    }

    // 返回距离（厘米）
    float readDistanceCM() {
        // 1. 触发10us脉冲
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);

        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        // 2. 读取回波时间
        unsigned long duration = pulseIn(echoPin, HIGH, timeout);

        // 3. 超时判断
        if (duration == 0) {
            return -1.0; // 无效距离
        }

        // 4. 计算距离（声波往返）
        float distance = (duration * SOUND_SPEED) / 2.0f;

        return distance;
    }
};