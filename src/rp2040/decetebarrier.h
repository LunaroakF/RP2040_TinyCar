#include "driver/HCSR04.h"
#include "driver/ServoSG90.h"
#include "Arduino.h"
#include "config.h"

class DistanceDetector {
private:
    HCSR04 detector;
    SG90_PWM servo;
    int currentAngle = -1; // 记录当前角度，角度没变就不用重新等待

public:
    DistanceDetector(int trigPin, int echoPin, int servoPin)
        : detector(trigPin, echoPin), servo(servoPin) {
        detector.begin();
        servo.begin();
    }
    // 获取距离
    float getDistanceCM() {
        return detector.readDistanceCM();
    }
    // 只转动，不测距。用于提前把探头转到需要的方向（比如转向/倒车前先回正）
    void aimServo(int angle) {
        if (angle != currentAngle) {
            servo.writeAngle(angle);
            currentAngle = angle;
            delay(SERVO_SETTLE_TIME); // 等待舵机就位
        }
    }
    // 检测物体
    bool isObjectDetected(float thresholdCM, int angle) {
        aimServo(angle);
        float distance = getDistanceCM();
        return (distance > 0 && distance < thresholdCM);
    }
};