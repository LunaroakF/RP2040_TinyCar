#include <Arduino.h>

class WheelEncoder;
WheelEncoder* encoderInstance = nullptr;

class WheelEncoder {
private:
    int pin;
    volatile long pulseCount = 0;
    volatile unsigned long lastPulseTime = 0;
    unsigned long debounceMicros; // 消抖时间,单位微秒

public:
    WheelEncoder(int encoderPin, unsigned long debounceUs = 2000)
        : pin(encoderPin), debounceMicros(debounceUs) {
        encoderInstance = this;
    }
    // 初始化编码器
    void begin() {
        pinMode(pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pin), isr, FALLING);
    }
    // 获取脉冲计数
    long getCount() {
        noInterrupts();
        long c = pulseCount;
        interrupts();
        return c;
    }
    // 重置脉冲计数
    void reset() {
        noInterrupts();
        pulseCount = 0;
        interrupts();
    }
    // 中断服务程序
    static void isr() {
        if (encoderInstance) {
            unsigned long now = micros();
            // 处理micros()溢出(约70分钟一次),用差值而非直接比较更安全
            if (now - encoderInstance->lastPulseTime > encoderInstance->debounceMicros) {
                encoderInstance->pulseCount++;
                encoderInstance->lastPulseTime = now;
            }
        }
    }
};