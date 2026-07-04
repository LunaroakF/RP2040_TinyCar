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

    void begin() {
        pinMode(pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pin), isr, FALLING);
    }

    long getCount() {
        noInterrupts();
        long c = pulseCount;
        interrupts();
        return c;
    }

    void reset() {
        noInterrupts();
        pulseCount = 0;
        interrupts();
    }

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