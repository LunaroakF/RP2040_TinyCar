// SG90_PWM servo(15);  // GPIO15
// servo.writeAngle(180);

#pragma once
#include <Arduino.h>
#include <hardware/pwm.h>
#include <hardware/gpio.h>

class SG90_PWM {
private:
    int pin;
    uint slice_num;
    uint channel;

public:
    SG90_PWM(int gpioPin) {
        pin = gpioPin;
    }

    void begin() {
        gpio_set_function(pin, GPIO_FUNC_PWM);
        slice_num = pwm_gpio_to_slice_num(pin);
        channel   = pwm_gpio_to_channel(pin);
        pwm_config cfg = pwm_get_default_config();
        // 1MHz计数
        pwm_config_set_clkdiv(&cfg, 125.0f);
        // 20ms周期
        pwm_config_set_wrap(&cfg, 20000);
        pwm_init(slice_num, &cfg, true);
    }

    void writeAngle(int angle) {
        angle = constrain(angle, 0, 180);
        uint16_t pulse = 500 + (angle * 2000) / 180;
        pwm_set_chan_level(slice_num, channel, pulse);
    }
};