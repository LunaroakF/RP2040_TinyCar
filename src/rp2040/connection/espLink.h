#ifndef ESP_LINK_H
#define ESP_LINK_H

#include <Arduino.h>

#ifdef ESPLINK_TARGET_ESP8266
#define ESPLINK_FRAME_SIZE   32   // 必须和ESP8266端 FRAME_SIZE 一致
#else
#define ESPLINK_FRAME_SIZE   64   // 必须和ESP32端 FRAME_SIZE 一致
#endif
#define ESPLINK_MAX_PAYLOAD  (ESPLINK_FRAME_SIZE - 1)

class ESPLink {
public:
    ESPLink(uint8_t mosiPin, uint8_t misoPin, uint8_t sclkPin, uint8_t csPin,
            uint32_t clockHz = 100000)
        : _mosi(mosiPin), _miso(misoPin), _sclk(sclkPin), _cs(csPin) {
        // 半周期微秒数，至少1us
        uint32_t halfPeriod = 500000UL / clockHz;
        _halfPeriodUs = halfPeriod < 1 ? 1 : halfPeriod;
    }
    void begin() {
        pinMode(_cs, OUTPUT);
        pinMode(_sclk, OUTPUT);
        pinMode(_mosi, OUTPUT);
        pinMode(_miso, INPUT);

        digitalWrite(_cs, HIGH);   // 空闲时CS拉高
        digitalWrite(_sclk, LOW);  // Mode0空闲电平为低
        digitalWrite(_mosi, LOW);
    }
    // 发送一条文本消息，超过63字节会自动分片成多帧发送
    void put(const String &msg) {
        put((const uint8_t*)msg.c_str(), msg.length());
    }
    void put(const char *msg) {
        put((const uint8_t*)msg, strlen(msg));
    }
    void put(const uint8_t *data, size_t len) {
        if (len == 0) {
            heartbeat();
            return;
        }
        size_t offset = 0;
        while (offset < len) {
            size_t chunk = min((size_t)ESPLINK_MAX_PAYLOAD, len - offset);
            sendFrame(data + offset, (uint8_t)chunk);
            offset += chunk;
        }
    }
    // 心跳帧(len=0)，接收端会忽略内容，只用来保活/占位，最终没有使用
    void heartbeat() {
        sendFrame(nullptr, 0);
    }
    // 和put()一样，但会在消息末尾自动加一个换行符，方便网页端按行显示
    void putln(const String &msg) {
        put(msg + "\n");
    }
    void putln(const char *msg) {
        String s(msg);
        s += "\n";
        put(s);
    }

private:
    uint8_t _mosi, _miso, _sclk, _cs;
    uint32_t _halfPeriodUs;

    // 软件模拟发送一个字节 (Mode 0, MSB first)，返回同时读到的MISO字节(本项目用不到，可忽略)
    uint8_t transferByte(uint8_t out) {
        uint8_t in = 0;
        for (int i = 7; i >= 0; i--) {
            digitalWrite(_mosi, (out >> i) & 0x01);
            delayMicroseconds(_halfPeriodUs);
            digitalWrite(_sclk, HIGH); // 上升沿，从机在此采样
            in = (in << 1) | digitalRead(_miso);
            delayMicroseconds(_halfPeriodUs);
            digitalWrite(_sclk, LOW);
        }
        return in;
    }

   void sendFrame(const uint8_t *data, uint8_t len) {
    uint8_t buf[ESPLINK_FRAME_SIZE];
    memset(buf, 0, ESPLINK_FRAME_SIZE);
    buf[0] = len;
    if (data && len > 0) {
        memcpy(&buf[1], data, len);
    }

    digitalWrite(_cs, LOW);
    delayMicroseconds(5);

    noInterrupts();               // 防止编码器等中断打乱bit-bang时序
    for (int i = 0; i < ESPLINK_FRAME_SIZE; i++) {
        transferByte(buf[i]);
    }
    interrupts();

    digitalWrite(_cs, HIGH);
    }
};

#endif // ESP_LINK_H