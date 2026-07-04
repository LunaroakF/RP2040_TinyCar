#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ---- Wi-Fi 热点配置 ----
const char* AP_SSID = "RP2040-Monitor";
const char* AP_PASS = "12345678";

// ---- SPI Slave 引脚 (ESP8266 硬件SPI引脚，但这里用软件bit-bang) ----
// 对应RP2040端: ECHIP_MOSI=8, ECHIP_MISO=11, ECHIP_SCLK=10, ECHIP_CS=9
#define ESP_SPI_MOSI 13  // 接 RP2040 (RP2040的输出)
#define ESP_SPI_MISO 12  // 接 RP2040 (未使用，拉低)
#define ESP_SPI_SCLK 14  // 接 RP2040 SCLK
#define ESP_SPI_CS   15  // 接 RP2040 CS

#define FRAME_SIZE 64 // 必须与RP2040端一致

// !!! 重要: ESP8266跑WiFi时中断有几us的抖动，bit-bang方式在高时钟频率下容易丢帧。
// 建议RP2040端SPI时钟降到 <=100kHz，越慢越稳。

ESP8266WebServer server(80);

String logBuffer;
const size_t MAX_LOG_SIZE = 6000;

// ---- 中断中使用的裸数据，不能用String/堆分配 ----
volatile uint8_t rx_raw[FRAME_SIZE];
volatile uint8_t g_bitCount = 0;
volatile uint8_t g_byteIndex = 0;
volatile uint8_t g_curByte = 0;
volatile bool g_csActive = false;
volatile bool g_frameReady = false;
volatile uint8_t g_frameLen = 0; // 帧结束时的字节数快照

void appendLog(const String &s) {
    logBuffer += s;
    if (logBuffer.length() > MAX_LOG_SIZE) {
        logBuffer = logBuffer.substring(logBuffer.length() - MAX_LOG_SIZE);
    }
}

// ---- SCLK 上升沿: 采样一个bit ----
void ICACHE_RAM_ATTR onSclkRise() {
    if (!g_csActive) return;
    if (g_byteIndex >= FRAME_SIZE) return; // 防止溢出，多余的bit丢弃

    uint8_t bit = digitalRead(ESP_SPI_MOSI);
    g_curByte = (g_curByte << 1) | bit;
    g_bitCount++;
    if (g_bitCount == 8) {
        rx_raw[g_byteIndex++] = g_curByte;
        g_curByte = 0;
        g_bitCount = 0;
    }
}

// ---- CS 变化: 帧开始/结束 ----
void ICACHE_RAM_ATTR onCsChange() {
    bool level = digitalRead(ESP_SPI_CS);
    if (level == LOW) {
        // 帧开始
        g_csActive = true;
        g_byteIndex = 0;
        g_bitCount = 0;
        g_curByte = 0;
    } else {
        // 帧结束
        g_csActive = false;
        if (g_byteIndex > 0) {
            g_frameLen = g_byteIndex;
            g_frameReady = true;
        }
    }
}

void processFrameIfReady() {
    if (!g_frameReady) return;

    // 短暂关中断，安全拷贝出来
    noInterrupts();
    uint8_t localLen = g_frameLen;
    uint8_t localBuf[FRAME_SIZE];
    memcpy(localBuf, (const void*)rx_raw, localLen);
    g_frameReady = false;
    interrupts();

    if (localLen < 1) return;
    uint8_t len = localBuf[0];
    if (len > 0 && len < FRAME_SIZE && (1 + len) <= localLen) {
        String s;
        s.reserve(len);
        for (int i = 0; i < len; i++) s += (char)localBuf[1 + i];
        appendLog(s);
    }
    // len == 0 是心跳帧，忽略
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<title>RP2040 消息监视</title>
<style>
  body { font-family: sans-serif; background:#1e1e1e; color:#eee; margin:20px; }
  h2 { color:#4fc3f7; }
  textarea {
    width: 100%; height: 70vh; background:#111; color:#0f0;
    font-family: monospace; font-size: 14px; border:1px solid #444;
    padding:8px; box-sizing:border-box; resize:none;
  }
</style>
</head>
<body>
  <h2>RP2040 串口消息 (SPI转发, ESP8266)</h2>
  <textarea id="log" readonly></textarea>
<script>
  const ta = document.getElementById('log');
  async function poll() {
    try {
      const res = await fetch('/log');
      const text = await res.text();
      const atBottom = (ta.scrollTop + ta.clientHeight >= ta.scrollHeight - 5);
      ta.value = text;
      if (atBottom) ta.scrollTop = ta.scrollHeight;
    } catch (e) {}
    setTimeout(poll, 500);
  }
  poll();
</script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }
void handleLog()  { server.send(200, "text/plain; charset=utf-8", logBuffer); }

void setup() {
    Serial.begin(115200);

    pinMode(ESP_SPI_MOSI, INPUT);
    pinMode(ESP_SPI_SCLK, INPUT);
    pinMode(ESP_SPI_CS, INPUT_PULLUP);
    // MISO本例不使用，拉低避免干扰主机线路
    pinMode(ESP_SPI_MISO, OUTPUT);
    digitalWrite(ESP_SPI_MISO, LOW);

    attachInterrupt(digitalPinToInterrupt(ESP_SPI_SCLK), onSclkRise, RISING);
    attachInterrupt(digitalPinToInterrupt(ESP_SPI_CS), onCsChange, CHANGE);

    // ---- 创建热点 ----
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/log", handleLog);
    server.begin();
    Serial.println("Web server started");
}

void loop() {
    server.handleClient();
    processFrameIfReady();
}