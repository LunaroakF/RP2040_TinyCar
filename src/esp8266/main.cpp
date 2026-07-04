// ============================================================
//  ESP8266 轻量转发固件
//  职责单一：从 RP2040 (SPI Slave) 收数据 -> 原样通过 UDP 转发给 Mac
//  不运行 WebServer，不存储/合并障碍物、轨迹，不拼 JSON
// ============================================================

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPISlave.h>   // ESP8266 Arduino核心内置库，无需额外安装
#include <cstring>

// ---- Wi-Fi 路由器配置 ----
const char* WIFI_SSID = "fox";
const char* WIFI_PASS = "12345678";

// ---- 静态 IP 配置（ESP8266 自己的地址）----
IPAddress local_IP(192, 168, 199, 25);
IPAddress gateway(192, 168, 199, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 199, 1);

// ---- Mac 接收端地址（你的电脑 IP + 监听端口）----
IPAddress MAC_IP(192, 168, 199, 231);
const uint16_t MAC_PORT = 5005;   // 要和 Mac App 里设置的端口一致

WiFiUDP udp;

// ---- SPI Slave 引脚 ----
// ESP8266硬件HSPI引脚是固定的，不能像ESP32那样自定义：
// MOSI = GPIO13, MISO = GPIO12, SCLK = GPIO14, CS = GPIO15
// 对应你RP2040端的接线注释: ECHIP_MOSI->IO13, ECHIP_MISO->IO12, ECHIP_SCLK->IO14, ECHIP_CS->IO15
//
// !!! 硬件注意 !!!
// GPIO15是ESP8266的启动strapping引脚，开机瞬间必须是低电平才能正常启动。
// 请在GPIO15上加一个10kΩ下拉电阻到地(飞线即可)，否则RP2040的CS信号在ESP8266上电瞬间
// 如果是高电平，会导致ESP8266无法正常开机。

// ESP8266硬件SPI Slave每次收发严格是32字节(数据不足硬件自动补0)。
// 帧格式: 第0字节=有效数据长度(1~31), 后面跟有效数据。
// RP2040端 espLink.h 记得定义 #define ESPLINK_TARGET_ESP8266 使FRAME_SIZE同步改成32。
#define FRAME_SIZE 32

// ---- 收数据用的环形缓冲区 ----
// ESP8266没有独立任务/双核，SPI数据是在中断上下文里到达的(onSpiData回调)。
// 中断里只做最轻量的工作：把收到的有效字节搬进环形缓冲区。
// 真正的按行解析 + UDP发送放到 loop() 里做，避免中断执行时间过长引发看门狗复位。
#define RING_BUF_SIZE 512
volatile uint8_t ringBuf[RING_BUF_SIZE];
volatile size_t ringHead = 0; // 中断(生产者)写
volatile size_t ringTail = 0; // loop(消费者)读

static uint8_t spiTxBuf[FRAME_SIZE]; // 回给RP2040的数据，这里用不到，全0即可

inline void ringPush(uint8_t b) {
    size_t next = (ringHead + 1) % RING_BUF_SIZE;
    if (next == ringTail) return; // 缓冲区满，丢弃(防止阻塞中断)
    ringBuf[ringHead] = b;
    ringHead = next;
}

// SPI Slave收到一帧数据时触发 (中断上下文, len固定是32, 硬件自动补0)
void onSpiData(uint8_t *data, size_t len) {
    uint8_t plen = data[0];
    if (plen > 0 && plen < FRAME_SIZE) {
        for (uint8_t i = 0; i < plen; i++) {
            ringPush(data[1 + i]);
        }
    }
    SPISlave.setData(spiTxBuf, FRAME_SIZE);
}

// 在loop()里把环形缓冲区中的字节按行解析，只转发以 # 或 @ 开头的有效行，攒成一批一次性UDP发出
void processIncomingSpiBytes() {
    static String lineBuf;
    if (ringTail == ringHead) return;

    String outBatch;
    while (ringTail != ringHead) {
        char c = (char)ringBuf[ringTail];
        ringTail = (ringTail + 1) % RING_BUF_SIZE;

        if (c == '\n') {
            if (lineBuf.length() >= 2 &&
                (lineBuf.charAt(0) == '#' || lineBuf.charAt(0) == '@')) {
                outBatch += lineBuf;
                outBatch += '\n';
            }
            lineBuf = "";
        } else if (c != '\r') {
            lineBuf += c;
            if (lineBuf.length() > 200) lineBuf = ""; // 异常保护
        }
    }

    if (outBatch.length() > 0) {
        udp.beginPacket(MAC_IP, MAC_PORT);
        udp.write((const uint8_t*)outBatch.c_str(), outBatch.length());
        udp.endPacket();
    }
}

void setup() {
    Serial.begin(115200);

    // ---- 初始化SPI Slave ----
    memset(spiTxBuf, 0, FRAME_SIZE);
    SPISlave.onData(onSpiData);
    SPISlave.begin();
    // 提升MISO在下降沿更新数据的稳定性 (社区常见的可靠性修复)
    SPI1C2 |= (1 << SPIC2MISODM_S);
    SPISlave.setData(spiTxBuf, FRAME_SIZE);

    // ---- 连接 Wi-Fi ----
    WiFi.mode(WIFI_STA);
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)) {
        Serial.println("STA Configuration Failed!");
    }
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // 省电：ESP8266用 setSleepMode 而不是ESP32的 setSleep(bool)
    WiFi.setSleepMode(WIFI_MODEM_SLEEP);

    udp.begin(0); // 只作为发送端，本地端口随意

    Serial.print("UDP forwarder ready -> ");
    Serial.print(MAC_IP);
    Serial.print(":");
    Serial.println(MAC_PORT);
}

void loop() {
    processIncomingSpiBytes(); // 取代原来ESP32的独立SPI任务
    // 不需要delay，尽快把环形缓冲区里的数据倒出来，减少丢帧风险
}