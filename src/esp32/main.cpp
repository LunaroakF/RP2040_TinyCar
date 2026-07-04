// ============================================================
//  ESP32 轻量转发固件
//  职责单一：从 RP2040 (SPI Slave) 收数据 -> 原样通过 UDP 转发给 Mac
//  不再运行 WebServer，不再存储/合并障碍物、轨迹，不再拼 JSON
//  这样 CPU 占用和功耗都会明显下降
// ============================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/spi_slave.h"
#include <cstring>

// ---- Wi-Fi 路由器配置 ----
const char* WIFI_SSID = "fox";
const char* WIFI_PASS = "12345678";

// ---- 静态 IP 配置（ESP32 自己的地址）----
IPAddress local_IP(192, 168, 199, 25);
IPAddress gateway(192, 168, 199, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 199, 1);

// ---- Mac 接收端地址（你的电脑 IP + 监听端口）----
IPAddress MAC_IP(192, 168, 199, 231);
const uint16_t MAC_PORT = 5005;   // 要和 Mac App 里设置的端口一致

WiFiUDP udp;

// ---- SPI Slave 引脚（接 RP2040）----
#define ECHIP_MOSI 13
#define ECHIP_MISO 12
#define ECHIP_SCLK 14
#define ECHIP_CS   27
#define SPI_HOST_USED HSPI_HOST
#define FRAME_SIZE 64

// ---- SPI Slave 接收任务：把整帧里解析出来的有效行，攒成一个包一次性 UDP 发出 ----
void spiSlaveTask(void *pv) {
    static WORD_ALIGNED_ATTR uint8_t rxbuf[FRAME_SIZE];
    static WORD_ALIGNED_ATTR uint8_t txbuf[FRAME_SIZE];
    memset(txbuf, 0, FRAME_SIZE);

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = FRAME_SIZE * 8;
    t.tx_buffer = txbuf;
    t.rx_buffer = rxbuf;

    static String lineBuf;   // 跨帧拼接不完整的行
    static String outBatch;  // 本次要发送给 Mac 的内容

    while (true) {
        memset(rxbuf, 0, FRAME_SIZE);
        esp_err_t ret = spi_slave_transmit(SPI_HOST_USED, &t, portMAX_DELAY);
        if (ret == ESP_OK) {
            uint8_t len = rxbuf[0];
            if (len > 0 && len < FRAME_SIZE) {
                outBatch = "";
                for (int i = 0; i < len; i++) {
                    char c = (char)rxbuf[1 + i];
                    if (c == '\n') {
                        // 只转发以 # 或 @ 开头的有效行（障碍物 / 小车位置）
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
                // 一帧攒好之后一次性发送，而不是每行发一次 UDP 包，进一步省电
                if (outBatch.length() > 0) {
                    udp.beginPacket(MAC_IP, MAC_PORT);
                    udp.write((const uint8_t*)outBatch.c_str(), outBatch.length());
                    udp.endPacket();
                }
            }
        }
    }
}

void setup() {
    Serial.begin(115200);

    // ---- 初始化 SPI Slave ----
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = ECHIP_MOSI;
    buscfg.miso_io_num = ECHIP_MISO;
    buscfg.sclk_io_num = ECHIP_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.mode = 0;
    slvcfg.spics_io_num = ECHIP_CS;
    slvcfg.queue_size = 3;
    slvcfg.flags = 0;

    esp_err_t ret = spi_slave_initialize(SPI_HOST_USED, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("SPI slave init failed: %d\n", ret);
    }

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

    // 省电：不需要一直保持高功率射频待命去响应网页请求了
    WiFi.setSleep(true);

    udp.begin(0); // 只作为发送端，本地端口随意

    xTaskCreatePinnedToCore(spiSlaveTask, "spiSlave", 4096, NULL, 1, NULL, 1);

    Serial.print("UDP forwarder ready -> ");
    Serial.print(MAC_IP);
    Serial.print(":");
    Serial.println(MAC_PORT);
}

void loop() {
    // 主循环几乎空闲，不再处理 HTTP，可以放心 delay
    delay(1000);
}
