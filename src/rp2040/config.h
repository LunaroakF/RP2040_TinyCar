
// 引脚定义
// 板载指示灯
#define LED_PIN 17

// I2C
#define IMU_SDA_PIN   4
#define IMU_SCL_PIN   5

// 舵机和超声波
#define SERVO_PIN     15
#define ULTRASONIC_TRIG_PIN 2
#define ULTRASONIC_ECHO_PIN 3

// 轮子编码器
#define WHEEL_ENCODER_IN1 6
#define WHEEL_ENCODER_IN2 7
#define WHEEL_ENCODER_IN3 12
#define WHEEL_ENCODER_IN4 13

// 轮子编码器输出
#define LM_DO 14

// ESP8266通信
#define ECHIP_MOSI 8 // 连至ESP8266的 IO13 (MOSI) 或测试用ESP32的 IO13
#define ECHIP_MISO 11 // 连至ESP8266的 IO12 (MISO) 或测试用ESP32的 IO12
#define ECHIP_SCLK  10 // 连至ESP8266的 IO14 (SCLK) 或测试用ESP32的 IO14
#define ECHIP_CS   9 // 连至ESP8266的 IO15 (CS) 或测试用ESP32的 IO27

#define SPI_FRAME_SIZE 64
#define SPI_SEND_INTERVAL_MS 20

// 编码轮块数
#define WHEEL_BLOCK_COUNT 20
// 车轮直径
#define WHEEL_DIAMETER 6.2 // 单位: cm