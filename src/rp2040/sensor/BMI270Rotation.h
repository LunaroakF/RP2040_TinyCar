/*
 * BMI270Rotation.h
 * ------------------------------------------------------------
 * 把 BMI270 的姿态解算丢到 RP2040 的 Core1 上后台跑，
 * Core0（你的 loop()）随时 getRotation() 就能拿到最新角度，
 * 不用管采样、滤波这些细节，也不会卡主循环。
 *
 * 用法：
 *   BMI270Rotation imuRot;
 *
 *   void setup() {
 *       Serial.begin(115200);
 *       if (!imuRot.begin(4, 5)) {   // SDA=GP4, SCL=GP5
 *           Serial.println("IMU init failed");
 *           while (1) {}
 *       }
 *   }
 *
 *   void loop() {
 *       float x, y, z;
 *       imuRot.getRotation(x, y, z);  // deg，相对于 begin() 时刻的姿态
 *       ...
 *       // 不想用了就释放 core1：
 *       // imuRot.end();
 *   }
 *
 * 注意：
 *   - begin() 会调用 multicore_launch_core1() 接管 Core1，
 *     所以工程里不要再自己写 setup1()/loop1()，两边会打架。
 *   - I2C(Wire) 在 begin() 里配置一次后，后续读取全部发生在
 *     Core1 内部，Core0 不会再碰 Wire，所以不存在总线抢占问题。
 *   - Core1内部用硬件定时器(repeating_timer)以100Hz精确触发，
 *     中断服务程序只置标志位，真正的I2C读取放在core1_task()主
 *     循环里做，避免在中断上下文里操作I2C。
 *   - yaw（Z轴）只能靠陀螺仪积分，长时间运行仍会缓慢漂移，
 *     这是没有磁力计情况下的物理限制，不是bug。
 * ------------------------------------------------------------
 */

#ifndef BMI270_ROTATION_H
#define BMI270_ROTATION_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/time.h"

class BMI270Rotation {
public:
    BMI270Rotation();
    // 初始化：配置I2C -> 连接BMI270 -> 静止零偏标定(记录初始偏移角度) -> 启动Core1后台任务
    // sda_pin/scl_pin: I2C引脚
    // calib_samples:   标定采样点数，越多越准但初始化越慢，默认500
    // 返回 true 表示初始化成功并已开始后台运行
    bool begin(uint8_t sda_pin, uint8_t scl_pin, int calib_samples = 500);
    // 销毁：通知Core1后台任务退出，并调用 multicore_reset_core1() 把该核心释放/复位
    // 调用后 Core1 可以被重新 begin()，或挪去做别的事情
    void end();
    // 获取当前姿态角，单位：度(deg)，相对于 begin() 完成那一刻的姿态
    // x = roll(绕X轴), y = pitch(绕Y轴), z = yaw(绕Z轴)
    void getRotation(float &x, float &y, float &z);
    // 获取去零偏后的瞬时角速度，单位：deg/s
    void getGyro(float &gx, float &gy, float &gz);
    // 当前IMU是否判定为静止状态（可用于调试/上层逻辑）
    bool isStationary();
    // 调试用：获取去零偏后的加速度(单位g)，用来自己核对静止时读数到底是多少
    void getAccel(float &ax, float &ay, float &az);
    // 后台任务是否仍在Core1上运行
    bool isRunning() const { return running; }

private:
    static void core1_entry_trampoline();
    void core1_task();
    bool deepCalibrate(int samples);

    // 硬件定时器中断回调（100Hz），只负责置标志位，不做任何I2C操作
    static bool timer_callback_trampoline(struct repeating_timer *t);

    // core1_entry_trampoline 是静态函数（pico-sdk要求），
    // 用这个指针把调用转发回具体的类实例
    static BMI270Rotation* activeInstance;

    BMI270 imu;
    uint8_t sdaPin, sclPin;

    // 零偏（begin()中标定一次，之后只读）
    float ax_bias, ay_bias, az_bias;
    float gx_bias, gy_bias, gz_bias;

    // 姿态角(deg)，Core1写，Core0读，用mutex保护
    volatile float roll, pitch, yaw;
    volatile float lastGx, lastGy, lastGz;
    volatile float lastAx, lastAy, lastAz;
    volatile bool  stationaryFlag;

    // 用于ZUPT判定的低通平滑值 + 连续帧计数（去抖动，防止噪声导致的抖动误判）
    float smoothGyroMag, smoothAccelDev;
    int   stillStreak;
    int   moveStreak;

    mutex_t dataMutex;
    volatile bool running;
    volatile bool stopRequested;

    // 硬件定时器：挂在Core1上，中断只置sensorDataReady标志位，
    // 真正的I2C读取和解算放在core1_task()主循环里做（中断里做I2C不安全）
    struct repeating_timer timer;
    volatile bool sensorDataReady;

    static constexpr float DT_S       = 0.01f;                 // 100Hz
    static constexpr float RAD2DEG    = 57.29577951f;           // 180/pi
    static constexpr float ALPHA_COMP = 0.995f;                 // 互补滤波：陀螺仪权重

    // ---- 静止判定相关参数（阈值放宽 + 去抖动，可按需再调）----
    static constexpr float ZUPT_GYRO_DPS   = 1.0f;   // 平滑后角速度模长阈值(deg/s)
    static constexpr float ZUPT_ACCEL_DEV  = 0.03f;  // |加速度模长-1g| 阈值(g)
    static constexpr float ZUPT_SMOOTH_A   = 0.2f;   // 判定用的低通滤波系数
    static constexpr int   ZUPT_STILL_N    = 8;      // 连续8帧(~80ms)满足才判STATIC
    static constexpr int   ZUPT_MOVE_N     = 2;      // 连续2帧不满足就判MOVING（快速响应真实运动）
};

#endif // BMI270_ROTATION_H