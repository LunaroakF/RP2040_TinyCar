#include "BMI270Rotation.h"

BMI270Rotation* BMI270Rotation::activeInstance = nullptr;

BMI270Rotation::BMI270Rotation()
    : sdaPin(0), sclPin(0),
      ax_bias(0), ay_bias(0), az_bias(0),
      gx_bias(0), gy_bias(0), gz_bias(0),
      roll(0), pitch(0), yaw(0),
      lastGx(0), lastGy(0), lastGz(0),
      lastAx(0), lastAy(0), lastAz(0),
      stationaryFlag(false),
      smoothGyroMag(0), smoothAccelDev(0),
      stillStreak(0), moveStreak(0),
      running(false), stopRequested(false),
      sensorDataReady(false)
{
    mutex_init(&dataMutex);
}

// ---------------------------------------------------------------
// 静止零偏标定：多点采样 + 方差剔除，跟你原来的deepCalibrate逻辑一致
// ---------------------------------------------------------------
bool BMI270Rotation::deepCalibrate(int samples) {
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sq_ax = 0, sq_ay = 0;

    for (int i = 0; i < samples; i++) {
        imu.getSensorData();

        sum_ax += imu.data.accelX; sq_ax += imu.data.accelX * imu.data.accelX;
        sum_ay += imu.data.accelY; sq_ay += imu.data.accelY * imu.data.accelY;
        sum_az += imu.data.accelZ;
        sum_gx += imu.data.gyroX;
        sum_gy += imu.data.gyroY;
        sum_gz += imu.data.gyroZ;
        delay(5);
    }

    float mean_ax = sum_ax / samples;
    float mean_ay = sum_ay / samples;
    float var_ax = sq_ax / samples - mean_ax * mean_ax;
    float var_ay = sq_ay / samples - mean_ay * mean_ay;

    if (sqrtf(var_ax) > 0.05f || sqrtf(var_ay) > 0.05f) {
        return false; // 标定时抖动过大
    }

    ax_bias = mean_ax;
    ay_bias = mean_ay;
    az_bias = (sum_az / samples) - 1.0f; // 假设Z轴垂直朝上，静止时应读到1g
    gx_bias = sum_gx / samples;
    gy_bias = sum_gy / samples;
    gz_bias = sum_gz / samples;

    return true;
}

// ---------------------------------------------------------------
// 初始化：I2C -> BMI270 -> 标定 -> 记录初始姿态 -> 启动Core1
// ---------------------------------------------------------------
bool BMI270Rotation::begin(uint8_t sda_pin, uint8_t scl_pin, int calib_samples) {
    if (running) {
        return true; // 已经在跑了，不重复初始化
    }

    sdaPin = sda_pin;
    sclPin = scl_pin;

    Wire.setSDA(sdaPin);
    Wire.setSCL(sclPin);
    Wire.begin();

    if (imu.beginI2C() != BMI2_OK) {
        return false;
    }

    bool ok = false;
    int retry = 0;
    while (!ok && retry < 5) {
        ok = deepCalibrate(calib_samples);
        if (!ok) {
            delay(1000);
            retry++;
        }
    }
    if (!ok) {
        return false; // 5次都因为抖动失败，放弃
    }

    // 用加速度计算一次初始roll/pitch，作为姿态起点（即“记录初始偏移角度”）
    imu.getSensorData();
    float ax = imu.data.accelX - ax_bias;
    float ay = imu.data.accelY - ay_bias;
    float az = imu.data.accelZ - az_bias; // az_bias已经包含了"减去1g基准"，这里不用再+1.0f

    roll  = atan2f(ay, az) * RAD2DEG;
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
    yaw   = 0.0f; // 加速度计测不出yaw，以此刻朝向为0点

    stopRequested = false;
    activeInstance = this;

    multicore_launch_core1(core1_entry_trampoline);
    running = true;
    return true;
}

// ---------------------------------------------------------------
// 销毁：让Core1任务退出，然后彻底复位/释放该核心
// ---------------------------------------------------------------
void BMI270Rotation::end() {
    if (!running) return;

    stopRequested = true;

    // 等 core1_task() 自己退出（最多等500ms，防止死锁卡死）
    for (int i = 0; i < 50; i++) {
        if (!running) break;
        delay(10);
    }

    multicore_reset_core1(); // 复位Core1，硬件层面释放
    activeInstance = nullptr;
    running = false;
}

// ---------------------------------------------------------------
// Core1入口（pico-sdk只能传纯C风格函数指针，这里做个跳板）
// ---------------------------------------------------------------
void BMI270Rotation::core1_entry_trampoline() {
    if (activeInstance) {
        activeInstance->core1_task();
    }
}

// ---------------------------------------------------------------
// 定时器中断回调（100Hz，挂在Core1上）：只置标志位，绝不做I2C操作
// ---------------------------------------------------------------
bool BMI270Rotation::timer_callback_trampoline(struct repeating_timer *t) {
    BMI270Rotation* self = static_cast<BMI270Rotation*>(t->user_data);
    if (self) {
        self->sensorDataReady = true;
    }
    return true; // 返回true表示继续重复触发
}

// ---------------------------------------------------------------
// Core1后台任务：硬件定时器100Hz触发 -> 读取IMU -> 陀螺仪积分 + 互补滤波 -> 更新姿态
// ---------------------------------------------------------------
void BMI270Rotation::core1_task() {
    sensorDataReady = false;

    // 负数周期：确保严格按10ms间隔触发，不受回调本身执行耗时影响（不会累积漂移）
    add_repeating_timer_us(-(int32_t)(DT_S * 1000000.0f), timer_callback_trampoline, this, &timer);

    while (!stopRequested) {
        if (!sensorDataReady) {
            tight_loop_contents(); // 轻量让出，不做实际工作，等中断置位
            continue;
        }
        sensorDataReady = false;

        imu.getSensorData();

        float ax = imu.data.accelX - ax_bias;
        float ay = imu.data.accelY - ay_bias;
        float az = imu.data.accelZ - az_bias; // az_bias已经包含了"减去1g基准"，这里不用再+1.0f

        float gx = imu.data.gyroX - gx_bias; // deg/s
        float gy = imu.data.gyroY - gy_bias;
        float gz = imu.data.gyroZ - gz_bias;

        // --- 静止判定：先对"陀螺仪模长"和"加速度偏离1g的程度"做低通平滑，
        //     再用连续多帧确认(去抖动)，避免单帧噪声导致状态来回跳 ---
        float gyroMag  = sqrtf(gx * gx + gy * gy + gz * gz);
        float accNorm  = sqrtf(ax * ax + ay * ay + az * az);
        float accelDev = fabsf(accNorm - 1.0f);

        smoothGyroMag  = ZUPT_SMOOTH_A * gyroMag  + (1.0f - ZUPT_SMOOTH_A) * smoothGyroMag;
        smoothAccelDev = ZUPT_SMOOTH_A * accelDev + (1.0f - ZUPT_SMOOTH_A) * smoothAccelDev;

        bool instantStill = (smoothGyroMag < ZUPT_GYRO_DPS) && (smoothAccelDev < ZUPT_ACCEL_DEV);

        if (instantStill) {
            stillStreak++;
            moveStreak = 0;
        } else {
            moveStreak++;
            stillStreak = 0;
        }

        bool still = stationaryFlag; // 默认保持上一次状态
        if (stillStreak >= ZUPT_STILL_N) still = true;
        if (moveStreak  >= ZUPT_MOVE_N)  still = false;

        mutex_enter_blocking(&dataMutex);

        // 陀螺仪积分（姿态角持续累加）
        roll  += gx * DT_S;
        pitch += gy * DT_S;
        yaw   += gz * DT_S;

        // 互补滤波：用加速度计修正roll/pitch的漂移（yaw没有可用的参考修正不了）
        float accel_roll  = atan2f(ay, az) * RAD2DEG;
        float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
        roll  = ALPHA_COMP * roll  + (1.0f - ALPHA_COMP) * accel_roll;
        pitch = ALPHA_COMP * pitch + (1.0f - ALPHA_COMP) * accel_pitch;

        lastGx = gx; lastGy = gy; lastGz = gz;
        lastAx = ax; lastAy = ay; lastAz = az;
        stationaryFlag = still;

        mutex_exit(&dataMutex);
    }

    cancel_repeating_timer(&timer); // 停止定时器，避免end()之后残留的回调
    running = false; // 告诉主核：Core1即将退出，end()里的等待循环靠这个跳出
}

// ---------------------------------------------------------------
// 对外读取接口
// ---------------------------------------------------------------
void BMI270Rotation::getRotation(float &x, float &y, float &z) {
    mutex_enter_blocking(&dataMutex);
    x = roll;
    y = pitch;
    z = yaw;
    mutex_exit(&dataMutex);
}

void BMI270Rotation::getGyro(float &gx, float &gy, float &gz) {
    mutex_enter_blocking(&dataMutex);
    gx = lastGx;
    gy = lastGy;
    gz = lastGz;
    mutex_exit(&dataMutex);
}

bool BMI270Rotation::isStationary() {
    mutex_enter_blocking(&dataMutex);
    bool s = stationaryFlag;
    mutex_exit(&dataMutex);
    return s;
}

void BMI270Rotation::getAccel(float &ax, float &ay, float &az) {
    mutex_enter_blocking(&dataMutex);
    ax = lastAx;
    ay = lastAy;
    az = lastAz;
    mutex_exit(&dataMutex);
}