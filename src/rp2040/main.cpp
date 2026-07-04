#define ESPLINK_TARGET_ESP8266

#include "sensor/BMI270Rotation.h"
#include "driver/Motordriver.h"
#include "sensor/WheelEncoder.h"
#include "connection/espLink.h"
#include "Arduino.h"
#include "Wire.h"
#include "decetebarrier.h"
#include "config.h"
#include <math.h>

// 行为参数（可根据实际测试效果调整）
static const int   FORWARD_SPEED_PCT   = 50;   // 默认前进速度 (%)
static const int   TURN_SPEED_PCT      = 55;   // 转向时的轮速 (%)
static const float FRONT_STOP_DIST_CM  = 20.0f;// 正前方触发停车距离
static const float CLEAR_DIST_CM       = 15.0f;// 判定"无障碍物"的距离阈值

// 舵机角度约定
static const int   SERVO_FRONT_DEG     = 90;
static const int   SERVO_LEFT_DEG      = 180;
static const int   SERVO_RIGHT_DEG     = 0;

static const float TURN_STEP_DEG       = 83.0f; // 每次确认方向后，先转多少度再重新前进探测
static const float YAW_TOLERANCE_DEG   = 3.0f;  // 转向到位的容差

static const int   MAX_OBSTACLES       = 999;     // 障碍物坐标数组容量

// 左右都有障碍时的后退策略：第一次后退 BACKUP_STEP_CM，
// 若再次左右都被挡住，则后退距离变为 BACKUP_STEP_CM*2，再不行 *3，以此类推，
// 直到某一侧探测到无障碍物为止；一旦成功找到可通行方向，倍数清零重置为1。
static const float BACKUP_STEP_CM      = 5.0f;
static const int   BACKUP_SPEED_PCT    = 35;   // 后退速度 (%)

// 硬件配置
DistanceDetector barrierDetector(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, SERVO_PIN);
WheelEncoder leftEncoder(LM_DO);
BMI270Rotation imuRot;
MotorDriver motor(WHEEL_ENCODER_IN1, WHEEL_ENCODER_IN2, WHEEL_ENCODER_IN3, WHEEL_ENCODER_IN4);
ESPLink esp(ECHIP_MOSI, ECHIP_MISO, ECHIP_SCLK, ECHIP_CS, 100000); // 100kHz，对ESP8266更稳

unsigned long lastLoop = 0;

// 状态机
enum RobotState {
    STATE_FORWARD,       // 正常直行
    STATE_SCAN_LEFT,     // 停车后先看左边
    STATE_SCAN_RIGHT,    // 再看右边
    STATE_TURNING,       // 按选定方向转向中
    STATE_BACKING_UP     // 左右都有障碍，后退一段距离后重新检测
};

RobotState state = STATE_FORWARD;

// 位置与航向（航向直接取 IMU 的 Z 轴，即 yaw，单位：度）
float robotX = 0.0f;
float robotY = 0.0f;

// 转向目标与方向
float turnTargetHeading = 0.0f;
int   turnSign = 1; // +1 = 按"左转"极性, -1 = 按"右转"极性（见 turnBySign 说明）

// 障碍物坐标记录
struct Obstacle {
    float x;
    float y;
};
Obstacle obstacles[MAX_OBSTACLES];
int obstacleCount = 0;

// 后退相关状态
int   backupMultiplier = 1;   // 当前后退倍数（1,2,3...），找到通路后清零重置为1
float backupTargetCm   = 0.0f; // 当前后退目标距离
float backupTraveledCm = 0.0f; // 当前已后退距离

// 工具函数
// 将角度差归一化到 [-180, 180]
float angleDiff(float target, float current) {
    float d = fmod(target - current + 180.0f, 360.0f);
    if (d < 0) d += 360.0f;
    return d - 180.0f;
}

// 根据编码轮脉冲数与航向，累加二维位置（仅在直行状态下调用，转向时轮速不对称，单编码轮无法代表真实位移，故不在转向时累加）
void updateOdometry(float headingDeg) {
    long count = leftEncoder.getCount();
    if (count == 0) return;
    leftEncoder.reset();

    float revolutions = (float)count / (float)WHEEL_BLOCK_COUNT;
    float distanceCm = revolutions * PI * WHEEL_DIAMETER;

    float rad = headingDeg * DEG_TO_RAD;
    robotX += distanceCm * cos(rad);
    robotY += distanceCm * sin(rad);
}

// 记录一个障碍物坐标（世界坐标系），基于当前车身位置 + 航向 + 舵机相对角 + 测得距离
void recordObstacle(float headingDeg, int servoAngle, float distanceCm) {
    if (obstacleCount >= MAX_OBSTACLES) return;

    float relativeAngle = (float)(servoAngle - SERVO_FRONT_DEG); // 相对车头的偏角
    float worldAngleRad = (headingDeg + relativeAngle) * DEG_TO_RAD;

    Obstacle o;
    o.x = robotX + distanceCm * cos(worldAngleRad);
    o.y = robotY + distanceCm * sin(worldAngleRad);
    obstacles[obstacleCount++] = o;

    Serial.print("记录障碍物 #"); Serial.print(obstacleCount);
    Serial.print(" 坐标: ("); Serial.print(o.x, 1);
    Serial.print(", "); Serial.print(o.y, 1); Serial.println(")");
    esp.putln("#" + String(o.x, 2) + "," + String(o.y, 2));
}

// 按符号原地转向：sign=+1 与 sign=-1 分别对应两个转向方向。
// 具体哪个符号对应"向左"取决于电机接线，如实测方向相反，
// 只需把 TURN_SIGN_LEFT 常量改成 -1 即可，无需改动其他逻辑。
static const int TURN_SIGN_LEFT = 1;

void turnBySign(int sign, int speedPct) {
    motor.setLeftSpeed(sign * speedPct);
    motor.setRightSpeed(-sign * speedPct);
}

void startTurn(int sign) {
    turnSign = sign;
    float yaw = 0, dummyX = 0, dummyY = 0;
    imuRot.getRotation(dummyX, dummyY, yaw);
    turnTargetHeading = yaw + sign * TURN_STEP_DEG;
    leftEncoder.reset();
    barrierDetector.aimServo(SERVO_FRONT_DEG); // 转向前先回正探头
    turnBySign(sign, TURN_SPEED_PCT);
    state = STATE_TURNING;
    backupMultiplier = 1;
}

void startBackup() {
    backupTargetCm = BACKUP_STEP_CM * backupMultiplier;
    backupTraveledCm = 0.0f;
    backupMultiplier++;
    leftEncoder.reset();
    barrierDetector.aimServo(SERVO_FRONT_DEG); // 倒车时也看正前方，防止倒车撞上东西
    motor.setLeftSpeed(-BACKUP_SPEED_PCT);
    motor.setRightSpeed(-BACKUP_SPEED_PCT);
    state = STATE_BACKING_UP;
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("正在初始化BMI270，请保持设备绝对静止...");
    delay(1000);
    if (!imuRot.begin(IMU_SDA_PIN, IMU_SCL_PIN, /*calib_samples=*/500)) {
       Serial.println("IMU初始化失败");
    }
    else {
       Serial.println("BMI270 初始化成功");
    }
    motor.begin();
    leftEncoder.begin();
    esp.begin();

    motor.setLeftSpeed(FORWARD_SPEED_PCT);
    motor.setRightSpeed(FORWARD_SPEED_PCT);
}

void loop() {
    float ix, iy, yaw;
    imuRot.getRotation(ix, iy, yaw); // yaw(z) 作为航向角使用

    // 每0.5秒打印一次调试信息
    if (millis() - lastLoop >= 100) {
        lastLoop = millis();
        // Serial.print("Left: "); Serial.print(leftEncoder.getCount());
        // Serial.print(" Yaw(Z):"); Serial.print(yaw, 2);
        // Serial.print(" Pos:("); Serial.print(robotX, 1);
        // Serial.print(","); Serial.print(robotY, 1);
        // Serial.print(") State:"); Serial.println((int)state);
        esp.putln("@" + String(robotX, 2) + "," + String(robotY, 2));// 发送当前位置给ESP8266 保留2位小数
    }

    switch (state) {

    case STATE_FORWARD: {
        updateOdometry(yaw);

        if (barrierDetector.isObjectDetected(FRONT_STOP_DIST_CM, SERVO_FRONT_DEG)) {
            motor.stop();
            leftEncoder.reset();
            recordObstacle(yaw, SERVO_FRONT_DEG, FRONT_STOP_DIST_CM);
            Serial.println("正前方发现障碍物，停车，开始左右探查...");
            state = STATE_SCAN_LEFT;
        }
        break;
    }

    case STATE_SCAN_LEFT: {
        bool leftBlocked = barrierDetector.isObjectDetected(CLEAR_DIST_CM, SERVO_LEFT_DEG);
        if (!leftBlocked) {
            Serial.println("左侧无障碍物，向左转向");
            startTurn(TURN_SIGN_LEFT);
        } else {
            // 左侧仍有障碍：记录坐标后转去看右侧
            // TODO: 若 DistanceDetector 提供的是具体距离而非布尔z值，
            // 请用实际距离替换下面的 CLEAR_DIST_CM
            recordObstacle(yaw, SERVO_LEFT_DEG, CLEAR_DIST_CM);
            Serial.println("左侧仍有障碍物，改为探查右侧");
            state = STATE_SCAN_RIGHT;
        }
        break;
    }

    case STATE_SCAN_RIGHT: {
        bool rightBlocked = barrierDetector.isObjectDetected(CLEAR_DIST_CM, SERVO_RIGHT_DEG);
        if (!rightBlocked) {
            Serial.println("右侧无障碍物，向右转向");
            startTurn(-TURN_SIGN_LEFT);
        } else {
            recordObstacle(yaw, SERVO_RIGHT_DEG, CLEAR_DIST_CM);
            startBackup();
        }
        break;
    }

    case STATE_BACKING_UP: {
        long count = leftEncoder.getCount();
        if (count != 0) {
            leftEncoder.reset();
            float revolutions = (float)count / (float)WHEEL_BLOCK_COUNT;
            float distanceCm = revolutions * PI * WHEEL_DIAMETER;
            backupTraveledCm += distanceCm;

            // 后退时沿航向的反方向移动，位置相应减少
            float rad = yaw * DEG_TO_RAD;
            robotX -= distanceCm * cos(rad);
            robotY -= distanceCm * sin(rad);
        }

        if (backupTraveledCm >= backupTargetCm) {
            motor.stop();
            leftEncoder.reset();
            //motor.setLeftSpeed(FORWARD_SPEED_PCT);
            //motor.setRightSpeed(FORWARD_SPEED_PCT);
            Serial.println("后退完成，恢复直行并重新检测障碍物");
            state = STATE_SCAN_LEFT;
        }
        break;
    }

    case STATE_TURNING: {
        float diff = angleDiff(turnTargetHeading, yaw);
        if (fabs(diff) <= YAW_TOLERANCE_DEG) {
            motor.stop();
            leftEncoder.reset();
            motor.setLeftSpeed(FORWARD_SPEED_PCT);
            motor.setRightSpeed(FORWARD_SPEED_PCT);
            Serial.println("转向完成，恢复直行");
            state = STATE_FORWARD;
        }
        break;
    }

    }
}


//网页测试用loop，模拟小车运动轨迹和障碍物
// void loop() {
//     static bool inited = false;
//     static unsigned long lastRobotSend = 0;
//     static unsigned long lastObstacleSend = 0;
//     static int targetIndex = 0;
//     static float circleAngle = 0.0f;
//     static float circleTraveled = 0.0f;
//     static enum { SIM_MOVE, SIM_CIRCLE } simState = SIM_MOVE;

//     const float ROBOT_SPEED         = 3.0f;                 // 每次更新前进距离(cm)
//     const float CIRCLE_RADIUS       = 15.0f;                // 绕障碍物半径(cm)
//     const float CIRCLE_ANGULAR_STEP = 10.0f * PI / 180.0f;  // 每次绕行角度(弧度)

//     // 首次进入时写入一份模拟障碍物地图，并复位小车位置
//     if (!inited) {
//         inited = true;
//         obstacleCount = 0;
//         obstacles[obstacleCount++] = {60, 0};
//         obstacles[obstacleCount++] = {100, 80};
//         obstacles[obstacleCount++] = {-40, 90};
//         obstacles[obstacleCount++] = {-90, -30};
//         obstacles[obstacleCount++] = {20, -100};
//         robotX = 0;
//         robotY = 0;
//     }

//     unsigned long now = millis();

//     // 小车位置更新 + 发送(150ms一次，模拟连续运动)
//     if (now - lastRobotSend >= 150) {
//         lastRobotSend = now;

//         Obstacle &target = obstacles[targetIndex];

//         if (simState == SIM_MOVE) {
//             float dx = target.x - robotX;
//             float dy = target.y - robotY;
//             float dist = sqrtf(dx * dx + dy * dy);
//             if (dist > CIRCLE_RADIUS) {
//                 float ang = atan2f(dy, dx);
//                 robotX += cosf(ang) * ROBOT_SPEED;
//                 robotY += sinf(ang) * ROBOT_SPEED;
//             } else {
//                 simState = SIM_CIRCLE;
//                 circleAngle = atan2f(robotY - target.y, robotX - target.x);
//                 circleTraveled = 0.0f;
//             }
//         } else { // SIM_CIRCLE
//             circleAngle += CIRCLE_ANGULAR_STEP;
//             circleTraveled += CIRCLE_ANGULAR_STEP;
//             robotX = target.x + CIRCLE_RADIUS * cosf(circleAngle);
//             robotY = target.y + CIRCLE_RADIUS * sinf(circleAngle);
//             if (circleTraveled >= 2.0f * PI) {
//                 simState = SIM_MOVE;
//                 targetIndex = (targetIndex + 1) % obstacleCount;
//             }
//         }

//         Serial.print("Pos:("); Serial.print(robotX, 1);
//         Serial.print(","); Serial.print(robotY, 1); Serial.println(")");
//         esp.putln("@" + String(robotX, 2) + "," + String(robotY, 2));
//     }

//     // 障碍物是静态的，1秒发一次全量即可
//     if (now - lastObstacleSend >= 1000) {
//         lastObstacleSend = now;
//         for (int i = 0; i < obstacleCount; i++) {
//             esp.putln("#" + String(obstacles[i].x, 2) + "," + String(obstacles[i].y, 2));
//         }
//     }
// }