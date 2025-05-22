/******************************************************
* FileName:      SafetyFeatures.cpp
* Date:          2025/05/12
* Description:   机械臂安全特性相关功能的实现，包括蜂鸣器提示、安全开关和状态指示灯
*****************************************************/

#include "SafetyFeatures.h"
#include "RobotArmController.h"
#include "RobotArmCommon.h"

// 全局变量
TaskHandle_t safetyMonitorTaskHandle = NULL;
static bool safetyLocked = true;  // 默认为锁定状态
static unsigned long lastButtonTime = 0;  // 上次按钮按下时间
static unsigned long lastSwitchTime = 0;  // 用于去抖动

// 初始化安全功能
void initSafetyFeatures() {
    // 初始化蜂鸣器
    initBuzzer();
    
    // 初始化安全开关
    initSafetySwitch();
    
    // 初始化状态指示灯
    initStatusLEDs();
    
    // 创建安全监控任务
    xTaskCreatePinnedToCore(
        safetyMonitorTask,             // 任务函数
        "SafetyMonitorTask",           // 任务名称
        SAFETY_MONITOR_STACK_SIZE,     // 堆栈大小
        NULL,                          // 任务参数
        SAFETY_MONITOR_PRIORITY,       // 任务优先级
        &safetyMonitorTaskHandle,      // 任务句柄指针
        0                              // 在Core 0上运行
    );
    
    // 启动时播放提示音
    playStartupTone();
}

/******************************************************
 * 蜂鸣器功能实现
 *****************************************************/

// 初始化蜂鸣器
void initBuzzer() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // 确保蜂鸣器初始状态为关闭
}

// 启动提示音
void playStartupTone() {
    beepTimes(STARTUP_BEEP_COUNT);
}

// 蜂鸣一次
void beep(uint16_t duration) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
}

// 蜂鸣多次
void beepTimes(uint8_t times, uint16_t duration, uint16_t interval) {
    for (uint8_t i = 0; i < times; i++) {
        beep(duration);
        if (i < times - 1) {
            delay(interval);
        }
    }
}

/******************************************************
 * 安全按键功能实现
 *****************************************************/

// 初始化安全按键
void initSafetySwitch() {
    pinMode(SAFETY_SWITCH_PIN, INPUT_PULLUP);
    safetyLocked = true; // 默认为锁定状态
}

// 检查当前安全锁定状态
bool isSafetyLocked() {
    return safetyLocked;
}

// 锁定机械臂移动
void lockArmMotion() {
    safetyLocked = true;
    Serial.println("机械臂已锁定，防止误操作");
    // 发出锁定提示音（短-长）
    beep(100);
    delay(100);
    beep(300);
    
    // 更新系统LED状态
    updateSystemLED();
}

// 解锁机械臂移动
void unlockArmMotion() {
    safetyLocked = false;
    Serial.println("机械臂已解锁，可以操作");
    // 发出解锁提示音（长-短）
    beep(300);
    delay(100);
    beep(100);
    
    // 更新系统LED状态
    updateSystemLED();
}

// 设置安全锁定状态
void setSafetyLockState(bool locked) {
    if (locked != safetyLocked) {
        if (locked) {
            lockArmMotion();
        } else {
            unlockArmMotion();
        }
    }
}

// 获取当前安全锁定状态
bool getSafetyLockState() {
    return safetyLocked;
}

/******************************************************
 * 状态指示灯功能实现
 *****************************************************/

// 初始化状态指示灯
void initStatusLEDs() {
    pinMode(BASE_LED_PIN, OUTPUT);
    pinMode(SHOULDER_PITCH_LED_PIN, OUTPUT);
    pinMode(SHOULDER_ROLL_LED_PIN, OUTPUT);
    pinMode(ELBOW_LED_PIN, OUTPUT);
    pinMode(WRIST_PITCH_LED_PIN, OUTPUT);
    pinMode(WRIST_ROLL_LED_PIN, OUTPUT);
    pinMode(WRIST_YAW_LED_PIN, OUTPUT);
    pinMode(GRIPPER_LED_PIN, OUTPUT);
    pinMode(SYSTEM_LED_PIN, OUTPUT);
    
    // 初始状态全部关闭
    digitalWrite(BASE_LED_PIN, LOW);
    digitalWrite(SHOULDER_PITCH_LED_PIN, LOW);
    digitalWrite(SHOULDER_ROLL_LED_PIN, LOW);
    digitalWrite(ELBOW_LED_PIN, LOW);
    digitalWrite(WRIST_PITCH_LED_PIN, LOW);
    digitalWrite(WRIST_ROLL_LED_PIN, LOW);
    digitalWrite(WRIST_YAW_LED_PIN, LOW);
    digitalWrite(GRIPPER_LED_PIN, LOW);
    
    // 系统LED初始为红色（锁定状态）
    digitalWrite(SYSTEM_LED_PIN, HIGH);
}

// 设置LED状态
void setLEDState(uint8_t pin, bool state) {
    digitalWrite(pin, state ? HIGH : LOW);
}

// 更新轴状态LED
void updateAxisLEDs() {
    // 获取各轴当前状态并更新LED
    // 当舵机在移动时，对应轴的LED会亮起
    
    // 使用RobotArmController来获取舵机状态
    extern RobotArmController armController;
    
    // 为每个舵机单独更新状态指示灯
    setLEDState(BASE_LED_PIN, armController.isServoMoving(BASE_SERVO_ID));
    setLEDState(SHOULDER_PITCH_LED_PIN, armController.isServoMoving(SHOULDER_PITCH_ID));
    setLEDState(SHOULDER_ROLL_LED_PIN, armController.isServoMoving(SHOULDER_ROLL_ID));
    setLEDState(ELBOW_LED_PIN, armController.isServoMoving(ELBOW_SERVO_ID));
    setLEDState(WRIST_PITCH_LED_PIN, armController.isServoMoving(WRIST_PITCH_ID));
    setLEDState(WRIST_ROLL_LED_PIN, armController.isServoMoving(WRIST_ROLL_ID));
    setLEDState(WRIST_YAW_LED_PIN, armController.isServoMoving(WRIST_YAW_ID));
    setLEDState(GRIPPER_LED_PIN, armController.isServoMoving(GRIPPER_SERVO_ID));
}

// LED闪烁函数
void blinkLED(uint8_t pin, uint8_t times, uint16_t duration) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(pin, HIGH);
        delay(duration);
        digitalWrite(pin, LOW);
        if (i < times - 1) {
            delay(duration);
        }
    }
}

// 更新系统LED状态
void updateSystemLED() {
    // 系统LED状态：绿色=解锁，红色=锁定
    if (safetyLocked) {
        // 红色LED（锁定状态）
        digitalWrite(SYSTEM_LED_PIN, HIGH);
    } else {
        // 绿色LED（解锁状态）
        // 在实际应用中可能需要使用RGB LED，这里简化为电平变化
        digitalWrite(SYSTEM_LED_PIN, LOW);
    }
}

/******************************************************
 * 安全监控任务
 *****************************************************/

// 安全监控任务
void safetyMonitorTask(void *pvParameters) {
    bool lastSwitchState = digitalRead(SAFETY_SWITCH_PIN);
    bool buttonPressed = false;  // 标记按钮是否已经被处理
    
    for (;;) {
        // 读取安全按键状态
        bool currentSwitchState = digitalRead(SAFETY_SWITCH_PIN);
          // 检测按钮是否被按下（从高电平变为低电平）
        if (currentSwitchState == LOW && lastSwitchState == HIGH) {
            unsigned long currentTime = millis();
            if (currentTime - lastButtonTime > SAFETY_DEBOUNCE_TIME) {
                // 按钮刚被按下，且经过去抖动处理
                lastButtonTime = currentTime;
                buttonPressed = true;  // 标记为按下状态
            }
        } 
        // 检测按钮是否释放（从低电平变为高电平）
        else if (currentSwitchState == HIGH && lastSwitchState == LOW) {
            unsigned long currentTime = millis();
            if (currentTime - lastButtonTime > SAFETY_DEBOUNCE_TIME) {
                // 按钮已释放，且经过去抖动处理
                lastButtonTime = currentTime;
                
                // 如果之前检测到按钮被按下，现在切换锁定状态
                if (buttonPressed) {
                    // 切换锁定状态（按一下按钮切换一次状态）
                    setSafetyLockState(!safetyLocked);
                    buttonPressed = false;  // 重置按钮状态
                }
            }
        }
        
        // 更新轴状态LED
        updateAxisLEDs();
        
        // 任务休眠
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms刷新一次
    }
}
