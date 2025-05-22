/******************************************************
* FileName:      SafetyFeatures.h
* Date:          2025/05/12
* Description:   机械臂安全特性相关功能的头文件，包括蜂鸣器提示、安全开关和状态指示灯
*****************************************************/

#ifndef SAFETY_FEATURES_H
#define SAFETY_FEATURES_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 蜂鸣器相关定义
#define BUZZER_PIN            3     // 蜂鸣器连接引脚
#define STARTUP_BEEP_COUNT    3     // 启动时蜂鸣次数
#define BEEP_DURATION        100    // 蜂鸣持续时间(毫秒)
#define BEEP_INTERVAL        100    // 蜂鸣间隔时间(毫秒)

// 安全按键相关定义
#define SAFETY_SWITCH_PIN     4     // 安全按键连接引脚
#define SAFETY_DEBOUNCE_TIME  50    // 按键去抖动时间(毫秒)

// 状态指示灯相关定义 - 每个舵机一个LED
#define BASE_LED_PIN                21    // 底座舵机状态指示灯
#define SHOULDER_PITCH_LED_PIN      47    // 肩部俯仰舵机状态指示灯
#define SHOULDER_ROLL_LED_PIN       48    // 肩部滚转舵机状态指示灯
#define ELBOW_LED_PIN               45    // 肘部舵机状态指示灯
#define WRIST_PITCH_LED_PIN         35    // 腕部俯仰舵机状态指示灯
#define WRIST_ROLL_LED_PIN          36    // 腕部滚转舵机状态指示灯
#define WRIST_YAW_LED_PIN           37    // 腕部偏航舵机状态指示灯
#define GRIPPER_LED_PIN             38    // 夹爪舵机状态指示灯
#define SYSTEM_LED_PIN              39    // 系统状态指示灯

// 任务优先级与栈大小
#define SAFETY_MONITOR_PRIORITY  (3)    // 安全监控任务优先级
#define SAFETY_MONITOR_STACK_SIZE (2048) // 安全监控任务栈大小

// 全局任务句柄声明
extern TaskHandle_t safetyMonitorTaskHandle;

// 蜂鸣器函数声明
void initBuzzer();
void playStartupTone();
void beep(uint16_t duration = BEEP_DURATION);
void beepTimes(uint8_t times, uint16_t duration = BEEP_DURATION, uint16_t interval = BEEP_INTERVAL);

// 安全按键函数声明
void initSafetySwitch();
bool isSafetyLocked();
void lockArmMotion();
void unlockArmMotion();
void setSafetyLockState(bool locked);
bool getSafetyLockState();

// 状态指示灯函数声明
void initStatusLEDs();
void setLEDState(uint8_t pin, bool state);
void updateAxisLEDs();
void blinkLED(uint8_t pin, uint8_t times, uint16_t duration = 100);
void updateSystemLED();

// 安全监控任务
void safetyMonitorTask(void *pvParameters);
void initSafetyFeatures();


#endif // SAFETY_FEATURES_H
