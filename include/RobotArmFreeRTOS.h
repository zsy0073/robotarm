/******************************************************
* FileName:      RobotArmFreeRTOS.h
* Date:          2025/05/04
* Description:   机械臂项目的FreeRTOS配置与任务声明
*****************************************************/

#ifndef ROBOT_ARM_FREERTOS_H
#define ROBOT_ARM_FREERTOS_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "LobotServoController.h"
#include "RobotArmConfig.h"
#include "RobotArmCommon.h" // 使用共享头文件获取ServoCommandType和ServoCommand_t定义

// 任务优先级定义
#define SERVO_CONTROL_PRIORITY      (3)
#define INPUT_PROCESSING_PRIORITY   (2)
#define WEB_SERVER_PRIORITY         (2)  // Web服务器任务优先级
#define STATUS_MONITOR_PRIORITY     (1)

// 任务栈大小定义
#define SERVO_CONTROL_STACK_SIZE    (4096)
#define INPUT_PROCESSING_STACK_SIZE (2048)
#define WEB_SERVER_STACK_SIZE       (8192)  // Web服务器需要更大的堆栈
#define STATUS_MONITOR_STACK_SIZE   (2048)

// 全局任务句柄
extern TaskHandle_t servoControlTaskHandle;
extern TaskHandle_t inputProcessingTaskHandle;
extern TaskHandle_t webServerTaskHandle;  
extern TaskHandle_t statusMonitorTaskHandle;

// 全局队列句柄
extern QueueHandle_t servoCommandQueue;

// 任务函数声明
void servoControlTask(void *pvParameters);
void inputProcessingTask(void *pvParameters);
void webServerTask(void *pvParameters);  
void statusMonitorTask(void *pvParameters);
void processPS2Input();

// 初始化FreeRTOS任务和资源
bool initRobotArmTasks();

// 舵机控制辅助函数
void sendServoCommand(ServoCommandType type, uint8_t servoID = 0, 
                     uint16_t position = 0, uint16_t time = DEFAULT_MOVE_TIME);
void moveToHomePosition();
void moveAllServos(LobotServo servos[], uint8_t count, uint16_t time);

// 约束舵机角度在有效范围内 - 现在委托给RobotArmController
uint16_t constrainServoPosition(uint8_t servoID, uint16_t position);

// 获取当前舵机位置的函数 - 用于Web界面
uint16_t getCurrentServoPosition(uint8_t servoID);

#endif // ROBOT_ARM_FREERTOS_H