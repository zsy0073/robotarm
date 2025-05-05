/******************************************************
* FileName:      main.cpp
* Date:          2025/05/05
* Description:   机械臂控制程序的主入口文件，使用FreeRTOS架构控制机械臂
*****************************************************/

#include <Arduino.h>
#include "RobotArmFreeRTOS.h"
#include "RobotArmConfig.h"
#include "WebServerController.h"


void setup() {
  // 初始化串口通信
  Serial.begin(SERIAL_BAUDRATE);
  // 移除启动提示
  
  // 初始化FreeRTOS任务和资源
  initRobotArmTasks();
  
  // FreeRTOS任务已经在initRobotArmTasks中创建并运行
  // 包括Web服务器任务
}

void loop() {
  // 使用FreeRTOS架构，主循环基本上是空的
  // 所有功能由各个任务处理
  
  // 为了节省资源，让主循环休眠较长时间
  delay(1000);
}