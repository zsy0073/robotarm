/******************************************************
* FileName:      RobotArmCommon.h
* Date:          2025/05/05
* Description:   机械臂项目的公共类型定义
*****************************************************/

#ifndef ROBOT_ARM_COMMON_H
#define ROBOT_ARM_COMMON_H

#include <Arduino.h>
#include "LobotServoController.h"
#include "RobotArmConfig.h"

// 舵机控制命令类型
enum ServoCommandType {
    MOVE_SINGLE_SERVO,  // 移动单个舵机
    MOVE_ALL_SERVOS,    // 移动所有舵机
    MOVE_TO_HOME,       // 回到初始位置
    UNLOAD_SERVOS       // 掉电所有舵机
};

// 舵机控制命令结构体
typedef struct {
    ServoCommandType type;
    uint8_t servoID;
    uint16_t position;
    uint16_t time;
    LobotServo servos[SERVO_COUNT];
    uint8_t servoCount;
} ServoCommand_t;

#endif // ROBOT_ARM_COMMON_H